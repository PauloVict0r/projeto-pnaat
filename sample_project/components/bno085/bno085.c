#include "bno085.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "euler.h"
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"

static const char TAG[] = "bno085";

static bno085_dev_t s_dev;
static sh2_Hal_t s_hal;

static sh2_SensorId_t s_enabled_sensor;
static uint32_t s_enabled_interval_us;
static volatile bool s_need_reenable;

static bno085_orientation_t s_last_orientation;

/* Estado para os 3 relatórios completos (rotation vector + gravity + linear accel) */
static bool s_reports_mode;           /* true quando bno085_enable_reports() foi chamada */
static volatile bool s_rv_valid;      /* rotation vector reportou pelo menos uma vez */
static volatile bool s_grav_valid;    /* gravity reportou pelo menos uma vez */
static volatile bool s_laccel_valid;  /* linear acceleration reportou pelo menos uma vez */

static float s_quat_w, s_quat_x, s_quat_y, s_quat_z;
static float s_grav_x, s_grav_y, s_grav_z;
static float s_laccel_x, s_laccel_y, s_laccel_z;
static float s_roll, s_pitch, s_yaw;
static float s_accuracy_rad;
static uint32_t s_timestamp_us;

/* ---- HAL: framing bruto SHTP sobre I2C, sem registrador ---- */

/*
 * O driver I2C legado do ESP-IDF não tem o limite de ~32 bytes por
 * transação que o Wire do Arduino tem, então ao contrário da HAL de
 * referência (Adafruit_BNO08x), não precisamos fatiar a leitura do
 * payload em vários pedaços: lemos o cabeçalho de 4 bytes pra descobrir
 * o tamanho, depois lemos o pacote inteiro (cabeçalho + payload) numa
 * segunda transação só.
 */
static uint8_t s_i2c_rx_buf[SH2_HAL_MAX_TRANSFER_IN];

/*
 * Datasheet oficial (CEVA, BNO080/85/86, secao "Host Interface"): depois
 * do reset o sensor NAO espera escrita nenhuma -- ele prepara o pacote
 * de "advertisement" SHTP e sinaliza isso baixando H_INTN; "the host may
 * begin communicating with the BNO08X after it has asserted H_INTN", e a
 * primeira transacao tem que ser uma LEITURA. Por isso todo o bring-up
 * (toggle de RESET + espera do INT) acontece em bno085_init(), ANTES de
 * chamar sh2_open() -- hal_open() nao faz I2C nenhum, so devolve OK.
 */
static int hal_open(sh2_Hal_t *self) {
    return 0;
}

static void hal_close(sh2_Hal_t *self) {
    if (s_dev.reset_gpio != GPIO_NUM_NC) {
        gpio_set_level(s_dev.reset_gpio, 0);
    }
}

static int hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us) {
    /* shtp_service() chama isso sem condicao nenhuma, a cada tick do
     * service loop -- e a HAL quem tem que respeitar o datasheet ("host
     * pode ler depois que H_INTN foi assertado"). Sem esse gate, a gente
     * fica martelando o barramento com uma leitura de verdade mesmo com
     * o sensor sem nada pra mandar. */
    if (s_dev.int_gpio != GPIO_NUM_NC && gpio_get_level(s_dev.int_gpio) != 0) {
        ESP_LOGD(TAG, "hal_read: INT alto (sem dado), pulando leitura");
        return 0;
    }

    esp_err_t err = i2c_master_read_from_device(s_dev.i2c_port, s_dev.i2c_addr,
                                                 s_i2c_rx_buf, 4, pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "hal_read: falha lendo cabecalho: %s", esp_err_to_name(err));
        return 0;
    }

    uint16_t packet_size = (uint16_t)s_i2c_rx_buf[0] | ((uint16_t)s_i2c_rx_buf[1] << 8);
    packet_size &= ~0x8000; /* bit 15 = continuation, não usamos */

    if (packet_size == 0 || packet_size > len || packet_size > sizeof(s_i2c_rx_buf)) {
        ESP_LOGD(TAG, "hal_read: cabecalho com packet_size=%u (len=%u) -- descartando",
                 packet_size, len);
        return 0;
    }

    err = i2c_master_read_from_device(s_dev.i2c_port, s_dev.i2c_addr,
                                       s_i2c_rx_buf, packet_size, pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "hal_read: falha lendo payload (%u bytes): %s", packet_size,
                 esp_err_to_name(err));
        return 0;
    }

    memcpy(pBuffer, s_i2c_rx_buf, packet_size);
    if (t_us) {
        *t_us = (uint32_t)esp_timer_get_time();
    }
    ESP_LOGD(TAG, "hal_read: pacote de %u bytes recebido", packet_size);
    return (int)packet_size;
}

static int hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
    esp_err_t err = i2c_master_write_to_device(s_dev.i2c_port, s_dev.i2c_addr,
                                                pBuffer, len, pdMS_TO_TICKS(100));
    ESP_LOGD(TAG, "hal_write: %u bytes (report id 0x%02X) -- %s", len,
             len > 0 ? pBuffer[0] : 0, esp_err_to_name(err));
    return (err == ESP_OK) ? (int)len : 0;
}

static uint32_t hal_getTimeUs(sh2_Hal_t *self) {
    return (uint32_t)esp_timer_get_time();
}

/* ---- Callbacks sh2 ---- */

static void event_callback(void *cookie, sh2_AsyncEvent_t *event) {
    ESP_LOGD(TAG, "event_callback: eventId=%lu", (unsigned long)event->eventId);
    if (event->eventId == SH2_RESET) {
        ESP_LOGW(TAG, "BNO085 resetou, relatorios precisam ser reabilitados");
        s_need_reenable = true;
    }
}

static void sensor_callback(void *cookie, sh2_SensorEvent_t *event) {
    sh2_SensorValue_t value;
    if (sh2_decodeSensorEvent(&value, event) != SH2_OK) {
        ESP_LOGD(TAG, "sensor_callback: falha ao decodificar evento");
        return;
    }

    switch (value.sensorId) {
    case SH2_ROTATION_VECTOR: {
        float roll, pitch, yaw;
        /* q_to_ypr(r,i,j,k, *pYaw, *pPitch, *pRoll) */
        q_to_ypr(value.un.rotationVector.real, value.un.rotationVector.i,
                 value.un.rotationVector.j, value.un.rotationVector.k,
                 &yaw, &pitch, &roll);

        /* Estado legado (bno085_get_orientation) */
        s_last_orientation.angle_x = roll * (180.0f / (float)M_PI);
        s_last_orientation.angle_y = pitch * (180.0f / (float)M_PI);
        s_last_orientation.angle_z = yaw * (180.0f / (float)M_PI);
        s_last_orientation.accuracy_rad = value.un.rotationVector.accuracy;
        s_last_orientation.valid = true;

        /* Estado completo (bno085_get_imu_data) */
        s_quat_w = value.un.rotationVector.real;
        s_quat_x = value.un.rotationVector.i;
        s_quat_y = value.un.rotationVector.j;
        s_quat_z = value.un.rotationVector.k;
        s_roll = roll * (180.0f / (float)M_PI);
        s_pitch = pitch * (180.0f / (float)M_PI);
        s_yaw = yaw * (180.0f / (float)M_PI);
        s_accuracy_rad = value.un.rotationVector.accuracy;
        s_timestamp_us = (uint32_t)esp_timer_get_time();
        s_rv_valid = true;
        break;
    }
    case SH2_GRAVITY:
        s_grav_x = value.un.gravity.x;
        s_grav_y = value.un.gravity.y;
        s_grav_z = value.un.gravity.z;
        s_grav_valid = true;
        break;

    case SH2_LINEAR_ACCELERATION:
        s_laccel_x = value.un.linearAcceleration.x;
        s_laccel_y = value.un.linearAcceleration.y;
        s_laccel_z = value.un.linearAcceleration.z;
        s_laccel_valid = true;
        break;

    default:
        ESP_LOGD(TAG, "sensor_callback: sensorId inesperado 0x%02X", value.sensorId);
        break;
    }
}

/* ---- API pública ---- */

esp_err_t bno085_init(const bno085_dev_t *dev) {
    s_dev = *dev;
    memset(&s_last_orientation, 0, sizeof(s_last_orientation));
    s_enabled_sensor = 0;
    s_enabled_interval_us = 0;
    s_need_reenable = false;
    s_reports_mode = false;
    s_rv_valid = false;
    s_grav_valid = false;
    s_laccel_valid = false;

    /* Barramento I2C (dev->i2c_port) já vem inicializado pelo chamador
     * via i2c_config::initialize_i2c() -- este driver só faz device I/O
     * (ver hal_read/hal_write acima), nunca traz o barramento à vida. */

    if (s_dev.reset_gpio != GPIO_NUM_NC) {
        gpio_config_t reset_cfg = {
            .pin_bit_mask = 1ULL << s_dev.reset_gpio,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&reset_cfg);
    }
    if (s_dev.int_gpio != GPIO_NUM_NC) {
        gpio_config_t int_cfg = {
            .pin_bit_mask = 1ULL << s_dev.int_gpio,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&int_cfg);
    }

    /* Reset por hardware: LOW pulsa o reset, HIGH libera. Datasheet
     * pede so >=10ns de pulso -- 10ms e bem generoso/seguro. */
    if (s_dev.reset_gpio != GPIO_NUM_NC) {
        gpio_set_level(s_dev.reset_gpio, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(s_dev.reset_gpio, 1);
    }

    /* Espera H_INTN assertar (nivel baixo) -- so leitura de GPIO, nao
     * pode dar NACK, e distingue "sensor nunca respondeu" (problema de
     * alimentacao/RESET/INT) de "respondeu mas a leitura I2C falhou"
     * (problema de SDA/SCL) de forma bem mais clara que tentar uma
     * escrita as cegas. Timing do datasheet: init interno ~90ms +
     * config ~4ms -- 500ms da bastante folga. */
    if (s_dev.int_gpio != GPIO_NUM_NC) {
        const TickType_t timeout_ticks = pdMS_TO_TICKS(500);
        TickType_t start_tick = xTaskGetTickCount();
        bool int_asserted = false;
        while ((xTaskGetTickCount() - start_tick) < timeout_ticks) {
            if (gpio_get_level(s_dev.int_gpio) == 0) {
                int_asserted = true;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        if (!int_asserted) {
            ESP_LOGE(TAG,
                     "BNO085 nunca assertou INT apos reset (GPIO %d) -- "
                     "confira alimentacao 3V3/GND e a fiacao do "
                     "RESET (GPIO %d)/INT",
                     s_dev.int_gpio, s_dev.reset_gpio);
            return ESP_ERR_TIMEOUT;
        }
    }

    s_hal.open = hal_open;
    s_hal.close = hal_close;
    s_hal.read = hal_read;
    s_hal.write = hal_write;
    s_hal.getTimeUs = hal_getTimeUs;

    int status = sh2_open(&s_hal, event_callback, NULL);
    if (status != SH2_OK) {
        ESP_LOGE(TAG,
                 "sh2_open falhou (%d) -- INT respondeu mas a leitura I2C "
                 "falhou, confira SDA/SCL (porta %d), pull-up e o endereco "
                 "0x%02X (ADR em GND=0x4A, em 3V3=0x4B)",
                 status, s_dev.i2c_port, s_dev.i2c_addr);
        return ESP_FAIL;
    }

    sh2_setSensorCallback(sensor_callback, NULL);

    /* O handshake inicial de sh2_open() acima passa pelo mesmo
     * SH2_RESET que event_callback() usa pra saber que precisa
     * reabilitar relatorios -- mas nesse ponto nada foi habilitado
     * ainda (isso e feito depois, por bno085_enable_rotation_vector()).
     * Zera aqui pra nao disparar um reenable redundante no primeiro
     * bno085_service(). */
    s_need_reenable = false;

    ESP_LOGI(TAG, "BNO085 inicializado (addr 0x%02X)", s_dev.i2c_addr);
    return ESP_OK;
}

esp_err_t bno085_enable_rotation_vector(uint32_t interval_us) {
    sh2_SensorConfig_t config = {0};
    config.reportInterval_us = interval_us;

    ESP_LOGD(TAG, "sh2_setSensorConfig: SH2_ROTATION_VECTOR interval_us=%lu",
             (unsigned long)interval_us);
    int status = sh2_setSensorConfig(SH2_ROTATION_VECTOR, &config);
    if (status != SH2_OK) {
        ESP_LOGE(TAG, "sh2_setSensorConfig falhou: %d", status);
        return ESP_FAIL;
    }

    s_enabled_sensor = SH2_ROTATION_VECTOR;
    s_enabled_interval_us = interval_us;
    s_reports_mode = false;
    return ESP_OK;
}

esp_err_t bno085_enable_reports(uint32_t interval_us) {
    sh2_SensorConfig_t config = {0};
    config.reportInterval_us = interval_us;

    /* 1. Rotation Vector */
    int status = sh2_setSensorConfig(SH2_ROTATION_VECTOR, &config);
    if (status != SH2_OK) {
        ESP_LOGE(TAG, "setSensorConfig ROTATION_VECTOR falhou: %d", status);
        return ESP_FAIL;
    }

    /* 2. Gravity */
    status = sh2_setSensorConfig(SH2_GRAVITY, &config);
    if (status != SH2_OK) {
        ESP_LOGE(TAG, "setSensorConfig GRAVITY falhou: %d", status);
        return ESP_FAIL;
    }

    /* 3. Linear Acceleration */
    status = sh2_setSensorConfig(SH2_LINEAR_ACCELERATION, &config);
    if (status != SH2_OK) {
        ESP_LOGE(TAG, "setSensorConfig LINEAR_ACCELERATION falhou: %d", status);
        return ESP_FAIL;
    }

    s_enabled_sensor = SH2_ROTATION_VECTOR; /* primário para legado */
    s_enabled_interval_us = interval_us;
    s_reports_mode = true;
    ESP_LOGI(TAG, "3 relatorios habilitados (RV+GRAV+LACCEL) a %lu us",
             (unsigned long)interval_us);
    return ESP_OK;
}

void bno085_service(void) {
    ESP_LOGD(TAG, "bno085_service tick");
    sh2_service();

    if (s_need_reenable && s_enabled_interval_us != 0) {
        ESP_LOGW(TAG, "Reabilitando relatorios apos reset assincrono");
        esp_err_t err;
        if (s_reports_mode) {
            err = bno085_enable_reports(s_enabled_interval_us);
        } else {
            err = bno085_enable_rotation_vector(s_enabled_interval_us);
        }
        if (err == ESP_OK) {
            s_need_reenable = false;
        }
    }
}

bool bno085_get_orientation(bno085_orientation_t *out) {
    if (!s_last_orientation.valid) {
        return false;
    }
    *out = s_last_orientation;
    return true;
}

bool bno085_get_imu_data(bno085_imu_data_t *out) {
    if (!s_rv_valid || !s_grav_valid || !s_laccel_valid) {
        return false;
    }

    out->quat_w = s_quat_w;
    out->quat_x = s_quat_x;
    out->quat_y = s_quat_y;
    out->quat_z = s_quat_z;

    out->grav_x = s_grav_x;
    out->grav_y = s_grav_y;
    out->grav_z = s_grav_z;

    out->lin_accel_x = s_laccel_x;
    out->lin_accel_y = s_laccel_y;
    out->lin_accel_z = s_laccel_z;

    out->roll = s_roll;
    out->pitch = s_pitch;
    out->yaw = s_yaw;

    out->accuracy_rad = s_accuracy_rad;
    out->timestamp_us = s_timestamp_us;
    out->valid = true;

    return true;
}
