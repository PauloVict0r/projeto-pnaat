#include "imu_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "freertos/task.h"
#include "freertos/FreeRTOS.h"

#include "bno085.h"
#include "i2c_config.h"
#include "oled_printf.h"

static const char TAG[] = "imu_config";

TaskHandle_t imu_task_handle = NULL;
QueueHandle_t imu_queue;

void imu_task_code(void *pvParameter) {
 TickType_t tick_period = pdMS_TO_TICKS(200);
 TickType_t last_wake_tick = xTaskGetTickCount();

 while (1) {
   bno085_service();

   bno085_imu_data_t imu;
   if (bno085_get_imu_data(&imu)) {
      imu_data_t current_imu_data;

      /* Quaternion */
      current_imu_data.quat_w = imu.quat_w;
      current_imu_data.quat_x = imu.quat_x;
      current_imu_data.quat_y = imu.quat_y;
      current_imu_data.quat_z = imu.quat_z;

      /* Gravidade */
      current_imu_data.grav_x = imu.grav_x;
      current_imu_data.grav_y = imu.grav_y;
      current_imu_data.grav_z = imu.grav_z;

      /* Aceleração linear */
      current_imu_data.accel_x = imu.lin_accel_x;
      current_imu_data.accel_y = imu.lin_accel_y;
      current_imu_data.accel_z = imu.lin_accel_z;

      /* Euler */
      current_imu_data.roll  = imu.roll;
      current_imu_data.pitch = imu.pitch;
      current_imu_data.yaw   = imu.yaw;

      current_imu_data.timestamp_us = imu.timestamp_us;

      /* Saída para Edge Impulse data forwarder (apenas números separados por vírgula) */
      printf("%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f\n",
             imu.quat_w, imu.quat_x, imu.quat_y, imu.quat_z,
             imu.grav_x, imu.grav_y, imu.grav_z,
             imu.lin_accel_x, imu.lin_accel_y, imu.lin_accel_z,
             imu.roll, imu.pitch, imu.yaw);

      /* Exibição resumida no OLED (128x64, ~21 chars/linha) */
      printf_oled(
          "R:%5.1f P:%5.1f Y:%5.1f\n"
          "G:%5.1f %5.1f %5.1f\n"
          "A:%5.1f %5.1f %5.1f",
          imu.roll, imu.pitch, imu.yaw,
          imu.grav_x, imu.grav_y, imu.grav_z,
          imu.lin_accel_x, imu.lin_accel_y, imu.lin_accel_z);

      if (imu_queue != NULL) {
        xQueueOverwrite(imu_queue, &current_imu_data);
      }

   } else {
     ESP_LOGI(TAG, "BNO085 sem leitura completa ainda...");
   }

   vTaskDelayUntil(&last_wake_tick, tick_period);
 }
}

void imu_config_init(i2c_port_t *i2c_port) {
 ESP_LOGI(TAG, "Initializing IMU...");

 /* Fila de tamanho 1 com xQueueOverwrite: sempre contém o dado mais recente */
 imu_queue = xQueueCreate(1, sizeof(imu_data_t));
  if (imu_queue == NULL) {
      ESP_LOGE(TAG, "Falha ao criar a fila imu_queue!");
      return;
  }

 i2c_port_t bno_i2c_port = I2C_NUM_1;
 initialize_i2c(&bno_i2c_port, CONFIG_BNO085_SDA_GPIO, CONFIG_BNO085_SCL_GPIO);

 bno085_dev_t dev = {
     .i2c_port = bno_i2c_port,
     .i2c_addr = CONFIG_BNO085_I2C_ADDR,
     .reset_gpio = CONFIG_BNO085_RESET_GPIO,
     .int_gpio = CONFIG_BNO085_INT_GPIO,
 };
 /* Sensor externo em bancada -- fio solto/mau contato na primeira
   * tentativa e funciona na segunda é comum, por isso algumas tentativas

   * antes de desistir de vez. */

  const int max_attempts = 3;

  esp_err_t err = ESP_FAIL;

  for (int attempt = 1; attempt <= max_attempts; attempt++) {

    err = bno085_init(&dev);

    if (err == ESP_OK) {

      break;

    }

    ESP_LOGW(TAG, "IMU init tentativa %d/%d falhou: %s", attempt, max_attempts,

              esp_err_to_name(err));

    if (attempt < max_attempts) {

      vTaskDelay(pdMS_TO_TICKS(500));

    }

  }

  if (err != ESP_OK) {

    ESP_LOGE(TAG, "Error in initializing IMU: %s", esp_err_to_name(err));

    return;

  }

  /* Habilita os 3 relatórios: rotation vector + gravity + linear acceleration */
  err = bno085_enable_reports(100000); /* 100ms = 10Hz */
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error enabling IMU reports: %s", esp_err_to_name(err));
    return;
  }


  ESP_LOGI(TAG, "IMU initialized");

  xTaskCreate(imu_task_code, "imu_task_code", 5 * 1024, NULL, 5,
              &imu_task_handle);
}

#ifdef __cplusplus
}
#endif