#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Includes from hello_world
extern "C" {
#include "i2c_config.h"
#include "oled_setup.h"
#include "oled_printf.h"
#include "imu_config.h"
}

// Include from Edge Impulse
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"

static float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };
const int STRIDE_SIZE = 3;

int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
    memcpy(out_ptr, features + offset, length * sizeof(float));
    return 0;
}

extern "C" void app_main(void) {
    /* Liga o rail Vext (3V3 do OLED e sensores externos) */
    enable_vext_rail();

    /* Inicializa OLED */
    i2c_port_t oled_i2c_port = I2C_NUM_0;
    initialize_i2c(&oled_i2c_port, GPIO_NUM_17, GPIO_NUM_18);
    configure_oled_screen(&oled_i2c_port);
    oled_printf_init(local_disp);

    printf_oled("OLED OK. Iniciando IMU...");

    /* Inicializa BNO085 */
    i2c_port_t imu_i2c_port = I2C_NUM_1;
    imu_config_init(&imu_i2c_port);

    ei_impulse_result_t result = { 0 };
    static int feature_ix = 0;

    while (1) {
        imu_data_t imu_data;
        if (xQueueReceive(imu_queue, &imu_data, portMAX_DELAY) == pdPASS) {
            // Alimentando apenas com os 3 ângulos de Euler (treinados)
            features[feature_ix + 0] = imu_data.roll;
            features[feature_ix + 1] = imu_data.pitch;
            features[feature_ix + 2] = imu_data.yaw;
            
            feature_ix += 3;

            if (feature_ix >= EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
                signal_t features_signal;
                features_signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
                features_signal.get_data = &raw_feature_get_data;

                EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, false);

                if (res == EI_IMPULSE_OK) {
                    float max_val = -1.0f;
                    const char *best_label = "Unknown";
                    
                    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
                        ei_printf(" %s: %.5f\n",
                                  result.classification[i].label,
                                  result.classification[i].value);
                                  
                        if (result.classification[i].value > max_val) {
                            max_val = result.classification[i].value;
                            best_label = result.classification[i].label;
                        }
                    }
                    
                    printf_oled("Estado Atual:\n%s\n%.0f%%", best_label, max_val * 100.0f);
                }

                int elementos_para_mover = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - STRIDE_SIZE;
                memmove(&features[0],
                        &features[STRIDE_SIZE],
                        elementos_para_mover * sizeof(float));
                feature_ix = elementos_para_mover;
            }
        }
    }
}
