#ifndef __IMU_CONFIG_H__
#define __IMU_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

/**
 * @brief Dados completos do IMU transmitidos pela fila imu_queue.
 *
 * Contém quaternion, vetor de gravidade, aceleração linear e ângulos
 * de Euler — tudo necessário para classificação de posição e formação
 * de dataset Edge Impulse.
 */
typedef struct {
    /* Quaternion — orientação absoluta */
    float quat_w;
    float quat_x;
    float quat_y;
    float quat_z;

    /* Vetor de gravidade (m/s²) */
    float grav_x;
    float grav_y;
    float grav_z;

    /* Aceleração linear, sem gravidade (m/s²) */
    float accel_x;
    float accel_y;
    float accel_z;

    /* Euler — para debug/visualização */
    float roll;   /**< graus */
    float pitch;  /**< graus */
    float yaw;    /**< graus */

    uint32_t timestamp_us;
} imu_data_t;

extern QueueHandle_t imu_queue;
extern TaskHandle_t imu_task_handle;

void imu_config_init(i2c_port_t *i2c_port);

#ifdef __cplusplus
}
#endif

#endif // __IMU_CONFIG_H__
