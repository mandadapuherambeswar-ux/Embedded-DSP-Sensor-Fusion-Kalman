/**
 * @file mpu6050_dma.h
 * @brief High-Rate MPU-6050 6-DOF IMU Driver Header
 * @author Herambeswar Mandadapu
 */

#ifndef MPU6050_DMA_H
#define MPU6050_DMA_H

#include "quaternion_math.h"
#include <stdint.h>
#include <stdbool.h>

#define MPU6050_I2C_ADDR    0x68

bool MPU6050_Init(void);
void MPU6050_CalibrateGyro(Vector3f_t *out_bias, int sample_count);
bool MPU6050_ReadSensors(Vector3f_t *accel_g, Vector3f_t *gyro_dps);

#endif /* MPU6050_DMA_H */
