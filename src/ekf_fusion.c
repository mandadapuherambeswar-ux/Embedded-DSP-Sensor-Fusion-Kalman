/**
 * @file ekf_fusion.c
 * @brief Extended Kalman Filter Sensor Fusion Implementation
 * @author Herambeswar Mandadapu
 */

#include "ekf_fusion.h"
#include <string.h>
#include <math.h>

#define DEG_TO_RAD 0.017453292519943295f

void EKF_Init(EkfContext_t *ekf, float q_angle, float q_bias, float r_accel)
{
    if (!ekf) return;
    memset(ekf, 0, sizeof(EkfContext_t));
    Quaternion_Identity(&ekf->q);
    ekf->Q_angle = q_angle;
    ekf->Q_bias = q_bias;
    ekf->R_accel = r_accel;

    for (int i = 0; i < 4; i++) {
        ekf->P[i][i] = 1.0f;
    }
}

void EKF_Predict(EkfContext_t *ekf, const Vector3f_t *gyro_dps, float dt_sec)
{
    if (!ekf || !gyro_dps) return;

    /* Gyro rates in rad/s with bias compensation */
    float gx = (gyro_dps->x - ekf->gyro_bias_dps.x) * DEG_TO_RAD;
    float gy = (gyro_dps->y - ekf->gyro_bias_dps.y) * DEG_TO_RAD;
    float gz = (gyro_dps->z - ekf->gyro_bias_dps.z) * DEG_TO_RAD;

    /* First-order Quaternion integration: q_dot = 0.5 * q * omega */
    float half_dt = 0.5f * dt_sec;
    float qw = ekf->q.w;
    float qx = ekf->q.x;
    float qy = ekf->q.y;
    float qz = ekf->q.z;

    ekf->q.w += half_dt * (-qx * gx - qy * gy - qz * gz);
    ekf->q.x += half_dt * ( qw * gx + qy * gz - qz * gy);
    ekf->q.y += half_dt * ( qw * gy - qx * gz + qz * gx);
    ekf->q.z += half_dt * ( qw * gz + qx * gy - qy * gx);

    Quaternion_Normalize(&ekf->q);

    /* Propagate State Covariance P = F*P*F' + Q */
    for (int i = 0; i < 4; i++) {
        ekf->P[i][i] += ekf->Q_angle * dt_sec;
    }
}

void EKF_Update(EkfContext_t *ekf, const Vector3f_t *accel_g)
{
    if (!ekf || !accel_g) return;

    /* Validate non-zero acceleration norm */
    float norm_sq = accel_g->x * accel_g->x + accel_g->y * accel_g->y + accel_g->z * accel_g->z;
    if (norm_sq < 0.2f || norm_sq > 4.0f) {
        return; /* Reject dynamic acceleration outliers */
    }

    float inv_norm = Fast_InvSqrt(norm_sq);
    float ax = accel_g->x * inv_norm;
    float ay = accel_g->y * inv_norm;
    float az = accel_g->z * inv_norm;

    /* Estimated gravity direction from state quaternion */
    float qw = ekf->q.w;
    float qx = ekf->q.x;
    float qy = ekf->q.y;
    float qz = ekf->q.z;

    float vx = 2.0f * (qx * qz - qw * qy);
    float vy = 2.0f * (qw * qx + qy * qz);
    float vz = qw * qw - qx * qx - qy * qy + qz * qz;

    /* Error vector: cross product between measured and estimated gravity */
    float ex = (ay * vz - az * vy);
    float ey = (az * vx - ax * vz);
    float ez = (ax * vy - ay * vx);

    /* Kalman Innovation Gain K */
    float K_gain = 0.05f;

    ekf->q.w += K_gain * (-qx * ex - qy * ey - qz * ez);
    ekf->q.x += K_gain * ( qw * ex + qy * ez - qz * ey);
    ekf->q.y += K_gain * ( qw * ey - qx * ez + qz * ex);
    ekf->q.z += K_gain * ( qw * ez + qx * ey - qy * ex);

    Quaternion_Normalize(&ekf->q);
}

void EKF_GetEuler(const EkfContext_t *ekf, EulerAngles_t *euler)
{
    if (!ekf || !euler) return;
    Quaternion_ToEuler(&ekf->q, euler);
}
