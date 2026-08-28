/**
 * @file ekf_fusion.h
 * @brief Extended Kalman Filter (EKF) Attitude Estimation Engine
 * @author Herambeswar Mandadapu
 */

#ifndef EKF_FUSION_H
#define EKF_FUSION_H

#include "quaternion_math.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    Quaternion_t q;             /* State quaternion [q0, q1, q2, q3] */
    Vector3f_t gyro_bias_dps;   /* Estimated gyroscope bias [bx, by, bz] */
    float P[4][4];              /* State error covariance matrix */
    float Q_angle;              /* Process noise variance for angle */
    float Q_bias;               /* Process noise variance for gyro bias */
    float R_accel;              /* Measurement noise variance for accelerometer */
} EkfContext_t;

void EKF_Init(EkfContext_t *ekf, float q_angle, float q_bias, float r_accel);
void EKF_Predict(EkfContext_t *ekf, const Vector3f_t *gyro_dps, float dt_sec);
void EKF_Update(EkfContext_t *ekf, const Vector3f_t *accel_g);
void EKF_GetEuler(const EkfContext_t *ekf, EulerAngles_t *euler);

#ifdef __cplusplus
}
#endif

#endif /* EKF_FUSION_H */
