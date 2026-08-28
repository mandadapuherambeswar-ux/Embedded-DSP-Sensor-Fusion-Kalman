/**
 * @file main.c
 * @brief Embedded DSP Sensor Fusion Demo Runner
 * @author Herambeswar Mandadapu
 */

#include "ekf_fusion.h"
#include <stdio.h>
#include <unistd.h>

int main(void)
{
    printf("====================================================\n");
    printf(" 🧭 Embedded DSP EKF IMU Sensor Fusion Engine\n");
    printf(" Author: Herambeswar Mandadapu\n");
    printf("====================================================\n\n");

    EkfContext_t ekf;
    EKF_Init(&ekf, 0.001f, 0.003f, 0.03f);

    printf("[INFO] EKF Filter Initialized. Simulating 100Hz IMU stream...\n\n");

    Vector3f_t gyro = { .x = 2.5f, .y = -1.2f, .z = 0.0f }; /* deg/s */
    Vector3f_t accel = { .x = 0.04f, .y = -0.02f, .z = 0.98f }; /* g */
    EulerAngles_t euler;

    for (int step = 0; step < 10; step++) {
        /* Predict step with 10ms delta */
        EKF_Predict(&ekf, &gyro, 0.01f);

        /* Update step with accelerometer gravity vector */
        EKF_Update(&ekf, &accel);

        /* Extract attitude */
        EKF_GetEuler(&ekf, &euler);

        printf("[STEP %02d] Roll: %+6.2f° | Pitch: %+6.2f° | Yaw: %+6.2f° | Quat: [%.3f, %.3f, %.3f, %.3f]\n",
               step + 1, euler.roll_deg, euler.pitch_deg, euler.yaw_deg,
               ekf.q.w, ekf.q.x, ekf.q.y, ekf.q.z);
    }

    printf("\n[SUCCESS] EKF Sensor Fusion validated.\n");
    return 0;
}
