/**
 * @file quaternion_math.c
 * @brief Fast Quaternion Kinematics Implementation
 * @author Herambeswar Mandadapu
 */

#include "quaternion_math.h"
#include <math.h>

#define RAD_TO_DEG 57.29577951308232f

float Fast_InvSqrt(float x)
{
    float halfx = 0.5f * x;
    union {
        float f;
        uint32_t i;
    } conv = { .f = x };
    conv.i = 0x5F3759DF - (conv.i >> 1);
    conv.f = conv.f * (1.5f - (halfx * conv.f * conv.f));
    return conv.f;
}

void Quaternion_Identity(Quaternion_t *q)
{
    if (!q) return;
    q->w = 1.0f;
    q->x = 0.0f;
    q->y = 0.0f;
    q->z = 0.0f;
}

void Quaternion_Normalize(Quaternion_t *q)
{
    if (!q) return;
    float norm_sq = q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z;
    if (norm_sq > 0.0f) {
        float inv_norm = Fast_InvSqrt(norm_sq);
        q->w *= inv_norm;
        q->x *= inv_norm;
        q->y *= inv_norm;
        q->z *= inv_norm;
    }
}

void Quaternion_Multiply(const Quaternion_t *q1, const Quaternion_t *q2, Quaternion_t *out)
{
    if (!q1 || !q2 || !out) return;
    out->w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
    out->x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
    out->y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
    out->z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;
}

void Quaternion_ToEuler(const Quaternion_t *q, EulerAngles_t *euler)
{
    if (!q || !euler) return;

    /* Roll (x-axis rotation) */
    float sinr_cosp = 2.0f * (q->w * q->x + q->y * q->z);
    float cosr_cosp = 1.0f - 2.0f * (q->x * q->x + q->y * q->y);
    euler->roll_deg = atan2f(sinr_cosp, cosr_cosp) * RAD_TO_DEG;

    /* Pitch (y-axis rotation) */
    float sinp = 2.0f * (q->w * q->y - q->z * q->x);
    if (fabsf(sinp) >= 1.0f) {
        euler->pitch_deg = copysignf(90.0f, sinp);
    } else {
        euler->pitch_deg = asinf(sinp) * RAD_TO_DEG;
    }

    /* Yaw (z-axis rotation) */
    float siny_cosp = 2.0f * (q->w * q->z + q->x * q->y);
    float cosy_cosp = 1.0f - 2.0f * (q->y * q->y + q->z * q->z);
    euler->yaw_deg = atan2f(siny_cosp, cosy_cosp) * RAD_TO_DEG;
}
