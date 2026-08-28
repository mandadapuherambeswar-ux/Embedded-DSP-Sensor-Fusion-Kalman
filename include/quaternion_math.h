/**
 * @file quaternion_math.h
 * @brief High-Speed Embedded Quaternion & Vector Kinematics
 * @author Herambeswar Mandadapu
 */

#ifndef QUATERNION_MATH_H
#define QUATERNION_MATH_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float w;
    float x;
    float y;
    float z;
} Quaternion_t;

typedef struct {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
} EulerAngles_t;

typedef struct {
    float x;
    float y;
    float z;
} Vector3f_t;

void Quaternion_Identity(Quaternion_t *q);
void Quaternion_Normalize(Quaternion_t *q);
void Quaternion_Multiply(const Quaternion_t *q1, const Quaternion_t *q2, Quaternion_t *out);
void Quaternion_ToEuler(const Quaternion_t *q, EulerAngles_t *euler);
float Fast_InvSqrt(float x);

#ifdef __cplusplus
}
#endif

#endif /* QUATERNION_MATH_H */
