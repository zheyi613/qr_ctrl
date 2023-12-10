/**
 * @file attitude_control.c
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief attitude control
 * @date 2023-04-26
 */

#include "math.h"

#define YAW_WEIGHT      0.6F

static float inv_sqrt(float x)
{
    float xhalf = 0.5f * x;
    int i = *((int *)&x);           // get bits for floating value
    i = 0x5f375a86 - (i >> 1);      // gives initial guess y0
    x = *((float*)&i);              // convert bits back to float
    x = x * (1.5f - xhalf * x * x); // Newton 1st iteration
    return x;
}

static void inv_quat(float q[4], float q_inv[4])
{
        q_inv[0] = q[0];
        q_inv[1] = -q[1];
        q_inv[2] = -q[2];
        q_inv[3] = -q[3];
}

static void mul_quat(float qa[4], float qb[4], float q[4])
{       
        q[0] = qa[0] * qb[0] - qa[1] * qb[1] - qa[2] * qb[2] - qa[3] * qb[3];
        q[1] = qa[0] * qb[1] + qa[1] * qb[0] + qa[2] * qb[3] - qa[3] * qb[2];
        q[2] = qa[0] * qb[2] - qa[1] * qb[3] + qa[2] * qb[0] + qa[3] * qb[1];
        q[3] = qa[0] * qb[3] + qa[1] * qb[2] - qa[2] * qb[1] + qa[3] * qb[0];
}


static void canonicalize_quat(float q[4])
{
        float sign = 1.f;

        for (int i = 0; i < 4; i++) {
                if (fabsf(q[i]) > 0.0000001f) {
                        sign = (q[i] > 0) ? 1.f: -1.f;
                        q[0] *= sign;
                        q[1] *= sign;
                        q[2] *= sign;
                        q[3] *= sign;
                        
                        break;
                }
        }
}

// static void axis_angle2quat(float x, float y, float z, float angle,
//                             float q[4])
// {
//         float tmp;
        
//         if (angle < 0.000001f) {
//                 q[0] = 1.f;
//                 q[1] = 0.f;
//                 q[2] = 0.f;
//                 q[3] = 0.f;
//         } else {
//                 tmp = sinf(angle * 0.5f);
//                 q[0] = cosf(angle * 0.5f);
//                 q[1] = x * tmp;
//                 q[2] = y * tmp;
//                 q[3] = z * tmp;
//         }
// }

static void quat2dcm_z(float q[4], float v[3])
{
        v[0] = 2.f * (q[1] * q[3] + q[0] * q[2]);
        v[1] = 2.f * (q[2] * q[3] - q[0] * q[1]);
        v[2] = 2.f * (0.5f - q[1] * q[1] - q[2] * q[2]);
}

static void cross_vec(float a[3], float b[3], float c[3])
{
        c[0] = a[1] * b[2] - b[1] * a[2];
        c[1] = b[0] * a[2] - a[0] * b[2];
        c[2] = a[0] * b[1] - b[0] * a[1];
}

static void cross_vec2quat(float a[3], float b[3], float q[4])
{
        float c[3], ab[3];
        float dot, cross_norm_square, recip_norm;

        dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
        cross_vec(a, b, c);
        cross_norm_square = c[0] * c[0] + c[1] * c[1] + c[2] * c[2];
        if (cross_norm_square < 0.000001f && dot < 0.f) { /* angle ~ 180 deg */
                c[0] = fabs(c[0]);
                c[1] = fabs(c[1]);
                c[2] = fabs(c[2]);
                if (c[0] < c[1] && c[0] < c[2]) {
                        q[1] = 0.f;
                        q[2] = a[2];
                        q[3] = -a[1];
                } else if (c[1] < c[0] && c[1] < c[2]) {
                        q[1] = -a[2];
                        q[2] = 0.f;
                        q[3] = a[0];
                } else {
                        q[1] = a[1];
                        q[2] = -a[0];
                        q[3] = 0.f;
                }
                q[0] = 0.f;
        } else {
                ab[0] = a[0] + b[0];
                ab[1] = a[1] + b[1];
                ab[2] = a[2] + b[2];
                recip_norm = inv_sqrt(ab[0] * ab[0] + ab[1] * ab[1] +
                                      ab[2] * ab[2]);
                q[0] = (1 + dot) * recip_norm;
                q[1] = c[0] * recip_norm;
                q[2] = c[1] * recip_norm;
                q[3] = c[2] * recip_norm;
        }
}

void attitude_err(float q[4], float sp_r, float sp_p, float sp_y, float err[3])
{
        float cos_x, sin_x, cos_y, sin_y, cos_z, sin_z;
        float q_inv[4];
        float qd[4]; /* desire (set point) */
        float qe[4]; /* error */
        float ez[3]; /* error of rotation of body z */
        float ezd[3]; /* world z */
        float qe_xy[4];
        float qe_z[4]; 
        /* desired euler to quaternion */
        sp_r *= 0.5;
        sp_p *= 0.5;
        sp_y *= 0.5;
        cos_x = cosf(sp_r);
        sin_x = sinf(sp_r);
        cos_y = cosf(sp_p);
        sin_y = sinf(sp_p);
        cos_z = cosf(sp_y);
        sin_z = sinf(sp_y);
        qd[0] = sin_z * sin_y * sin_x + cos_z * cos_y * cos_x;
        qd[1] = -sin_z * sin_y * cos_x + sin_x * cos_z * cos_y;
        qd[2] = sin_z * sin_x * cos_y + sin_y * cos_z * cos_x;
        qd[3] = sin_z * cos_y * cos_x - sin_y * sin_x * cos_z;
        /* compute error of rotation */
        inv_quat(q, q_inv);
        mul_quat(q_inv, qd, qe);
        /* seperate xy and z rotation */
        quat2dcm_z(qe, ez);
        ezd[0] = 0;
        ezd[1] = 0;
        ezd[2] = 1.f;
        cross_vec2quat(ezd, ez, qe_xy);
        canonicalize_quat(qe_xy);
        inv_quat(qe_xy, q_inv);
        mul_quat(q_inv, qe, qe_z);
        canonicalize_quat(qe_z);

        if (qe_z[0] > 1.f)
                qe_z[0] = 0.9999999f;
        else if (qe_z[0] < -1.f)
                qe_z[0] = -0.9999999f;
        if (qe_z[3] > 1.f)
                qe_z[3] = 0.9999999f;
        else if (qe_z[3] < -1.f)
                qe_z[3] = -0.9999999f;
        /* restrict z error */
        qe_z[0] = cosf(YAW_WEIGHT * acosf(qe_z[0]));
        qe_z[3] = sinf(YAW_WEIGHT * asinf(qe_z[3]));

        err[0] = 2.f * qe_xy[1];
        err[1] = 2.f * qe_xy[2];
        err[2] = 2.f * qe_z[3];
}