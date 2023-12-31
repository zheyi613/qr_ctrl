/**
 * @file ahrs.c
 * @author zheyi613 (zheyi880613@gmail.com)
 * @brief Mahony's complementary filter and get initial attitude
 * @date 2023-03-31
 */

#include "ahrs.h"
#include "math.h"

#define DOUBLE_KP    1.F   // P gain governs rate of convergence of accel/mag
#define DOUBLE_KI    0.05F // I gain governs rate of convergence of gyro biases
#define ACC_GAIN     1.F
#define MAG_GAIN     5.F

static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
static float ex_int = 0, ey_int = 0, ez_int = 0; // scaled integral error

#if defined(ROLL_BIAS) && defined(PITCH_BIAS)
static float qb0, qb1, qb2, qb3;

static void set_eulerXYbias(void)
{
        float half_roll = -ROLL_BIAS * 0.5f;
        float half_pitch = -PITCH_BIAS * 0.5f;
        
        qb0 = cosf(half_roll) * cosf(half_pitch);
#ifdef NED_FRAME
        qb1 = sinf(half_roll) * cosf(half_pitch);
        qb2 = cosf(half_roll) * sinf(half_pitch);
        qb3 = -sinf(half_roll) * sinf(half_pitch);
#else
        qb1 = -sinf(half_pitch) * cosf(half_roll);
        qb2 = cosf(half_pitch) * sinf(half_roll);
        qb3 = -sinf(half_pitch) * sinf(half_roll);
#endif
}
#endif

static float inv_sqrt(float x)
{
        float xhalf = 0.5f * x;
        int i = *((int *)&x);           // get bits for floating value
        i = 0x5f375a86 - (i >> 1);      // gives initial guess y0
        x = *((float*)&i);              // convert bits back to float
        x = x * (1.5f - xhalf * x * x); // Newton 1st iteration
        return x;
}

static void normalize_quat(float *a, float *b, float *c, float *d)
{
        float recip_norm;

        recip_norm = inv_sqrt((*a) * (*a) + (*b) * (*b) + (*c) * (*c) +
                              (*d) * (*d));
        *a *= recip_norm;
        *b *= recip_norm;
        *c *= recip_norm;
        *d *= recip_norm;        
}

static void normalize_vec(float *x, float *y, float *z)
{
        float recip_norm;

        recip_norm = inv_sqrt((*x) * (*x) + (*y) * (*y) + (*z) * (*z));
        *x *= recip_norm;
        *y *= recip_norm;
        *z *= recip_norm;
}

static void cross_vec(float ax, float ay, float az,
                             float bx, float by, float bz,
                             float *cx, float *cy, float *cz)
{
        *cx = ay * bz - az * by;
        *cy = az * bx - ax * bz;
        *cz = ax * by - ay * bx;
}

static float dot_vec(float ax, float ay, float az,
                            float bx, float by, float bz)
{
        float dot;

        dot = ax * bx;
        dot += ay * by;
        dot += az * bz;

        return dot;
}

/**
 * @brief Cross two unit vector to get quaternion
 * 
 * @param ax
 * @param ay 
 * @param az 
 * @param bx 
 * @param by 
 * @param bz 
 * @param qa 
 * @param qb 
 * @param qc 
 * @param qd 
 */
static void cross_vec2quat(float ax, float ay, float az,
                           float bx, float by, float bz,
                           float *qa, float *qb, float *qc, float *qd)
{
        float cx, cy, cz, ab_x, ab_y, ab_z;
        float dot, cross_norm_square, recip_norm;

        dot = dot_vec(ax, ay, az, bx, by, bz);
        cross_vec(ax, ay, az, bx, by, bz, &cx, &cy, &cz);
        cross_norm_square = cx * cx + cy * cy + cz * cz;
        if (cross_norm_square < 0.0004f && dot < 0.f) { /* angle ~ 180 deg */
                cx = fabs(cx);
                cy = fabs(cy);
                cz = fabs(cz);
                if (cx < cy && cx < cz) {
                        *qb = 0.f;
                        *qc = az;
                        *qd = -ay;
                } else if (cy < cx && cy < cz) {
                        *qb = -az;
                        *qc = 0.f;
                        *qd = ax;
                } else {
                        *qb = ay;
                        *qc = -ax;
                        *qd = 0.f;
                }
                *qa = 0.f;
        } else {
                ab_x = ax + bx;
                ab_y = ay + by;
                ab_z = az + bz;
                recip_norm = inv_sqrt(ab_x * ab_x + ab_y * ab_y + ab_z * ab_z);
                *qa = (1 + dot) * recip_norm;
                *qb = cx * recip_norm;
                *qc = cy * recip_norm;
                *qd = cz * recip_norm;
                normalize_quat(qa, qb, qc, qd);
        }
}

/**
 * @brief Initialize quaternion of attitude with acceleration and
 *        magnetic field
 * 
 * @param ax 
 * @param ay 
 * @param az 
 * @param mx 
 * @param my 
 * @param mz 
 */
// void ahrs_init(float ax, float ay, float az,
//                float mx, float my, float mz)
// {
//         float y_x, y_y, y_z;
//         float by_x, by_y, by_z;
//         float qz0, qz1, qz2, qz3;
//         float qy0, qy1, qy2, qy3;
// #ifdef NED_FRAME
//         ax = -ax;
//         ay = -ay;
//         az = -az;
// #endif
//         /* normalize measurement data */
//         normalize_vec(&ax, &ay, &az);
//         normalize_vec(&mx, &my, &mz);
//         /* get rotation of world z to body z */
//         cross_vec2quat(ax, ay, az, 0, 0, 1, &qz0, &qz1, &qz2, &qz3);
//         /* rotate body (NED: y/ENU: x) to make world z coincide with body z */
//         by_x = 2.f * (qz1 * qz2 + qz0 * qz3);
//         by_y = 2.f * (0.5f - qz1 * qz1 + qz3 * qz3);
//         by_z = 2.f * (qz2 * qz3 - qz0 * qz1);
//         /* acceleration (wz) cross magnetic field to get world 
//          * (NED: y/ENU: x) */
//         cross_vec(ax, ay, az, mx, my, mz, &y_x, &y_y, &y_z);
//         normalize_vec(&y_x, &y_y, &y_z);
//         /* compute quaternion between world y and body y */
//         cross_vec2quat(y_x, y_y, y_z, by_x, by_y, by_z,
//                        &qy0, &qy1, &qy2, &qy3);
//         /* product two quaternion */
//         q0 = qz0 * qy0 - qz1 * qy1 - qz2 * qy2 - qz3 * qy3;
//         q1 = qz1 * qy0 + qz0 * qy1 - qz3 * qy2 + qz2 * qy3;
//         q2 = qz2 * qy0 + qz3 * qy1 + qz0 * qy2 - qz1 * qy3;
//         q3 = qz3 * qy0 - qz2 * qy1 + qz1 * qy2 + qz0 * qy3;
//         /* normalize quaternion */
//         normalize_quat(&q0, &q1, &q2, &q3);
// #if defined(ROLL_BIAS) && defined(PITCH_BIAS)
//         set_eulerXYbias();
// #endif
// }
/**
 * @brief Initialize quaternion of attitude with acceleration and
 *        magnetic field
 * 
 * @param ax 
 * @param ay 
 * @param az 
 * @param mx 
 * @param my 
 * @param mz 
 */
void ahrs_init(float ax, float ay, float az,
               float mx, float my, float mz)
{
        float qz0, qz1, qz2, qz3;
        float qy0, qy1, qy2, qy3;
        float bzx, bzy, bzz;
        float cx, cy, cz;

        /* normalize measurement data */
        normalize_vec(&ax, &ay, &az);
        normalize_vec(&mx, &my, &mz);
#ifdef NED_FRAME
        ax = -ax;
        ay = -ay;
        az = -az;
#endif
        /* Compute cross vector between world z and mag (world (NED:x / ENU:y))
         * to get vector world (NED:y / ENU: x) */
        cross_vec(ax, ay, az, mx, my, mz, &cx, &cy, &cz);
        normalize_vec(&cx, &cy, &cz);

        /* Compute rotation from body (NED:y / ENU:x) */
#ifdef NED_FRAME
        cross_vec2quat(0, 1, 0, cx, cy, cz, &qy0, &qy1, &qy2, &qy3);
#else
        cross_vec2quat(1, 0, 0, cx, cy, cz, &qy0, &qy1, &qy2, &qy3);
#endif
        /* Get vector of bz after rotating from b to w (NED:y / ENU:x) */
        bzx = 2.f * (qy1 * qy3 - qy0 * qy2);
        bzy = 2.f * (qy2 * qy3 - qy0 * qy1);
        bzz = 2.f * (0.5f - qy1 * qy1 + qy2 * qy2);

        /* Compute rotation from body z to world z after first rotation */
        cross_vec2quat(bzx, bzy, bzz, ax, ay, az, &qz0, &qz1, &qz2, &qz3);

        /* Multiply two rotation (outer rotation = R2 * R1) */
        q0 = qz0 * qy0 - qz1 * qy1 - qz2 * qy2 - qz3 * qy3;
        q1 = qz1 * qy0 + qz0 * qy1 - qz3 * qy2 + qz2 * qy3;
        q2 = qz2 * qy0 + qz3 * qy1 + qz0 * qy2 - qz1 * qy3;
        q3 = qz3 * qy0 - qz2 * qy1 + qz1 * qy2 + qz0 * qy3;
        
        /* Inverse rotation to get rotation from world frame to body frame */
        q1 = -q1;
        q2 = -q2;
        q3 = -q3;

        normalize_quat(&q0, &q1, &q2, &q3);
#if defined(ROLL_BIAS) && defined(PITCH_BIAS)
        set_eulerXYbias();
#endif
}

/**
 * @brief Initialize quaternion of attitude with acceleration
 *        Assume yaw is zero.
 * 
 * @param ax 
 * @param ay 
 * @param az 
 */
void ahrs_init_imu(float ax, float ay, float az)
{
#ifdef NED_FRAME
        ax = -ax;
        ay = -ay;
        az = -az;
#endif
        normalize_vec(&ax, &ay, &az);
        /* get rotation of world z to body z */
        cross_vec2quat(ax, ay, az, 0, 0, 1, &q0, &q1, &q2, &q3);
        normalize_quat(&q0, &q1, &q2, &q3);
#if defined(ROLL_BIAS) && defined(PITCH_BIAS)
        set_eulerXYbias();
#endif
}

/**
 * @brief Update attitude with only gyro data
 * 
 * @param gx gyroscope x (rad/s)
 * @param gy gyroscope y (rad/s)
 * @param gz gyroscope z (rad/s)
 * @param dt time between two measurement
 */
void ahrs_update(float gx, float gy, float gz, float dt)
{
        float qa = q0, qb = q1, qc = q2;

        /* Adjusted gyroscope measurements */
        gx += ex_int;
        gy += ey_int;
        gz += ez_int;
        /* integrate quaternion rate and normalize (1st-order Rouge-Kutta) */
        gx *= (0.5f * dt);
        gy *= (0.5f * dt);
        gz *= (0.5f * dt);
        q0 += (-qb * gx - qc * gy - q3 * gz);
        q1 += (qa * gx + qc * gz - q3 * gy);
        q2 += (qa * gy - qb * gz + q3 * gx);
        q3 += (qa * gz + qb * gy - qc * gx);
        /* normalise quaternion */
        normalize_quat(&q0, &q1, &q2, &q3);
}

/**
 * @brief Update attitute with 9-dof sensor data
 * 
 * @param gx gyroscope x (rad/s)
 * @param gy gyroscope y (rad/s)
 * @param gz gyroscope z (rad/s)
 * @param ax acceleration x
 * @param ay acceleration y
 * @param az acceleration z
 * @param mx magnetometer x
 * @param my magnetometer y
 * @param mz magnetometer z
 * @param dt time between two measurement
 */
void ahrs_update_marg(float gx, float gy, float gz,
                      float ax, float ay, float az,
                      float mx, float my, float mz,
                      float dt)
{
        float a_norm;
        float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
        float halfex, halfey, halfez;
        float vx, vy, vz;
        float dot, proj_x, proj_y, proj_z;
        /* auxiliary variables to reduce number of repeated operations */
        float q0q0 = q0*q0;
        float q0q1 = q0*q1;
        float q0q2 = q0*q2;
        float q0q3 = q0*q3;
#ifdef NED_FRAME
        float q1q1 = q1*q1;
#endif
        float q1q2 = q1*q2;
        float q1q3 = q1*q3;
#ifndef NED_FRAME
        float q2q2 = q2*q2;
#endif
        float q2q3 = q2*q3;
        float q3q3 = q3*q3;

        /* Estimated world z (v) in body frame */
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 + q3q3 - 0.5f;
#ifdef NED_FRAME       /* gravity: (0, 0, -1) */ 
        halfvx = -halfvx;
        halfvy = -halfvy;
        halfvz = -halfvz;
#endif
        a_norm = sqrtf(ax * ax + ay * ay + az * az);
        if ((a_norm < 1.1f) && (a_norm > 0.9f)) {
                normalize_vec(&ax, &ay, &az);
                halfex = (ay * halfvz - az * halfvy) * ACC_GAIN;
                halfey = (az * halfvx - ax * halfvz) * ACC_GAIN;
                // halfez = (ax * halfvy - ay * halfvx) * ACC_GAIN;
        } else {
                halfex = 0;
                halfey = 0;
                halfez = 0;
        }

        normalize_vec(&mx, &my, &mz);

        /* Estimated direction of magnetic field (w) */
#ifdef NED_FRAME
        halfwx = q0q0 + q1q1 - 0.5f;
        halfwy = q1q2 - q0q3;
        halfwz = q1q3 + q0q2;
#else
        halfwx = q1q2 + q0q3;
        halfwy = q0q0 + q2q2 - 0.5f;
        halfwz = q2q3 - q0q1;
#endif
        vx = halfvx * 2.f;
        vy = halfvy * 2.f;
        vz = halfvz * 2.f;
        dot = dot_vec(vx, vy, vz, mx, my, mz);
        proj_x = mx - dot * vx;
        proj_y = my - dot * vy;
        proj_z = mz - dot * vz;
        normalize_vec(&proj_x, &proj_y, &proj_z);
        /* Error is sum of cross product between reference direction of fields
         * and direction measured by sensors */
        // halfex += (proj_y * halfwz - proj_z * halfwy) * MAG_GAIN;
        // halfey += (proj_z * halfwx - proj_x * halfwz) * MAG_GAIN;
        halfez += (proj_x * halfwy - proj_y * halfwx) * MAG_GAIN;
        /* Integral error scaled integral gain */
        ex_int += halfex * DOUBLE_KI * dt;
        ey_int += halfey * DOUBLE_KI * dt;
        ez_int += halfez * DOUBLE_KI * dt;
        /* Adjusted gyroscope measurements */
        gx += DOUBLE_KP * halfex;
        gy += DOUBLE_KP * halfey;
        gz += DOUBLE_KP * halfez;

        ahrs_update(gx, gy, gz, dt);
}

/**
 * @brief Update attitute with 6-dof sensor data
 * 
 * @param gx gyroscope x (rad/s)
 * @param gy gyroscope y (rad/s)
 * @param gz gyroscope z (rad/s)
 * @param ax acceleration x
 * @param ay acceleration y
 * @param az acceleration z
 * @param dt time between two measurement
 */
void ahrs_update_imu(float gx, float gy, float gz,
                     float ax, float ay, float az,
                     float dt)
{
        float a_norm;
        float halfvx, halfvy, halfvz;
        float halfex, halfey, halfez;

        a_norm = sqrtf(ax * ax + ay * ay + az * az);
        if ((a_norm < 1.1f) && (a_norm > 0.9f)) {
                normalize_vec(&ax, &ay, &az);
                halfvx = q1 * q3 - q0 * q2;
                halfvy = q0 * q1 + q2 * q3;
                halfvz = q0 * q0 - 0.5f + q3 * q3;
#ifdef NED_FRAME       /* gravity: (0, 0, -1) */ 
                halfvx = -halfvx;
                halfvy = -halfvy;
                halfvz = -halfvz;
#endif
                halfex = (ay * halfvz - az * halfvy) * ACC_GAIN;
                halfey = (az * halfvx - ax * halfvz) * ACC_GAIN;
                halfez = (ax * halfvy - ay * halfvx) * ACC_GAIN;
                ex_int += halfex * DOUBLE_KI * dt;
                ey_int += halfey * DOUBLE_KI * dt;
                // ez_int += halfez * DOUBLE_KI * dt;
        } else {
                halfex = 0;
                halfey = 0;
                halfez = 0;
        }
        gx += DOUBLE_KP * halfex;
        gy += DOUBLE_KP * halfey;
        gz += DOUBLE_KP * halfez;

        ahrs_update(gx, gy, gz, dt);
}

void ahrs_update_mag(float gx, float gy, float gz,
                     float mx, float my, float mz,
                     float dt)
{
        float halfwx, halfwy, halfwz;
        float halfex, halfey, halfez;
        float vx, vy, vz;
        float dot, proj_x, proj_y, proj_z;
        /* auxiliary variables to reduce number of repeated operations */
        float q0q0 = q0*q0;
        float q0q1 = q0*q1;
        float q0q2 = q0*q2;
        float q0q3 = q0*q3;
#ifdef NED_FRAME
        float q1q1 = q1*q1;
#endif
        float q1q2 = q1*q2;
        float q1q3 = q1*q3;
#ifndef NED_FRAME
        float q2q2 = q2*q2;
#endif
        float q2q3 = q2*q3;
        float q3q3 = q3*q3;

        normalize_vec(&mx, &my, &mz);

        /* Estimated direction of magnetic field (w) */
#ifdef NED_FRAME
        halfwx = q0q0 + q1q1 - 0.5f;
        halfwy = q1q2 - q0q3;
        halfwz = q1q3 + q0q2;
#else
        halfwx = q1q2 + q0q3;
        halfwy = q0q0 + q2q2 - 0.5f;
        halfwz = q2q3 - q0q1;
#endif
        /* Estimated world z (v) in body frame */
        vx = 2.f * (q1q3 - q0q2);
        vy = 2.f * (q0q1 + q2q3);
        vz = 2.f * (q0q0 + q3q3 - 0.5f);
#ifdef NED_FRAME       /* gravity: (0, 0, -1) */ 
        vx = -vx;
        vy = -vy;
        vz = -vz;
#endif
        dot = dot_vec(vx, vy, vz, mx, my, mz);
        proj_x = mx - dot * vx;
        proj_y = my - dot * vy;
        proj_z = mz - dot * vz;
        normalize_vec(&proj_x, &proj_y, &proj_z);
        /* Error is sum of cross product between reference direction of fields
         * and direction measured by sensors */
        halfex = (proj_y * halfwz - proj_z * halfwy) * MAG_GAIN;
        halfey = (proj_z * halfwx - proj_x * halfwz) * MAG_GAIN;
        halfez = (proj_x * halfwy - proj_y * halfwx) * MAG_GAIN;

        /* Integral error scaled integral gain */
        ex_int += halfex * DOUBLE_KI * dt;
        ey_int += halfey * DOUBLE_KI * dt;
        ez_int += halfez * DOUBLE_KI * dt;
        /* Adjusted gyroscope measurements */
        gx += DOUBLE_KP * halfex;
        gy += DOUBLE_KP * halfey;
        gz += DOUBLE_KP * halfez;
        
        ahrs_update(gx, gy, gz, dt);
}
/**
 * @brief get Enler angle in rad/s
 *        ENU frame: ZXY, NED frame: ZYX
 * 
 * @param r roll  (rad/s)
 * @param p pitch (rad/s)
 * @param y yaw   (rad/s)
 */
void ahrs2euler(float *r, float *p, float *y)
{
#if defined(ROLL_BIAS) && defined(PITCH_BIAS)
        float qa, qb, qc, qd;

        qa = q0 * qb0 - q1 * qb1 - q2 * qb2 - q3 * qb3;
        qb = q1 * qb0 + q0 * qb1 - q3 * qb2 + q2 * qb3;
        qc = q2 * qb0 + q3 * qb1 + q0 * qb2 - q1 * qb3;
        qd = q3 * qb0 - q2 * qb1 + q1 * qb2 + q0 * qb3;
        normalize_quat(&qa, &qb, &qc, &qd);
#ifdef NED_FRAME
        *r = atan2f(qa * qb + qc * qd, 0.5f - qb * qb - qc * qc);
        *p = asinf(2.0f * (qa * qc - qb * qd));
        *y = atan2f(qa * qd + qb * qc, 0.5f - qc * qc - qd * qd);
#else
        *r = atan2f(qa * qc - qb * qd, 0.5f - qb * qb - qc * qc);
        *p = asinf(2.0f * (qa * qb + qc * qd));
        *y = atan2f(qa * qd - qb * qc, 0.5f - qb * qb - qd * qd);
#endif
#else
// #ifdef NED_FRAME
        *r = atan2f(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2);
        *p = asinf(2.0f * (q0 * q2 - q1 * q3));
        *y = atan2f(q0 * q3 + q1 * q2, 0.5f - q2 * q2 - q3 * q3);
// #else
//         *r = atan2f(q0 * q2 - q1 * q3, 0.5f - q1 * q1 - q2 * q2);
//         *p = asinf(2.0f * (q0 * q1 + q2 * q3));
//         *y = atan2f(q0 * q3 - q1 * q2, 0.5f - q1 * q1 - q3 * q3);
// #endif
#endif
}

void ahrs2quat(float q[4])
{
#if defined(ROLL_BIAS) && defined(PITCH_BIAS)
        q[0] = q0 * qb0 - q1 * qb1 - q2 * qb2 - q3 * qb3;
        q[1] = q1 * qb0 + q0 * qb1 - q3 * qb2 + q2 * qb3;
        q[2] = q2 * qb0 + q3 * qb1 + q0 * qb2 - q1 * qb3;
        q[3] = q3 * qb0 - q2 * qb1 + q1 * qb2 + q0 * qb3;
        normalize_quat(&q[0], &q[1], &q[2], &q[3]);
#else
        q[0] = q0;
        q[1] = q1;
        q[2] = q2;
        q[3] = q3;
#endif
}

/**
 * @brief Get linear acceleration of world z
 * 
 * @param ax acceleration of body x
 * @param ay acceleration of body y
 * @param az acceleration of body z
 * @return float linear acceleration of world z
 */
float ahrs_world_linear_az(float ax, float ay, float az)
{
   float wz;

   wz = 2 * (q1 * q3 - q0 * q2) * ax;
   wz += 2 * (q2 * q3 + q0 * q1) * ay;
   /* (1 - 2(q1q1 + q2q2)) * az + 1 (gravity) */
   wz += 2 * (0.5f - q1 * q1 - q2 * q2) * az + 1.f;

   return wz;
}