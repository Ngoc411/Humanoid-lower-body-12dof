/* MPU6050.c */
#include "MPU6050.h"
#include <math.h>
#include <string.h>

#define MPU6050_ADDR_68 0xD0
#define MPU6050_ADDR_69 0xD2
#define LSB_G           131.0f
#define LSB_A           16384.0f
#define G_MS2           9.81f
#define CALIB_SAMPLES   500
#define I2C_TIMEOUT_MS  10

#define REG_SMPLRT_DIV  0x19
#define REG_CONFIG      0x1A
#define REG_GYRO_CFG    0x1B
#define REG_ACCEL_CFG   0x1C
#define REG_ACCEL_XOUT  0x3B
#define REG_PWR_MGMT_1  0x6B
#define REG_WHO_AM_I    0x75

#define DT              0.02f
#define ALPHA_ACCEL     0.15f
#define ALPHA_GYRO      0.10f
#define COMP_NOMINAL    0.98f
#define GYRO_BIAS_ALPHA 0.001f
#define MOTION_THRESH   0.05f

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static I2C_HandleTypeDef *mpu_i2c = NULL;
static uint8_t mpu_addr = MPU6050_ADDR_68;

static int16_t ax, ay, az, gx, gy, gz;

static float ax_off, ay_off, az_off;
static float gx_off, gy_off, gz_off;

#if MPU6050_FILTER
static float ax_s1, ay_s1, az_s1;
static float gx_s1, gy_s1, gz_s1;
#endif
static float ax_f,  ay_f,  az_f;
static float gx_f,  gy_f,  gz_f;

static float gx_bias, gy_bias, gz_bias;

static void InvalidateOutputs(MPU6050_Data_t *data)
{
    data->angvel_x = data->angvel_y = data->angvel_z = NAN;
    data->accel_x  = data->accel_y  = data->accel_z  = NAN;
    data->grav_x   = data->grav_y   = data->grav_z   = NAN;
}

static inline float inv_sqrt(float x)
{
    float halfx = 0.5f * x;
    union { float f; uint32_t i; } u = { x };
    u.i = 0x5f3759df - (u.i >> 1);
    u.f *= (1.5f - halfx * u.f * u.f);
    u.f *= (1.5f - halfx * u.f * u.f);
    return u.f;
}

static HAL_StatusTypeDef ReadRaw(void)
{
    if (mpu_i2c == NULL) return HAL_ERROR;

    uint8_t buf[14];
    HAL_StatusTypeDef ret =
        HAL_I2C_Mem_Read(mpu_i2c, mpu_addr, REG_ACCEL_XOUT,
                         1, buf, 14, I2C_TIMEOUT_MS);
    if (ret != HAL_OK) return ret;

    ax = (int16_t)(buf[0]  << 8 | buf[1]);
    ay = (int16_t)(buf[2]  << 8 | buf[3]);
    az = (int16_t)(buf[4]  << 8 | buf[5]);
    gx = (int16_t)(buf[8]  << 8 | buf[9]);
    gy = (int16_t)(buf[10] << 8 | buf[11]);
    gz = (int16_t)(buf[12] << 8 | buf[13]);

    return HAL_OK;
}

#if MPU6050_FILTER
static void ApplyIIR(void)
{
    float raw_ax = ax - ax_off, raw_ay = ay - ay_off, raw_az = az - az_off;
    float raw_gx = gx - gx_off, raw_gy = gy - gy_off, raw_gz = gz - gz_off;

    ax_s1 = ALPHA_ACCEL * raw_ax + (1.0f - ALPHA_ACCEL) * ax_s1;
    ay_s1 = ALPHA_ACCEL * raw_ay + (1.0f - ALPHA_ACCEL) * ay_s1;
    az_s1 = ALPHA_ACCEL * raw_az + (1.0f - ALPHA_ACCEL) * az_s1;
    gx_s1 = ALPHA_GYRO  * raw_gx + (1.0f - ALPHA_GYRO)  * gx_s1;
    gy_s1 = ALPHA_GYRO  * raw_gy + (1.0f - ALPHA_GYRO)  * gy_s1;
    gz_s1 = ALPHA_GYRO  * raw_gz + (1.0f - ALPHA_GYRO)  * gz_s1;

    ax_f = ALPHA_ACCEL * ax_s1 + (1.0f - ALPHA_ACCEL) * ax_f;
    ay_f = ALPHA_ACCEL * ay_s1 + (1.0f - ALPHA_ACCEL) * ay_f;
    az_f = ALPHA_ACCEL * az_s1 + (1.0f - ALPHA_ACCEL) * az_f;
    gx_f = ALPHA_GYRO  * gx_s1 + (1.0f - ALPHA_GYRO)  * gx_f;
    gy_f = ALPHA_GYRO  * gy_s1 + (1.0f - ALPHA_GYRO)  * gy_f;
    gz_f = ALPHA_GYRO  * gz_s1 + (1.0f - ALPHA_GYRO)  * gz_f;
}
#endif

static void ComputeOutputs(MPU6050_Data_t *data)
{
    float wx_dps = gx_f / LSB_G - gx_bias;
    float wy_dps = gy_f / LSB_G - gy_bias;
    float wz_dps = gz_f / LSB_G - gz_bias;

    data->angvel_x = wx_dps * (M_PI / 180.0f);
    data->angvel_y = wy_dps * (M_PI / 180.0f);
    data->angvel_z = wz_dps * (M_PI / 180.0f);

    float motion_dps = fabsf(gx_f / LSB_G) + fabsf(gy_f / LSB_G) + fabsf(gz_f / LSB_G);
    if (motion_dps < MOTION_THRESH)
    {
        gx_bias += GYRO_BIAS_ALPHA * (gx_f / LSB_G - gx_bias);
        gy_bias += GYRO_BIAS_ALPHA * (gy_f / LSB_G - gy_bias);
        gz_bias += GYRO_BIAS_ALPHA * (gz_f / LSB_G - gz_bias);
    }

    data->accel_x = ax_f * (G_MS2 / LSB_A);
    data->accel_y = ay_f * (G_MS2 / LSB_A);
    data->accel_z = az_f * (G_MS2 / LSB_A);

    float sq = ax_f * ax_f + ay_f * ay_f + az_f * az_f;
    if (sq > 0.0f)
    {
        float inv_n  = inv_sqrt(sq);
        data->grav_x = -(ax_f * inv_n);
        data->grav_y = -(ay_f * inv_n);
        data->grav_z = -(az_f * inv_n);
    }

    float accel_mag   = sqrtf(sq) / LSB_A;
    float accel_error = fabsf(accel_mag - 1.0f);
    float comp = (accel_error > 0.3f)
                    ? 0.9999f
                    : COMP_NOMINAL + (1.0f - COMP_NOMINAL) * (accel_error / 0.3f);

    float acc_pitch = atan2f(ay_f, sqrtf(ax_f * ax_f + az_f * az_f));
    float acc_roll  = atan2f(-ax_f, az_f);

    data->pitch = comp * (data->pitch + data->angvel_y * DT) + (1.0f - comp) * acc_pitch;
    data->roll  = comp * (data->roll  + data->angvel_x * DT) + (1.0f - comp) * acc_roll;
    data->yaw  += data->angvel_z * DT;
}

void MPU6050_Init(MPU6050_Data_t *data, I2C_HandleTypeDef *hi2c)
{
    if (data == NULL) return;

    mpu_i2c = hi2c;
    mpu_addr = MPU6050_ADDR_68;
    memset(data, 0, sizeof(MPU6050_Data_t));
    InvalidateOutputs(data);

    ax_off = ay_off = az_off = 0.0f;
    gx_off = gy_off = gz_off = 0.0f;
    gx_bias = gy_bias = gz_bias = 0.0f;
#if MPU6050_FILTER
    ax_s1 = ay_s1 = az_s1 = 0.0f;
    gx_s1 = gy_s1 = gz_s1 = 0.0f;
#endif
    ax_f  = ay_f  = az_f  = 0.0f;
    gx_f  = gy_f  = gz_f  = 0.0f;

    uint8_t check = 0;
    HAL_StatusTypeDef who_ret =
        HAL_I2C_Mem_Read(mpu_i2c, MPU6050_ADDR_68, REG_WHO_AM_I, 1, &check, 1, I2C_TIMEOUT_MS);

    if (who_ret != HAL_OK || check != 0x68)
    {
        uint8_t check_alt = 0;
        HAL_StatusTypeDef who_alt_ret =
            HAL_I2C_Mem_Read(mpu_i2c, MPU6050_ADDR_69, REG_WHO_AM_I, 1, &check_alt, 1, I2C_TIMEOUT_MS);

        if (who_alt_ret == HAL_OK && check_alt == 0x68)
        {
            mpu_addr = MPU6050_ADDR_69;
            check = check_alt;
            who_ret = who_alt_ret;
        }
    }

    data->dbg_who_status = (uint8_t)who_ret;
    data->dbg_i2c_addr = mpu_addr;

    if (who_ret != HAL_OK)
        return;
    data->dbg_who_am_i = check;

    if (check != 0x68) return;

    uint8_t reg;

    reg = 0x80;
    data->dbg_step_reset = HAL_I2C_Mem_Write(mpu_i2c, mpu_addr, REG_PWR_MGMT_1, 1, &reg, 1, I2C_TIMEOUT_MS);
    HAL_Delay(100);

    reg = 0x01;
    data->dbg_step_clk = HAL_I2C_Mem_Write(mpu_i2c, mpu_addr, REG_PWR_MGMT_1, 1, &reg, 1, I2C_TIMEOUT_MS);
    HAL_Delay(10);

#if MPU6050_FILTER
    reg = 0x02;  /* DLPF_CFG=2: accel 94 Hz, gyro 98 Hz */
#else
    reg = 0x00;  /* DLPF disabled: accel 260 Hz, gyro 256 Hz */
#endif
    data->dbg_step_dlpf = HAL_I2C_Mem_Write(mpu_i2c, mpu_addr, REG_CONFIG, 1, &reg, 1, I2C_TIMEOUT_MS);

#if MPU6050_FILTER
    reg = 0x04;  /* gyro output 1 kHz → 1000/(1+4) = 200 Hz */
#else
    reg = 0x27;  /* gyro output 8 kHz → 8000/(1+39) = 200 Hz */
#endif
    data->dbg_step_smplrt = HAL_I2C_Mem_Write(mpu_i2c, mpu_addr, REG_SMPLRT_DIV, 1, &reg, 1, I2C_TIMEOUT_MS);

    reg = 0x00;
    data->dbg_step_gyro_cfg = HAL_I2C_Mem_Write(mpu_i2c, mpu_addr, REG_GYRO_CFG, 1, &reg, 1, I2C_TIMEOUT_MS);

    reg = 0x00;
    data->dbg_step_accel_cfg = HAL_I2C_Mem_Write(mpu_i2c, mpu_addr, REG_ACCEL_CFG, 1, &reg, 1, I2C_TIMEOUT_MS);

    data->dbg_init_ok = (data->dbg_step_reset     == HAL_OK &&
                         data->dbg_step_clk       == HAL_OK &&
                         data->dbg_step_dlpf      == HAL_OK &&
                         data->dbg_step_smplrt    == HAL_OK &&
                         data->dbg_step_gyro_cfg  == HAL_OK &&
                         data->dbg_step_accel_cfg == HAL_OK);
}

void MPU6050_Calibrate(MPU6050_Data_t *data)
{
    if (data == NULL || data->dbg_init_ok == 0) return;

    int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
    uint16_t ok = 0;

    HAL_Delay(500);

    for (int i = 0; i < CALIB_SAMPLES; i++)
    {
        if (ReadRaw() == HAL_OK)
        {
            ok++;
            ax_sum += ax; ay_sum += ay; az_sum += az;
            gx_sum += gx; gy_sum += gy; gz_sum += gz;
        }
        HAL_Delay(2);
    }

    data->dbg_calib_ok_count = ok;
    if (ok == 0) return;

    ax_off = (float)ax_sum / ok;
    ay_off = (float)ay_sum / ok;
    az_off = (float)az_sum / ok - LSB_A;
    gx_off = (float)gx_sum / ok;
    gy_off = (float)gy_sum / ok;
    gz_off = (float)gz_sum / ok;

    gx_bias = 0.0f;
    gy_bias = 0.0f;
    gz_bias = 0.0f;
}

void MPU6050_Update(MPU6050_Data_t *data)
{
    if (data == NULL) return;

    data->dbg_update_count++;

    if (data->dbg_init_ok == 0)
    {
        data->dbg_read_err_count++;
        InvalidateOutputs(data);
        return;
    }

    if (ReadRaw() != HAL_OK)
    {
        data->dbg_read_err_count++;
        InvalidateOutputs(data);
        return;
    }

#if MPU6050_FILTER
    ApplyIIR();
#else
    ax_f = ax - ax_off; ay_f = ay - ay_off; az_f = az - az_off;
    gx_f = gx - gx_off; gy_f = gy - gy_off; gz_f = gz - gz_off;
#endif
    ComputeOutputs(data);
}
