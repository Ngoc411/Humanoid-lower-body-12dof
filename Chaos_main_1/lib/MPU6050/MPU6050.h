/* MPU6050.h */
#ifndef MPU6050_H
#define MPU6050_H

/* Set to 1 to enable IIR low-pass filtering on accel/gyro data, 0 to use raw data */
#ifndef MPU6050_FILTER
#define MPU6050_FILTER 0
#endif

#include "main.h"
#include <stdint.h>

typedef struct
{
    /* ── Outputs ─────────────────────────────────────────────────────────── */
    float angvel_x, angvel_y, angvel_z;   /* rad/s   — body-frame angular velocity  */
    float grav_x,   grav_y,   grav_z;     /* [-1,1]  — projected gravity, normalised: +9.81→-1, -9.81→+1 */
    float accel_x,  accel_y,  accel_z;    /* m/s²    — filtered linear acceleration */
    float pitch, roll, yaw;               /* rad     — Euler angles                 */

    /* ── Debug: Init steps ───────────────────────────────────────────────
     * dbg_who_am_i  : raw WHO_AM_I value — must be 0x68, else sensor missing
     * dbg_who_status: HAL status of WHO_AM_I read (0=OK 1=ERROR 3=TIMEOUT)
     * dbg_i2c_addr  : active HAL I2C address (0xD0 for 0x68, 0xD2 for 0x69)
     * dbg_step_*    : HAL_StatusTypeDef (0=OK 1=ERROR 3=TIMEOUT)
     *                 first non-zero = where Init failed
     * dbg_init_ok   : 1 = Init completed all steps successfully           */
    uint8_t  dbg_who_am_i;
    uint8_t  dbg_who_status;
    uint8_t  dbg_i2c_addr;
    uint8_t  dbg_step_reset;
    uint8_t  dbg_step_clk;
    uint8_t  dbg_step_dlpf;
    uint8_t  dbg_step_smplrt;
    uint8_t  dbg_step_gyro_cfg;
    uint8_t  dbg_step_accel_cfg;
    uint8_t  dbg_init_ok;

    /* ── Debug: Calibration ──────────────────────────────────────────────
     * dbg_calib_ok_count: successful reads out of CALIB_SAMPLES (500)
     * 0   = sensor not responding at all
     * 500 = all reads OK (nominal)                                        */
    uint16_t dbg_calib_ok_count;

    /* ── Debug: Runtime ──────────────────────────────────────────────────
     * dbg_update_count   : total MPU6050_Update() calls
     * dbg_read_err_count : ReadRaw() failures (HAL != OK)                 */
    uint32_t dbg_update_count;
    uint32_t dbg_read_err_count;

} MPU6050_Data_t;

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */
void MPU6050_Init(MPU6050_Data_t *data, I2C_HandleTypeDef *hi2c);
void MPU6050_Calibrate(MPU6050_Data_t *data);
void MPU6050_Update(MPU6050_Data_t *data);

#endif /* MPU6050_H */
