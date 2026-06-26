/* as5600.h */
#ifndef INC_AS5600_H_
#define INC_AS5600_H_

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "stm32g4xx_hal.h"
#include "FOC_utils.h"

/* ── I2C ─────────────────────────────────────────────────────────── */
#define AS5600_I2C_ADDR             0x36U

/* RAW ANGLE register (0x0C-0x0D): no internal filtering → lowest latency */
#define AS5600_REG_RAW_ANGLE_H      0x0CU
#define AS5600_REG_STATUS           0x0BU
#define AS5600_STATUS_MH            (1U << 3)
#define AS5600_STATUS_ML            (1U << 4)
#define AS5600_STATUS_MD            (1U << 5)

/* 12-bit resolution → 4096 counts per revolution */
#define AS5600_RESOLUTION           4096U

/* Pre-computed constant: 360 / 4096 */
#define AS5600_DEG_PER_COUNT        (360.0f / 4096.0f)

/* ── Filter tuning ───────────────────────────────────────────────── */
#define AS5600_ANGLE_FILTER_ALPHA   0.71539f

/* ── Spike / jump rejection ──────────────────────────────────────── */
#define AS5600_MAX_ANGLE_JUMP_DEG   50.0f
#define AS5600_SPIKE_REJECT_COUNT   10

typedef struct {
    uint16_t         rawAngle;
    float            prevRawTheta;
    float            mechTheta;
    uint8_t          spikeCounter;
    uint8_t          initialized;

    volatile uint8_t busy;         /* set before DMA, cleared in callbacks  */
    volatile uint8_t rx_buf[2];    /* DMA target — must be volatile         */
    volatile uint8_t data_ready;   /* set to 1 after first successful read  */
    volatile uint32_t validCount;
    volatile uint32_t rejectCount;
    volatile uint32_t lastUpdateMs;
    volatile uint8_t  lastStatus;

    uint32_t         errorCount;   /* cumulative I2C / DMA errors           */

    I2C_HandleTypeDef *hi2c;
} AS5600_t;

extern AS5600_t as5600;

void AS5600_Config(I2C_HandleTypeDef *hi2c);
uint8_t AS5600_Magnet_OK(void);
void AS5600_Start_DMA(void);
void AS5600_Update_Angle(foc_t *foc, float offset);   /* call from HAL_I2C_MemRxCpltCallback */
void AS5600_Error_Handler(void);        /* call from HAL_I2C_ErrorCallback     */

#endif /* INC_AS5600_H_ */
