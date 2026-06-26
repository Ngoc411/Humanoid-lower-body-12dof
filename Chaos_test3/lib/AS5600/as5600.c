/* as5600.c */
#include "as5600.h"

AS5600_t as5600;

/* ── Helpers ─────────────────────────────────────────────────────── */

/* Wrap angle to [-180, +180). Uses fmodf — typically 1 instruction on
   Cortex-M4F, avoids the floorf→multiply→subtract chain. */
static inline float wrap_pm_180(float deg) {
    deg = fmodf(deg + 180.0f, 360.0f);
    if (deg < 0.0f) deg += 360.0f;
    return deg - 180.0f;
}

/* ── Public API ──────────────────────────────────────────────────── */

void AS5600_Config(I2C_HandleTypeDef *hi2c) {
    as5600.rawAngle     = 0;
    as5600.prevRawTheta = 0.0f;
    as5600.mechTheta    = 0.0f;
    as5600.spikeCounter = 0;
    as5600.initialized  = 0;
    as5600.busy         = 0;
    as5600.data_ready   = 0;
    as5600.validCount   = 0;
    as5600.rejectCount  = 0;
    as5600.lastUpdateMs = 0;
    as5600.lastStatus   = 0;
    as5600.errorCount   = 0;
    as5600.rx_buf[0]    = 0;
    as5600.rx_buf[1]    = 0;
    as5600.hi2c         = hi2c;
}

uint8_t AS5600_Magnet_OK(void) {
    uint8_t status = 0;

    for (uint8_t attempt = 0; attempt < 10U; attempt++) {
        if (HAL_I2C_GetState(as5600.hi2c) != HAL_I2C_STATE_READY) {
            HAL_Delay(1);
            continue;
        }

        if (HAL_I2C_Mem_Read(as5600.hi2c,
                             (uint16_t)(AS5600_I2C_ADDR << 1),
                             AS5600_REG_STATUS,
                             I2C_MEMADD_SIZE_8BIT,
                             &status, 1, 5) == HAL_OK) {
            as5600.lastStatus = status;
            return ((status & AS5600_STATUS_MD) != 0U) &&
                   ((status & (AS5600_STATUS_ML | AS5600_STATUS_MH)) == 0U);
        }

        as5600.errorCount++;
        HAL_Delay(1);
    }

    return 0;
}

/*
 * Kick off a non-blocking 2-byte read of RAW ANGLE (0x0C–0x0D).
 * Using RAW ANGLE instead of ANGLE (0x0E) avoids the AS5600 internal
 * slow/fast filter → lower latency, better for high-rate FOC loops.
 */
void AS5600_Start_DMA(void) {
    if (as5600.busy) return;

    /* Non-blocking guard: skip if HAL still processing a previous transaction.
       Without this, HAL_I2C_Mem_Read_DMA polls BUSY for up to 25 ms — fatal
       when called from an ISR (e.g. 10 kHz ADC callback). */
    if (HAL_I2C_GetState(as5600.hi2c) != HAL_I2C_STATE_READY) return;

    as5600.busy = 1;

    if (HAL_I2C_Mem_Read_DMA(as5600.hi2c,
                              (uint16_t)(AS5600_I2C_ADDR << 1),
                              AS5600_REG_RAW_ANGLE_H,
                              I2C_MEMADD_SIZE_8BIT,
                              (uint8_t *)as5600.rx_buf, 2) != HAL_OK) {
        as5600.busy = 0;
        as5600.errorCount++;
    }
}

/*
 * Called from HAL_I2C_MemRxCpltCallback() when hi2c matches.
 * Runs in ISR context — keep it lean.
 */
void AS5600_Update_Angle(foc_t *foc, float offset) {
    as5600.busy = 0;

    /* ── 1. Reconstruct 12-bit raw angle ────────────────────────── */
    uint8_t b0 = as5600.rx_buf[0];   /* read volatile once */
    uint8_t b1 = as5600.rx_buf[1];
    as5600.rawAngle = ((uint16_t)(b0 & 0x0F) << 8) | b1;

    float rawTheta = (float)as5600.rawAngle * AS5600_DEG_PER_COUNT;

    if (!as5600.initialized) {
        as5600.prevRawTheta = rawTheta;
        as5600.mechTheta = rawTheta;
        as5600.spikeCounter = 0;
        as5600.initialized = 1;
    }

    /* ── 2. Spike / jump rejection ──────────────────────────────── */
    float rawDiff = wrap_pm_180(rawTheta - as5600.prevRawTheta);
    if (fabsf(rawDiff) > AS5600_MAX_ANGLE_JUMP_DEG) {
        if (++as5600.spikeCounter < AS5600_SPIKE_REJECT_COUNT) {
            as5600.rejectCount++;
            return;                   /* discard spike */
        }
        as5600.spikeCounter = 0;      /* real fast move — accept */
    } else {
        as5600.spikeCounter = 0;
    }
    as5600.prevRawTheta = rawTheta;

    /* ── 3. IIR low-pass filter (wrap-aware) ────────────────────── */
    /* mechTheta is the filter state — never modify it with offset */
    float filtDiff = wrap_pm_180(rawTheta - as5600.mechTheta);
    as5600.mechTheta += AS5600_ANGLE_FILTER_ALPHA * filtDiff;

    /* Normalise filter state to [0, 360) */
    if (as5600.mechTheta >= 360.0f)
        as5600.mechTheta -= 360.0f;
    else if (as5600.mechTheta < 0.0f)
        as5600.mechTheta += 360.0f;

    /* ── 4. Apply offset and wrap to (-180, 180] ───────────────── */
    float out = (foc->sensor_dir == REVERSE_DIR)
                ? -(as5600.mechTheta - offset)
                :  (as5600.mechTheta - offset);

    out = wrap_pm_180(out);

    as5600.data_ready = 1;
    as5600.validCount++;
    as5600.lastUpdateMs = HAL_GetTick();
    foc->actual_angle_as5600 = out;
}

/*
 * Called from HAL_I2C_ErrorCallback() when hi2c matches.
 * Releases the busy flag so polling/timer can retry on the next cycle.
 */
void AS5600_Error_Handler(void) {
    as5600.busy = 0;
    as5600.errorCount++;
}
