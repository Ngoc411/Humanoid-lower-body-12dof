#include "FOC_protection.h"
#include "as5600.h"
#include "MT6816.h"

#include <math.h>

static foc_t *protection_foc;
static DRV8323_t *protection_drv;
static volatile uint8_t *protection_init_done;
static volatile uint32_t *protection_fault_flags;
static foc_protection_config_t protection_cfg;
static float pos_limit_high;
static float pos_limit_low;
static float init_zero_last_abs_angle;
static uint32_t init_zero_away_count;
static uint8_t init_zero_trend_ready;
static float homing_target;   /* vị trí home mong muốn (= sp_input lúc init), mặc định 0 */

static float clamp_float(float value, float low, float high)
{
    if (value > high) return high;
    if (value < low) return low;
    return value;
}

static float wrap_pm_180_local(float deg)
{
    deg = fmodf(deg + 180.0f, 360.0f);
    if (deg < 0.0f) deg += 360.0f;
    return deg - 180.0f;
}

void foc_protection_init(foc_t *foc_handle,
                         DRV8323_t *drv_handle,
                         volatile uint8_t *init_done_flag,
                         volatile uint32_t *fault_flags,
                         const foc_protection_config_t *config)
{
    protection_foc = foc_handle;
    protection_drv = drv_handle;
    protection_init_done = init_done_flag;
    protection_fault_flags = fault_flags;
    if (config != NULL) {
        protection_cfg = *config;

        pos_limit_high = protection_cfg.pos_limit_high;
        pos_limit_low = protection_cfg.pos_limit_low;

        if (pos_limit_high < pos_limit_low) {
            float tmp = pos_limit_high;
            pos_limit_high = pos_limit_low;
            pos_limit_low = tmp;
        }
    }
}

uint8_t foc_protection_angle_is_valid(float angle)
{
    return 0.0f <= isfinite(angle) && fabsf(angle) <= 360.0f;
}

void foc_protection_motor_output_inhibit(void)
{
    if (protection_foc != NULL) {
        protection_foc->control_mode = POWER_UP_MODE;
        protection_foc->id_ref = 0.0f;
        protection_foc->iq_ref = 0.0f;
        pid_reset(&protection_foc->id_ctrl);
        pid_reset(&protection_foc->iq_ctrl);
        pid_reset(&protection_foc->speed_ctrl);
        pid_reset(&protection_foc->pos_ctrl);

        if (protection_foc->pwm_a != NULL) *protection_foc->pwm_a = 0;
        if (protection_foc->pwm_b != NULL) *protection_foc->pwm_b = 0;
        if (protection_foc->pwm_c != NULL) *protection_foc->pwm_c = 0;
    }

    if (protection_drv != NULL) {
        if (protection_drv->timer != NULL) {
            DRV8323_stop_pwm(protection_drv);
        }
        if (protection_drv->en_gate_port != NULL) {
            DRV8323_disable_gate(protection_drv);
        }
    }
}

void foc_protection_init_fail(uint32_t fault)
{
    if (protection_fault_flags != NULL) {
        *protection_fault_flags |= fault;
    }
    foc_protection_motor_output_inhibit();

    while (1) {
        HAL_GPIO_TogglePin(protection_cfg.fault_led_port, protection_cfg.fault_led_pin);
        for (volatile uint32_t d = 0; d < 425000UL; d++);  /* ~100 ms @ 170 MHz */
    }
}

uint8_t foc_protection_wait_as5600_init_ready(void)
{
    const uint32_t start_ms = HAL_GetTick();
    const uint32_t start_valid = as5600.validCount;
    const uint32_t start_errors = as5600.errorCount;

    while ((HAL_GetTick() - start_ms) < protection_cfg.encoder_init_timeout_ms) {
        AS5600_Start_DMA();

        if ((as5600.errorCount - start_errors) > protection_cfg.encoder_init_max_errors) {
            *protection_fault_flags |= INIT_FAULT_AS5600_ERRORS;
            return 0;
        }
        if ((as5600.validCount - start_valid) >= protection_cfg.encoder_init_min_valid_samples &&
            as5600.data_ready &&
            (HAL_GetTick() - as5600.lastUpdateMs) <= protection_cfg.encoder_fresh_ms &&
            foc_protection_angle_is_valid(protection_foc->actual_angle_as5600)) {
            return 1;
        }
        HAL_Delay(1);
    }

    *protection_fault_flags |= INIT_FAULT_AS5600_TIMEOUT;
    return 0;
}

uint8_t foc_protection_wait_as5600_stable_reference(float *angle_out)
{
    const uint32_t start_ms = HAL_GetTick();
    const uint32_t start_errors = as5600.errorCount;
    const uint32_t start_rejects = as5600.rejectCount;
    uint32_t last_valid = as5600.validCount;
    uint32_t stable_count = 0;
    float anchor = 0.0f;
    float sum_delta = 0.0f;

    while ((HAL_GetTick() - start_ms) < protection_cfg.encoder_init_timeout_ms) {
        AS5600_Start_DMA();

        if ((as5600.errorCount - start_errors) > protection_cfg.encoder_init_max_errors) {
            *protection_fault_flags |= INIT_FAULT_AS5600_ERRORS;
            return 0;
        }
        if ((as5600.rejectCount - start_rejects) > protection_cfg.as5600_ref_max_rejects) {
            *protection_fault_flags |= INIT_FAULT_AS5600_UNSTABLE;
            return 0;
        }

        if (as5600.validCount != last_valid) {
            last_valid = as5600.validCount;
            float angle = protection_foc->actual_angle_as5600;

            if (!foc_protection_angle_is_valid(angle)) {
                *protection_fault_flags |= INIT_FAULT_BAD_ANGLE;
                return 0;
            }

            if (stable_count == 0U) {
                anchor = angle;
                sum_delta = 0.0f;
                stable_count = 1U;
            } else {
                float delta = wrap_pm_180_local(angle - anchor);
                if (fabsf(delta) > protection_cfg.as5600_ref_max_spread_deg) {
                    anchor = angle;
                    sum_delta = 0.0f;
                    stable_count = 1U;
                } else {
                    sum_delta += delta;
                    stable_count++;
                }
            }

            if (stable_count >= protection_cfg.as5600_ref_min_stable_samples &&
                (HAL_GetTick() - as5600.lastUpdateMs) <= protection_cfg.encoder_fresh_ms) {
                *angle_out = wrap_pm_180_local(anchor + (sum_delta / (float)stable_count));
                return 1;
            }
        }

        HAL_Delay(1);
    }

    *protection_fault_flags |= INIT_FAULT_AS5600_UNSTABLE;
    return 0;
}

uint8_t foc_protection_wait_mt6816_init_ready(void)
{
    const uint32_t start_ms = HAL_GetTick();
    const uint32_t start_valid = encoder.validCount;
    const uint32_t start_errors = encoder.parityErrorCount + encoder.spiErrorCount;

    while ((HAL_GetTick() - start_ms) < protection_cfg.encoder_init_timeout_ms) {
        Spi_Transmit_Receive_Data_8Bit(reg_addrs[encoder.read_flag]);

        uint32_t errors = encoder.parityErrorCount + encoder.spiErrorCount;
        if ((errors - start_errors) > protection_cfg.encoder_init_max_errors) {
            *protection_fault_flags |= INIT_FAULT_MT6816_ERRORS;
            return 0;
        }
        if ((encoder.validCount - start_valid) >= protection_cfg.encoder_init_min_valid_samples &&
            (HAL_GetTick() - encoder.lastUpdateMs) <= protection_cfg.encoder_fresh_ms &&
            foc_protection_angle_is_valid(protection_foc->actual_angle)) {
            return 1;
        }
        HAL_Delay(1);
    }

    *protection_fault_flags |= INIT_FAULT_MT6816_TIMEOUT;
    return 0;
}

void foc_protection_limit_pre_homing_speed(void)
{
    if (protection_foc == NULL || protection_init_done == NULL) {
        return;
    }
    if (*protection_init_done ||
        fabsf(protection_foc->actual_angle_as5600 - homing_target) < protection_cfg.wait_ang) {
        return;
    }

    if (fabsf(protection_foc->actual_rpm) > protection_cfg.homing_rpm_limit &&
        (protection_foc->iq_ref * protection_foc->actual_rpm) > 0.0f) {
        protection_foc->iq_ref = 0.0f;
        pid_reset(&protection_foc->pos_ctrl);
    }
}

float foc_protection_position_target(float requested_target)
{
    if (protection_foc == NULL || protection_init_done == NULL || !(*protection_init_done)) {
        return requested_target;
    }

    return clamp_float(requested_target, pos_limit_low, pos_limit_high);
}

void foc_protection_set_homing_target(float target)
{
    homing_target = target;
}

void foc_protection_init_zero_trend_reset(void)
{
    init_zero_last_abs_angle = 0.0f;
    init_zero_away_count = 0;
    init_zero_trend_ready = 0;
}

/* angle: vị trí homing hiện tại — called at 10 kHz from ADC ISR.
 * Checks if khoảng cách |angle - homing_target| is consistently increasing
 * → motor moving away from the homing target (sai hướng).
 */
uint8_t foc_protection_init_moving_away_from_zero(float angle)
{
    if (protection_cfg.init_away_max_count == 0U) return 0;
    if (!foc_protection_angle_is_valid(angle)) return 0;

    const float abs_angle = fabsf(angle - homing_target);

    if (!init_zero_trend_ready) {
        init_zero_last_abs_angle = abs_angle;
        init_zero_trend_ready = 1;
        return 0;
    }

    if (abs_angle > init_zero_last_abs_angle + protection_cfg.init_away_min_delta_deg) {
        if (init_zero_away_count < protection_cfg.init_away_max_count)
            init_zero_away_count++;
    } else if (abs_angle + protection_cfg.init_away_min_delta_deg < init_zero_last_abs_angle) {
        init_zero_away_count = 0;
    }

    init_zero_last_abs_angle = abs_angle;
    return init_zero_away_count >= protection_cfg.init_away_max_count;
}
