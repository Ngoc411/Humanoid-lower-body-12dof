#ifndef FOC_INC_FOC_PROTECTION_H_
#define FOC_INC_FOC_PROTECTION_H_

#include <stdint.h>
#include "main.h"
#include "FOC_utils.h"
#include "DRV8323.h"

typedef enum {
    INIT_FAULT_AS5600_TIMEOUT  = (1U << 0),
    INIT_FAULT_AS5600_ERRORS   = (1U << 1),
    INIT_FAULT_MT6816_TIMEOUT  = (1U << 2),
    INIT_FAULT_MT6816_ERRORS   = (1U << 3),
    INIT_FAULT_HOMING_TIMEOUT  = (1U << 4),
    INIT_FAULT_BAD_ANGLE       = (1U << 5),
    INIT_FAULT_BAD_OFFSET      = (1U << 6),
    INIT_FAULT_AS5600_MAGNET   = (1U << 7),
    INIT_FAULT_AS5600_UNSTABLE = (1U << 8),
    INIT_FAULT_HOMING_AWAY       = (1U << 9),
    RUNTIME_FAULT_MT6816_STALE   = (1U << 10),
    RUNTIME_FAULT_CAN_TIMEOUT    = (1U << 11),
    RUNTIME_FAULT_DRV_FAULT      = (1U << 12),
} init_fault_t;

typedef struct {
    uint32_t encoder_init_timeout_ms;
    uint32_t encoder_init_min_valid_samples;
    uint32_t encoder_init_max_errors;
    uint32_t encoder_fresh_ms;
    uint32_t as5600_ref_min_stable_samples;
    uint32_t as5600_ref_max_rejects;
    float as5600_ref_max_spread_deg;
    float wait_ang;
    float homing_rpm_limit;
    float pos_limit_high;
    float pos_limit_low;
    float init_away_min_delta_deg;
    uint32_t init_away_max_count;
    GPIO_TypeDef *fault_led_port;
    uint16_t fault_led_pin;
} foc_protection_config_t;

void foc_protection_init(foc_t *foc_handle,
                         DRV8323_t *drv_handle,
                         volatile uint8_t *init_done_flag,
                         volatile uint32_t *fault_flags,
                         const foc_protection_config_t *config);

uint8_t foc_protection_angle_is_valid(float angle);
void foc_protection_motor_output_inhibit(void);
void foc_protection_init_fail(uint32_t fault);
uint8_t foc_protection_wait_as5600_init_ready(void);
uint8_t foc_protection_wait_as5600_stable_reference(float *angle_out);
uint8_t foc_protection_wait_mt6816_init_ready(void);
void foc_protection_limit_pre_homing_speed(void);
float foc_protection_position_target(float requested_target);
void foc_protection_set_homing_target(float target);
void foc_protection_init_zero_trend_reset(void);
uint8_t foc_protection_init_moving_away_from_zero(float angle);

#endif /* FOC_INC_FOC_PROTECTION_H_ */
