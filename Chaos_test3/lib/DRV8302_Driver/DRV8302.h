/*
 * DRV8302.h
 *
 *  Created on: May 31, 2025
 *      Author: munir
 */

#ifndef DRV8302_DRIVER_INC_DRV8302_H_
#define DRV8302_DRIVER_INC_DRV8302_H_

#include "DRV8302_config.h"

typedef enum {
	_10VPV,
	_40VPV
}CSA_gain_t;

typedef struct {
	GPIO_TypeDef *gain_port;  
	uint16_t gain_pin;   
	GPIO_TypeDef *en_gate_port;
	uint16_t en_gate_pin;

    TIM_HandleTypeDef *timer;
	uint32_t adc_a;
	uint32_t adc_b;

	uint32_t pwm_freq;
	uint32_t pwm_resolution;

	CSA_gain_t gain_mode;

	float R_shunt;
	float v_offset_a;
	float v_offset_b;
	float v_to_current; //pre-calculate conversion from voltaeg(V) to current(A)
}DRV8302_t;

static inline void DRV8302_enable_gate(DRV8302_t *cfg)
{
    cfg->en_gate_port->BSRR = cfg->en_gate_pin;
}

static inline void DRV8302_disable_gate(DRV8302_t *cfg)
{
    cfg->en_gate_port->BSRR = (cfg->en_gate_pin << 16);
}

static inline void DRV8302_set_adc_a(DRV8302_t *cfg, uint32_t val)
{
    cfg->adc_a = val;
}

static inline void DRV8302_set_adc_b(DRV8302_t *cfg, uint32_t val)
{
    cfg->adc_b = val;
}


void DRV8302_GPIO_GAIN_config(DRV8302_t *cfg, GPIO_TypeDef *port, uint16_t pin);
void DRV8302_GPIO_ENGATE_config(DRV8302_t *cfg, GPIO_TypeDef *port, uint16_t pin);
int DRV8302_TIMER_config(DRV8302_t *cfg, TIM_HandleTypeDef *timer, uint32_t freq);
int DRV8302_current_sens_config(DRV8302_t *cfg, CSA_gain_t gain, float R_shunt, float v_offset_a, float v_offset_b);
int DRV8302_stop_pwm(DRV8302_t *cfg);
int DRV8302_start_pwm(DRV8302_t *cfg);
int DRV8302_init(DRV8302_t *cfg);
void DRV8302_set_pwm(DRV8302_t *cfg, uint32_t pwma, uint32_t pwmb, uint32_t pwmc);
void DRV8302_get_current(DRV8302_t *cfg, float *ia, float *ib);

#endif /* DRV8302_DRIVER_INC_DRV8302_H_ */
