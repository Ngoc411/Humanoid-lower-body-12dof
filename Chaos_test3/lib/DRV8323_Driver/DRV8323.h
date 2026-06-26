/*
 * DRV8323.h
 *
 *  DRV8323SRTAT gate driver — SPI-configurable 3-phase MOSFET driver
 *
 *  SPI3 interface: PC10(SCK), PC11(MISO), PC12(MOSI), PD2(nSCS - software NSS)
 *
 *  Created on: Apr 2026
 *      Author: Ngoc
 */

#ifndef DRV8323_DRIVER_INC_DRV8323_H_
#define DRV8323_DRIVER_INC_DRV8323_H_

#include "DRV8323_config.h"

/* ------------------------------------------------------------------ */
/*  CSA gain define                                                   */
/* ------------------------------------------------------------------ */

//#define DRV_GAIN_5VV 0
//#define DRV_GAIN_10VV 1
//#define DRV_GAIN_20VV 2
//#define DRV_GAIN_40VV 3

typedef enum {
    DRV_GAIN_5VV  = DRV8323_GAIN_5VV,
    DRV_GAIN_10VV = DRV8323_GAIN_10VV,
    DRV_GAIN_20VV = DRV8323_GAIN_20VV,
    DRV_GAIN_40VV = DRV8323_GAIN_40VV,
} DRV8323_CSA_gain_t;

/* ------------------------------------------------------------------ */
/*  Driver handle                                                       */
/* ------------------------------------------------------------------ */
typedef struct {
	/* TIMER config */


    /* SPI interface for register access */
    SPI_HandleTypeDef *spi;
    GPIO_TypeDef      *nss_port;
    uint16_t           nss_pin;

    /* Gate-enable GPIO */
    GPIO_TypeDef      *en_gate_port;
    uint16_t           en_gate_pin;

    /* nFAULT GPIO (active-low input, open-drain from DRV8323) */
    GPIO_TypeDef      *nfault_port;
    uint16_t           nfault_pin;

    /* PWM timer */
    TIM_HandleTypeDef *timer;
    uint32_t           pwm_freq;
    uint32_t           pwm_resolution;   /* = ARR value */

    /* ADC raw values set by ISR */
    uint32_t adc_a;
    uint32_t adc_c;

    /* Current sensing */
    DRV8323_CSA_gain_t gain_mode;
    float R_shunt;
    float v_offset_a;
    float v_offset_c;
    float v_to_current;  /* = 1.0f / (gain * R_shunt) */

    uint8_t check_spi_cfg;
} DRV8323_t;

typedef struct {
    uint16_t fault1;
    uint16_t fault2;
    uint16_t drv_ctrl;
    uint16_t gate_hs;
    uint16_t gate_ls;
    uint16_t ocp_ctrl;
    uint16_t csa_ctrl;
    uint8_t  nfault_pin;
    uint8_t  engate_pin;
} DRV8323_dbg_t;

/* ------------------------------------------------------------------ */
/*  Inline helpers                                                      */
/* ------------------------------------------------------------------ */
static inline void DRV8323_enable_gate(DRV8323_t *d)
{
    d->en_gate_port->BSRR = d->en_gate_pin;
}

static inline void DRV8323_disable_gate(DRV8323_t *d)
{
    d->en_gate_port->BSRR = (uint32_t)(d->en_gate_pin) << 16;
}

static inline void DRV8323_set_adc_a(DRV8323_t *d, uint32_t val) { d->adc_a = val; }
static inline void DRV8323_set_adc_c(DRV8323_t *d, uint32_t val) { d->adc_c = val; }

static inline uint8_t DRV8323_fault_active(DRV8323_t *d)
{
    /* nFAULT is active-low — fault when pin reads 0 */
    return (d->nfault_port->IDR & d->nfault_pin) == 0;
}

/* ------------------------------------------------------------------ */
/*  Public API                                                          */
/* ------------------------------------------------------------------ */

/* Configuration */
void DRV8323_SPI_config(DRV8323_t *d,
                        SPI_HandleTypeDef *spi,
                        GPIO_TypeDef *nss_port,
                        uint16_t nss_pin);

void DRV8323_GPIO_ENGATE_config(DRV8323_t *d,
                                GPIO_TypeDef *port,
                                uint16_t pin);

void DRV8323_GPIO_NFAULT_config(DRV8323_t *d,
                                GPIO_TypeDef *port,
                                uint16_t pin);

int  DRV8323_TIMER_config(DRV8323_t *d,
                          TIM_HandleTypeDef *timer,
                          uint32_t freq);

int  DRV8323_current_sens_config(DRV8323_t *d,
                                 DRV8323_CSA_gain_t gain,
                                 float R_shunt,
                                 float v_offset_a,
                                 float v_offset_c);

/* Initialisation — writes default registers, starts PWM */
int DRV8323_init(DRV8323_t *d);

/* PWM control */
int  DRV8323_start_pwm(DRV8323_t *d);
int  DRV8323_stop_pwm(DRV8323_t *d);
void DRV8323_set_pwm(DRV8323_t *d, uint32_t pwma, uint32_t pwmb, uint32_t pwmc);

/* Current sensing */
void DRV8323_get_current(DRV8323_t *d, float *ia, float *ic);

/* SPI register access */
uint16_t DRV8323_reg_read(DRV8323_t *d, uint8_t addr);
void     DRV8323_reg_write(DRV8323_t *d, uint8_t addr, uint16_t data);

/* Fault management */
uint16_t DRV8323_read_fault1(DRV8323_t *d);
uint16_t DRV8323_read_fault2(DRV8323_t *d);
void     DRV8323_clear_faults(DRV8323_t *d);
void     DRV8323_debug_read(DRV8323_t *d, volatile DRV8323_dbg_t *dbg);


// DRV8323 setup
void DRV8323_set_0x02(DRV8323_t *d);
// config PWM_MODE

void DRV8323_set_0x03(DRV8323_t *d,
        uint8_t idrivep_hs, uint8_t idriven_hs);
// config IDRIVEP_HS, IDRIVEN_HS

void DRV8323_set_0x04(DRV8323_t *d,
        uint8_t tdrive,
        uint8_t idrivep_ls,
        uint8_t idriven_ls);
// config TDRIVE, IDRIVEP_LS, IDRIVEN_LS

void DRV8323_set_0x05(DRV8323_t *d,
        uint8_t dead_time,
        uint8_t ocp_mode,
        uint8_t vds_lvl);
// config DEAD_TIME, OCP_MODE, VDS_LVL

void DRV8323_set_0x06(DRV8323_t *d,
        uint8_t csa_gain);
// config CSA_GAIN

/* CSA gain update at runtime */
void DRV8323_csa_gain_set(DRV8323_t *d, DRV8323_CSA_gain_t gain);

#endif /* DRV8323_DRIVER_INC_DRV8323_H_ */
