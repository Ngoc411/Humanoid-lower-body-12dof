/*
 * DRV8323.c
 *
 *  DRV8323SRTAT gate driver implementation
 *  SPI3: PC10(SCK), PC11(MISO), PC12(MOSI), PD2(nSCS)
 *
 *  Created on: Apr 2026
 *      Author: Ngoc
 */

#include "DRV8323.h"
#include <stddef.h>
#include <math.h>

/* Last HAL status for every SPI transaction — watch in debugger */
volatile HAL_StatusTypeDef drv_spi_wr_status = HAL_ERROR;
volatile HAL_StatusTypeDef drv_spi_rd_status = HAL_ERROR;

/* ================================================================== */
/*  Internal SPI helpers                                               */
/* ================================================================== */

static inline void drv_nss_low(DRV8323_t *d)
{
    d->nss_port->BSRR = (uint32_t)(d->nss_pin) << 16; /* RESET */
}

static inline void drv_nss_high(DRV8323_t *d)
{
    d->nss_port->BSRR = d->nss_pin; /* SET */
//    /* tSCS ≥ 400 ns between transactions (DRV8323 datasheet).
//     * At 170 MHz: 100 NOPs ≈ 600 ns — safe margin.              */
    for (volatile int _i = 0; _i < 100; _i++) { __asm volatile("nop"); }
}

/* ------------------------------------------------------------------ */
/*  DRV8323_reg_write / _read  (16-bit polling SPI)                   */
/* ------------------------------------------------------------------ */

void DRV8323_reg_write(DRV8323_t *d, uint8_t addr, uint16_t data)
{
	uint16_t frame = DRV8323_SPI_WRITE
	               | ((uint16_t)(addr & 0x0FU) << DRV8323_SPI_ADDR_SHIFT)
	               | (data & DRV8323_SPI_DATA_MASK);

    uint16_t dummy;

    drv_nss_low(d);
    drv_spi_wr_status = HAL_SPI_TransmitReceive(d->spi, (uint8_t *)&frame, (uint8_t *)&dummy, 1, 10);
    drv_nss_high(d);
}

uint16_t DRV8323_reg_read(DRV8323_t *d, uint8_t addr)
{
    uint16_t frame = DRV8323_SPI_READ
                   | ((uint16_t)(addr & 0x0FU) << DRV8323_SPI_ADDR_SHIFT);
    uint16_t result = 0;

    drv_nss_low(d);
    drv_spi_rd_status = HAL_SPI_TransmitReceive(d->spi, (uint8_t *)&frame, (uint8_t *)&result, 1, 10);
    drv_nss_high(d);

    return result & DRV8323_SPI_DATA_MASK;
}

/* ================================================================== */
/*  Configuration API                                                  */
/* ================================================================== */

void DRV8323_SPI_config(DRV8323_t *d,
                        SPI_HandleTypeDef *spi,
                        GPIO_TypeDef *nss_port,
                        uint16_t nss_pin)
{
    d->spi      = spi;
    d->nss_port = nss_port;
    d->nss_pin  = nss_pin;
    drv_nss_high(d); /* ensure de-asserted */
}

void DRV8323_GPIO_ENGATE_config(DRV8323_t *d, GPIO_TypeDef *port, uint16_t pin)
{
    d->en_gate_port = port;
    d->en_gate_pin  = pin;
}

void DRV8323_GPIO_NFAULT_config(DRV8323_t *d, GPIO_TypeDef *port, uint16_t pin)
{
    d->nfault_port = port;
    d->nfault_pin  = pin;
}

static uint32_t drv_get_timer_clock(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1  || htim->Instance == TIM8  ||
        htim->Instance == TIM20 || htim->Instance == TIM15 ||
        htim->Instance == TIM16 || htim->Instance == TIM17) {
        /* APB2 timers */
        uint32_t pclk = HAL_RCC_GetPCLK2Freq();
        if ((RCC->CFGR & RCC_CFGR_PPRE2) != RCC_CFGR_PPRE2_DIV1)
            return pclk * 2U;
        return pclk;
    } else {
        /* APB1 timers */
        uint32_t pclk = HAL_RCC_GetPCLK1Freq();
        if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
            return pclk * 2U;
        return pclk;
    }
}

int DRV8323_TIMER_config(DRV8323_t *d, TIM_HandleTypeDef *timer, uint32_t freq)
{
    if (!d || !timer) return 0;

    d->timer    = timer;
    d->pwm_freq = freq;

    const uint32_t timer_clock = drv_get_timer_clock(timer);

    uint32_t prescaler = 0;
    uint32_t period    = (timer_clock / (2U * freq)) - 1U;

    if (period > 0xFFFFU) {
        prescaler = period / 0xFFFFU;
        period    = (timer_clock / (2U * freq * (prescaler + 1U))) - 1U;
    }

    timer->Instance->PSC = prescaler;
    timer->Instance->ARR = period;
    d->pwm_resolution    = period;

    return 1;
}

int DRV8323_current_sens_config(DRV8323_t *d,
                                DRV8323_CSA_gain_t gain,
                                float R_shunt,
                                float v_offset_a,
                                float v_offset_c)
{
    if (!d || R_shunt <= 0.0f) return 0;

    d->gain_mode  = gain;
    d->R_shunt    = R_shunt;
    d->v_offset_a = v_offset_a;
    d->v_offset_c = v_offset_c;

    const float gain_lut[4] = {5.0f, 10.0f, 20.0f, 40.0f};
    d->v_to_current = 1.0f / (gain_lut[(uint8_t)gain] * R_shunt);

    return 1;
}

/* ================================================================== */
/*  PWM control                                                        */
/* ================================================================== */

int DRV8323_stop_pwm(DRV8323_t *d)
{
    if (!d || !d->timer) return 0;

    HAL_TIM_PWM_Stop(d->timer, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(d->timer, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(d->timer, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(d->timer, TIM_CHANNEL_4);

    HAL_TIMEx_PWMN_Stop(d->timer, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(d->timer, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(d->timer, TIM_CHANNEL_3);

    return 1;
}

int DRV8323_start_pwm(DRV8323_t *d)
{
    if (!d || !d->timer) return 0;

    HAL_TIM_PWM_Start(d->timer, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(d->timer, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(d->timer, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(d->timer, TIM_CHANNEL_4);

    HAL_TIMEx_PWMN_Start(d->timer, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(d->timer, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(d->timer, TIM_CHANNEL_3);

    return 1;
}

void DRV8323_set_pwm(DRV8323_t *d, uint32_t pwma, uint32_t pwmb, uint32_t pwmc)
{
    d->timer->Instance->CCR1 = pwmc;
    d->timer->Instance->CCR2 = pwmb;
    d->timer->Instance->CCR3 = pwma;
}

/* ================================================================== */
/*  Current sensing                                                    */
/* ================================================================== */

void DRV8323_get_current(DRV8323_t *d, float *ia, float *ic)
{
    static float ia_filtered = 0.0f;
    static float ic_filtered = 0.0f;

    const float vshunt_a = (float)d->adc_a * ADC_2_VOLT;
    const float vshunt_c = (float)d->adc_c * ADC_2_VOLT;

    float ia_raw = (vshunt_a - d->v_offset_a) * d->v_to_current;
    float ic_raw = (vshunt_c - d->v_offset_c) * d->v_to_current;

    ia_filtered = (1.0f - CURRENT_FILTER_ALPHA) * ia_filtered + CURRENT_FILTER_ALPHA * ia_raw;
    ic_filtered = (1.0f - CURRENT_FILTER_ALPHA) * ic_filtered + CURRENT_FILTER_ALPHA * ic_raw;

    *ia = ia_filtered;
    *ic = ic_filtered;
}

/* ================================================================== */
/*  Fault management                                                   */
/* ================================================================== */

uint16_t DRV8323_read_fault1(DRV8323_t *d)
{
    return DRV8323_reg_read(d, DRV8323_REG_FAULT_STAT1);
}

uint16_t DRV8323_read_fault2(DRV8323_t *d)
{
    return DRV8323_reg_read(d, DRV8323_REG_FAULT_STAT2);
}

void DRV8323_clear_faults(DRV8323_t *d)
{
    /* CLR_FLT bit — self-clearing */
    uint16_t ctrl = DRV8323_reg_read(d, DRV8323_REG_DRV_CTRL);
    DRV8323_reg_write(d, DRV8323_REG_DRV_CTRL, ctrl | DRV_CTRL_CLR_FLT);
}

void DRV8323_debug_read(DRV8323_t *d, volatile DRV8323_dbg_t *dbg)
{
    dbg->fault1     = DRV8323_reg_read(d, 0x00);
    dbg->fault2     = DRV8323_reg_read(d, 0x01);
    dbg->drv_ctrl   = DRV8323_reg_read(d, 0x02);
    dbg->gate_hs    = DRV8323_reg_read(d, 0x03);
    dbg->gate_ls    = DRV8323_reg_read(d, 0x04);
    dbg->ocp_ctrl   = DRV8323_reg_read(d, 0x05);
    dbg->csa_ctrl   = DRV8323_reg_read(d, 0x06);
    dbg->nfault_pin = (d->nfault_port->IDR & d->nfault_pin) ? 1U : 0U;
    dbg->engate_pin = (d->en_gate_port->IDR & d->en_gate_pin) ? 1U : 0U;
}

/* ================================================================== */
/*  DRV8323 setup                                                     */
/* ================================================================== */

void DRV8323_set_0x02(DRV8323_t *d) // PWM_MODE
{
    uint16_t def = DRV8323_reg_read(d, 0x02);
    def &= ~(0x3U << 5);        // clear PWM_MODE (bit 6:5)
    def |= DRV_CTRL_PWM_MODE_6X;

    DRV8323_reg_write(d, 0x02, def);

    d->check_spi_cfg = 2;
}


void DRV8323_set_0x03(DRV8323_t *d,
                      uint8_t idrivep_hs, uint8_t idriven_hs) // config IDRIVEP_HS, IDRIVEN_HS
{
    uint16_t def = DRV8323_reg_read(d, 0x03);
    def &= ~((0xFU << 4) | (0xFU << 0)); // clear IDRIVEP_HS, IDRIVEN_HS

    def |= GATE_HS_UNLOCK |
    	   GATE_HS_IDRIVEP(idrivep_hs) |
           GATE_HS_IDRIVEN(idriven_hs);

    DRV8323_reg_write(d, 0x03, def);

    d->check_spi_cfg = 3;
}


void DRV8323_set_0x04(DRV8323_t *d,
                      uint8_t tdrive,
                      uint8_t idrivep_ls,
                      uint8_t idriven_ls) // config TDRIVE, IDRIVEP_LS, IDRIVEN_LS
{
    uint16_t def = DRV8323_reg_read(d, 0x04);
    def &= ~((0x3U << 8) | (0xFU << 4) | (0xFU << 0)); // clear fields

    def |= GATE_LS_TDRIVE(tdrive) |
           GATE_LS_IDRIVEP(idrivep_ls) |
           GATE_LS_IDRIVEN(idriven_ls);

    DRV8323_reg_write(d, 0x04, def);

    d->check_spi_cfg = 4;
}


void DRV8323_set_0x05(DRV8323_t *d,
                      uint8_t dead_time,
                      uint8_t ocp_mode,
                      uint8_t vds_lvl) // config DEAD_TIME, OCP_MODE, VDS_LVL
{
    uint16_t def = DRV8323_reg_read(d, 0x05);
    def &= ~((0x3U << 8) | (0x3U << 6) | (0xFU << 0)); // clear fields

    def |= OCP_DEAD_TIME(dead_time) |
           OCP_MODE(ocp_mode) |
           OCP_VDS_LVL(vds_lvl);

    DRV8323_reg_write(d, 0x05, def);

    d->check_spi_cfg = 5;
}


void DRV8323_set_0x06(DRV8323_t *d,
                      uint8_t csa_gain) // config CSA_GAIN
{
    uint16_t def = DRV8323_reg_read(d, 0x06);
    def &= ~(0x3U << 6); // clear CSA_GAIN

    def |= CSA_GAIN(csa_gain);

    DRV8323_reg_write(d, 0x06, def);

    d->check_spi_cfg = 6;
}


/* ================================================================== */
/*  DRV8323_init                                                       */
/* ================================================================== */
/*
 * Call order in application:
 *   DRV8323_SPI_config()
 *   DRV8323_GPIO_ENGATE_config()
 *   DRV8323_GPIO_NFAULT_config()   (optional)
 *   DRV8323_TIMER_config()
 *   DRV8323_current_sens_config()
 *   DRV8323_init()                 <-- this function
 *   DRV8323_enable_gate()          <-- called separately after motor init
 */
int DRV8323_init(DRV8323_t *d)
{
    if (!d || !d->timer || !d->spi) return 0;

    DRV8323_stop_pwm(d);

    /* EN_GATE HIGH trước — DRV8323 không nhận SPI khi sleep */
    DRV8323_enable_gate(d);
    HAL_Delay(10); /* t_WAKE: chờ chip khởi động nội bộ */

    /* Clear fault power-up trước khi cấu hình */
    DRV8323_clear_faults(d);
    HAL_Delay(1);

    /* --- Write configuration registers --- */

    DRV8323_set_0x02(d); // PWM_MODE - fixed 6x
    HAL_Delay(1);
    DRV8323_set_0x03(d, 8, 8); // I_drive = Qg / t_ris = 35nC / 150n = 233mA -> choose 260mA for idrivep and 520mA for idriven
    HAL_Delay(1);
    DRV8323_set_0x04(d, 0, 8, 8); // t_drive_min = Qg / I_drive = 35nC / 260mA = 135ns -> choose 0: 500ns
    HAL_Delay(1);
    /* OCP_MODE=2 (report only, không disable output) để test — đổi về 0 (latched) khi chạy thực */
    /* VDS_LVL=6 (0.53V) cho test không tải — đổi về 2 (0.2V) khi có motor */
    DRV8323_set_0x05(d, 2, 2, 9);
    HAL_Delay(1);
    DRV8323_set_0x06(d, 1); // choose 1 - gain 10

    HAL_Delay(10);

    /* Clear any fault latched during the config window (OCP_MODE was still
     * default=latched when registers were being written). */
    DRV8323_clear_faults(d);
    HAL_Delay(1);

    /* Start PWM outputs */
//    DRV8323_start_pwm(d);

    return 1;
}
