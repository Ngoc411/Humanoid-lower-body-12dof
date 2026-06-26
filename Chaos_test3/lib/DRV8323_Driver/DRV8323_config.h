/*
 * DRV8323_config.h
 *
 *  Created on: Apr 2026
 *      Author: Ngoc
 *
 *  Register bit fields verified against DRV832x datasheet SLVSDJ3D (Rev D, March 2022)
 */

#ifndef DRV8323_DRIVER_INC_DRV8323_CONFIG_H_
#define DRV8323_DRIVER_INC_DRV8323_CONFIG_H_

#include "stm32g4xx_hal.h"

#define ADC_RES     4096
#define AVDD        3.3f

// = AVDD / ADC_RES
#define ADC_2_VOLT          0.0008056640625f

#define ADC_2_POWER_VOLT    0.01651611328125f

#define CURRENT_FILTER_ALPHA 0.466512f

/* --------- DRV8323S SPI frame format (16-bit) ---------
 *  Bit 15   : R/W  (1 = read, 0 = write)
 *  Bits 14:11: Register address [3:0]
 *  Bits 10:0 : Data [10:0]
 */
#define DRV8323_SPI_READ            (1U << 15)
#define DRV8323_SPI_WRITE           (0U << 15)
#define DRV8323_SPI_ADDR_SHIFT      11U
#define DRV8323_SPI_DATA_MASK       0x07FFU

/* --------- Register addresses --------- */
#define DRV8323_REG_FAULT_STAT1     0x00U
#define DRV8323_REG_FAULT_STAT2     0x01U
#define DRV8323_REG_DRV_CTRL        0x02U
#define DRV8323_REG_GATE_HS         0x03U
#define DRV8323_REG_GATE_LS         0x04U
#define DRV8323_REG_OCP_CTRL        0x05U
#define DRV8323_REG_CSA_CTRL        0x06U

/* =========================================================
 * Driver Control Register (0x02)
 * ---------------------------------------------------------
 * Bit 10   : Reserved
 * Bit  9   : DIS_CPUV  0=charge pump UVLO fault enabled,   1=disabled
 * Bit  8   : DIS_GDF   0=gate drive fault enabled,         1=disabled
 * Bit  7   : OTW_REP   0=OTW không báo nFAULT,             1=OTW báo nFAULT
 * Bit 6:5  : PWM_MODE  00=6x PWM, 01=3x PWM, 10=1x PWM, 11=Independent
 * Bit  4   : 1PWM_COM  0=synchronous rectification, 1=asynchronous (diode freewheel)
 * Bit  3   : 1PWM_DIR  direction bit trong 1x PWM mode
 * Bit  2   : COAST     1=tất cả MOSFET Hi-Z
 * Bit  1   : BRAKE     1=bật tất cả low-side FET (chỉ dùng trong 1x PWM mode)
 * Bit  0   : CLR_FLT   write 1 để clear latched fault, tự reset sau khi write
 * ========================================================= */
#define DRV_CTRL_CLR_FLT        (1U << 0)   // write 1 to clear latched fault
#define DRV_CTRL_BRAKE          (0x0U << 1) // default bit 0
#define DRV_CTRL_COAST          (0x0U << 2) // default bit 0
#define DRV_CTRL_1PWM_DIR       (0x0U << 3) // default bit 0
#define DRV_CTRL_1PWM_COM       (0x0U << 4) // default bit 0
#define DRV_CTRL_PWM_MODE_6X    (0x0U << 5) // 6x PWM - bit 5,6 = 00
#define DRV_CTRL_PWM_MODE_3X    (0x1U << 5) // 3x PWM - bit 5,6 = 01
#define DRV_CTRL_PWM_MODE_1X    (0x2U << 5) // 1x PWM - bit 5,6 = 10
#define DRV_CTRL_PWM_MODE_IND   (0x3U << 5) // independent - bit 5,6 = 11
#define DRV_CTRL_OTW_REP        (0x0U << 7) // default bit 0
#define DRV_CTRL_DIS_GDF        (0x0U << 8) // default bit 0
#define DRV_CTRL_DIS_CPUV       (0x0U << 9) // default bit 0

/* =========================================================
 * Gate Drive HS Register (0x03)
 * ---------------------------------------------------------
 * Bit 10:8 : LOCK      011=unlock, 110=lock (phải unlock trước khi write)
 * Bit  7:4 : IDRIVEP   source current (charge gate)
 * Bit  3:0 : IDRIVEN   sink current   (discharge gate)
 *
 * IDRIVEP: 0=10mA,  1=30mA,   2=60mA,   3=80mA,   4=120mA,  5=140mA,
 *          6=170mA, 7=190mA,  8=260mA,  9=330mA,  10=370mA, 11=440mA,
 *          12=570mA,13=680mA, 14=820mA, 15=1000mA
 * IDRIVEN: 0=20mA,  1=60mA,   2=120mA,  3=160mA,  4=240mA,  5=280mA,
 *          6=340mA, 7=380mA,  8=520mA,  9=660mA,  10=740mA, 11=880mA,
 *          12=1140mA,13=1360mA,14=1640mA,15=2000mA
 * (this enum use for both HS and LS)
 *
 * AON6354: Qg(10V)=35nC max, target t_rise=150ns @ 10kHz FOC:
 *   I_source = 35nC / 150ns = 233mA  → index 8 = 260mA
 *   I_sink   = 2× source            → index 8 = 520mA
 * ========================================================= */
#define GATE_HS_UNLOCK          (0x3U << 8) // default unlock
#define GATE_HS_LOCK            (0x6U << 8)
#define GATE_HS_IDRIVEP(x)      (((x) & 0xFU) << 4)
#define GATE_HS_IDRIVEN(x)      (((x) & 0xFU) << 0)

/* =========================================================
 * Gate Drive LS Register (0x04)
 * ---------------------------------------------------------
 * Bit 10   : CBC        1=cycle-by-cycle OCP, 0=latched OCP
 * Bit  9:8 : TDRIVE     0=500ns, 1=1us, 2=2us, 3=4us
 * Bit  7:4 : IDRIVEP    source current (enum giống HS)
 * Bit  3:0 : IDRIVEN    sink current   (enum giống HS)
 * ========================================================= */
#define GATE_LS_CBC             (1U << 10) // default bit 0
#define GATE_LS_TDRIVE(x)       (((x) & 0x3U) << 8)
#define GATE_LS_IDRIVEP(x)      (((x) & 0xFU) << 4)
#define GATE_LS_IDRIVEN(x)      (((x) & 0xFU) << 0)

/* =========================================================
 * OCP Control Register (0x05)
 * ---------------------------------------------------------
 * Bit 10   : TRETRY     0=4ms retry, 1=50ms retry
 * Bit  9:8 : DEAD_TIME  0=50ns, 1=100ns, 2=200ns, 3=400ns
 * Bit  7:6 : OCP_MODE   0=latched, 1=auto-retry, 2=report only, 3=disabled
 * Bit  5:4 : OCP_DEG    0=2us, 1=4us, 2=6us, 3=8us  (deglitch time)
 * Bit  3:0 : VDS_LVL    0=0.06V, 1=0.13V, 2=0.20V,  3=0.26V,
 *                       4=0.31V, 5=0.45V, 6=0.53V,  7=0.60V,
 *                       8=0.68V, 9=0.75V, 10=0.94V, 11=1.13V,
 *                       12=1.30V,13=1.50V, 14=1.69V, 15=1.88V
 * ========================================================= */
#define OCP_TRETRY              (0U << 10) // default bit 0
#define OCP_DEAD_TIME(x)        (((x) & 0x3U) << 8)
#define OCP_MODE(x)             (((x) & 0x3U) << 6)
#define OCP_DEG(x)              (((x) & 0x3U) << 4)
#define OCP_VDS_LVL(x)          (((x) & 0xFU) << 0)

/* =========================================================
 * CSA Control Register (0x06)
 * =========================================================
 * Bit(s)   Name        Description
 * --------------------------------------------------------------------------
 * 10       CSA_FET     Current Sense Amplifier Input Selection
 *                      0b = Positive input is SPx (low-side shunt)
 *                      1b = Positive input is SHx (VDS sensing, also sets LS_REF=1)
 * 9        VREF_DIV    CSA Reference Voltage Select
 *                      0b = VREF (unidirectional mode)
 *                      1b = VREF/2 (bidirectional mode)  [DEFAULT]
 * 8        LS_REF      Low-Side MOSFET VDS_OCP Measurement Reference
 *                      0b = Measured across SHx to SPx  [DEFAULT]
 *                      1b = Measured across SHx to SNx
 * 7:6      CSA_GAIN    Current Sense Amplifier Gain
 *                      0b00 = 5 V/V,  0b01 = 10 V/V
 *                      0b10 = 20 V/V  [DEFAULT],  0b11 = 40 V/V
 * 5        DIS_SEN     Sense Overcurrent Fault
 *                      0b = OCP enabled  [DEFAULT]
 *                      1b = OCP disabled
 * 4        CSA_CAL_A   Phase A Current Sense Amplifier Calibration
 *                      0b = Normal operation  [DEFAULT]
 *                      1b = Short inputs for offset calibration
 * 3        CSA_CAL_B   Phase B Current Sense Amplifier Calibration
 *                      0b = Normal operation  [DEFAULT]
 *                      1b = Short inputs for offset calibration
 * 2        CSA_CAL_C   Phase C Current Sense Amplifier Calibration
 *                      0b = Normal operation  [DEFAULT]
 *                      1b = Short inputs for offset calibration
 * 1:0      SEN_LVL     Sense OCP Voltage Level
 *                      0b00 = 0.25 V, 0b01 = 0.5 V
 *                      0b10 = 0.75 V, 0b11 = 1.0 V  [DEFAULT]
 * =========================================================== */

#define CSA_FET             (0U << 10)          // 0b=SPx (default), 1b=SHx
#define CSA_VREF_DIV        (1U << 9)           // 0b=VREF, 1b=VREF/2 (default)
#define CSA_LS_REF          (0U << 8)           // 0b=SHx-SPx (default), 1b=SHx-SNx
#define CSA_GAIN(x)         (((x) & 0x03U) << 6) // 00=5V/V, 01=10V/V, 10=20V/V(default), 11=40V/V
#define CSA_DIS_SEN         (0U << 5)           // 0b=OCP enabled (default), 1b=disabled
#define CSA_CAL_A           (0U << 4)           // 0b=normal (default), 1b=calibrate
#define CSA_CAL_B           (0U << 3)           // 0b=normal (default), 1b=calibrate
#define CSA_CAL_C           (0U << 2)           // 0b=normal (default), 1b=calibrate
#define CSA_SEN_LVL(x)      (((x) & 0x03U) << 0) // 00=0.25V, 01=0.5V, 10=0.75V, 11=1V(default)

/* CSA gain enum values */
#define DRV8323_GAIN_5VV    0U
#define DRV8323_GAIN_10VV   1U
#define DRV8323_GAIN_20VV   2U
#define DRV8323_GAIN_40VV   3U

/* =========================================================
 * Default register values at init
 * Target: AON6354 MOSFET, FOC 10kHz, AVDD=3.3V
 * ========================================================= */

/*
 * DRV_CTRL (0x02):
 *   OTW_REP=1  → báo overtemp warning ra nFAULT, MCU có thể giảm tải kịp thời
 *   DIS_GDF=0  → giữ gate drive fault protection
 *   DIS_CPUV=0 → giữ charge pump undervoltage protection
 *   PWM_MODE=6x→ FOC cần điều khiển độc lập từng FET
 */
#define DRV8323_DEFAULT_DRV_CTRL    (DRV8323_DRV_CTRL_OTW_REP)

/*
 * GATE_HS (0x03): Unlock + IDRIVEP=8 (260mA), IDRIVEN=8 (520mA)
 *
 *   I_source = Qg / t_rise = 35nC / 150ns = 233mA → index 8 = 260mA
 *   I_sink   = 2× source để turn-off nhanh hơn     → index 8 = 520mA
 */
#define DRV8323_DEFAULT_GATE_HS     (DRV8323_GATE_HS_UNLOCK         | \
                                     DRV8323_GATE_HS_IDRIVEP(8)     | \
                                     DRV8323_GATE_HS_IDRIVEN(8))

/*
 * GATE_LS (0x04): CBC=0, TDRIVE=500ns, IDRIVEP=8, IDRIVEN=8
 *
 *   CBC=0     → latched OCP, không retry tự động từng cycle
 *   TDRIVE=0  → 500ns, lớn hơn t_drive_min = 35nC/260mA = 135ns
 */
#define DRV8323_DEFAULT_GATE_LS     (DRV8323_GATE_LS_TDRIVE(0)      | \
                                     DRV8323_GATE_LS_IDRIVEP(8)     | \
                                     DRV8323_GATE_LS_IDRIVEN(8))

/*
 * OCP_CTRL (0x05):
 *   DEAD_TIME=2 (400ns): t_fall≈150ns, margin × 2.5 → an toàn khi FET nóng
 *   OCP_MODE=0 (latched): fault → tắt hẳn cho đến khi MCU gửi CLR_FLT
 *   OCP_DEG=1  (4us): lọc spike ngắn do switching, tránh false-trip
 *   VDS_LVL=2  (0.20V): Vds = Ipeak × Rds_hot × margin
 *                       = 20A × 6.6mΩ × 1.5 = 198mV → 0.20V
 *   !! Điều chỉnh VDS_LVL theo Ipeak thực tế: VDS_LVL = Ipeak × 6.6mΩ × 1.5 !!
 */
#define DRV8323_DEFAULT_OCP_CTRL    (DRV8323_OCP_DEAD_TIME(2)       | \
                                     DRV8323_OCP_MODE(0)             | \
                                     DRV8323_OCP_DEG(1)              | \
                                     DRV8323_OCP_VDS_LVL(2))

/*
 * CSA_CTRL (0x06):
 *   SEN_LVL=1 (0.5V): ngưỡng OCP của CSA path
 *   GAIN=10V/V: Gain ≤ (AVDD/2) / (Ipeak × Rshunt)
 *               Ví dụ Rshunt=5mΩ, Ipeak=20A: ≤ 1.65/0.1 = 16.5 → chọn 10V/V
 *   VREF_DIV=1: output center = AVDD/2 = 1.65V → đo dòng 2 chiều (FOC cần)
 *   CSA_FET=0 : low-side shunt sensing
 *   !! Điều chỉnh GAIN theo Rshunt thực tế !!
 */
#define DRV8323_DEFAULT_CSA_CTRL    (DRV8323_CSA_SEN_LVL(1)              | \
                                     DRV8323_CSA_GAIN(DRV8323_GAIN_10VV) | \
                                     DRV8323_CSA_VREF_DIV)

#endif /* DRV8323_DRIVER_INC_DRV8323_CONFIG_H_ */
