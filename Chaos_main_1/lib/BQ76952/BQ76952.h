#ifndef BQ76952_H
#define BQ76952_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 * BQ76952 DRIVER - Header File
 * Chip: Texas Instruments BQ76952
 * Giao tiếp: I2C, 100kHz, không CRC
 * Địa chỉ I2C: 0x10 (8-bit format, HAL tự xử lý shift)
 * Tham khảo: Datasheet SLUSE13B + TRM SLUUBY2B
 * ============================================================ */


/* ============================================================
 * I2C
 * ============================================================ */
#define BQ76952_ADDR_WRITE      0x10
#define BQ76952_ADDR_READ       0x11
#define BQ76952_I2C_TIMEOUT     500    // ms


/* ============================================================
 * DIRECT COMMAND ADDRESSES (TRM mục 12.1)
 * ============================================================ */

// Control & Status
#define BQ_CMD_CONTROL_STATUS   0x00
#define BQ_CMD_SAFETY_ALERT_A   0x02   // R, 1 byte
#define BQ_CMD_SAFETY_STATUS_A  0x03   // R, 1 byte
#define BQ_CMD_SAFETY_ALERT_B   0x04   // R, 1 byte
#define BQ_CMD_SAFETY_STATUS_B  0x05   // R, 1 byte
#define BQ_CMD_SAFETY_ALERT_C   0x06   // R, 1 byte
#define BQ_CMD_SAFETY_STATUS_C  0x07   // R, 1 byte
#define BQ_CMD_BATTERY_STATUS   0x12   // R, 2 bytes
#define BQ_CMD_ALARM_STATUS     0x62   // R, 2 bytes
#define BQ_CMD_ALARM_RAW_STATUS 0x64   // R, 2 bytes
#define BQ_CMD_ALARM_ENABLE     0x66   // R/W, 2 bytes
#define BQ_CMD_FET_STATUS       0x7F   // R, 1 byte

// Cell Voltages — int16_t, mV
#define BQ_CMD_CELL1_VOLTAGE    0x14
#define BQ_CMD_CELL2_VOLTAGE    0x16
#define BQ_CMD_CELL3_VOLTAGE    0x18
#define BQ_CMD_CELL4_VOLTAGE    0x1A
#define BQ_CMD_CELL5_VOLTAGE    0x1C
#define BQ_CMD_CELL6_VOLTAGE    0x1E
#define BQ_CMD_CELL7_VOLTAGE    0x20
#define BQ_CMD_CELL8_VOLTAGE    0x22
#define BQ_CMD_CELL9_VOLTAGE    0x24
#define BQ_CMD_CELL10_VOLTAGE   0x26
#define BQ_CMD_CELL11_VOLTAGE   0x28
#define BQ_CMD_CELL12_VOLTAGE   0x2A
#define BQ_CMD_CELL13_VOLTAGE   0x2C
#define BQ_CMD_CELL14_VOLTAGE   0x2E
#define BQ_CMD_CELL15_VOLTAGE   0x30
#define BQ_CMD_CELL16_VOLTAGE   0x32

// Stack & Pack — uint16_t
#define BQ_CMD_STACK_VOLTAGE    0x34
#define BQ_CMD_PACK_VOLTAGE     0x36

// Current — int16_t, userA
#define BQ_CMD_CC2_CURRENT      0x3A

// Temperature — uint16_t, 0.1K (trừ 2731 → 0.1°C)
#define BQ_CMD_TEMP_INTERNAL    0x68
#define BQ_CMD_TEMP_TS1         0x70
#define BQ_CMD_TEMP_TS2         0x72
#define BQ_CMD_TEMP_TS3         0x74


/* ============================================================
 * SUBCOMMAND (TRM mục 12.3, 12.4)
 * ============================================================ */
#define BQ_SUBCMD_ADDR          0x3E
#define BQ_SUBCMD_DATA_ADDR     0x40
#define BQ_SUBCMD_CHECKSUM_ADDR 0x60
#define BQ_SUBCMD_LENGTH_ADDR   0x61

#define BQ_SUBCMD_ENTER_CFG     0x0090
#define BQ_SUBCMD_EXIT_CFG      0x0092
#define BQ_SUBCMD_FET_ENABLE    0x0022  // Toggle FET_EN bit
#define BQ_SUBCMD_DSG_PDSG_OFF  0x0093  // Tắt DSG + PDSG (latched)
#define BQ_SUBCMD_CHG_PCHG_OFF  0x0094  // Tắt CHG + PCHG (latched)
#define BQ_SUBCMD_ALL_FETS_OFF  0x0095  // Tắt tất cả FET (latched)
#define BQ_SUBCMD_ALL_FETS_ON   0x0096  // Cho phép tất cả FET bật lại
#define BQ_SUBCMD_FET_CONTROL   0x0097  // Điều khiển từng FET riêng lẻ
#define BQ_SUBCMD_RESET         0x0012
#define BQ_SUBCMD_SLEEP_EN      0x0099
#define BQ_SUBCMD_SLEEP_DIS     0x009A

/* FET_CONTROL (0x0097) data byte bits */
#define BQ_FETCTRL_DSG_OFF      (1u << 0)
#define BQ_FETCTRL_PDSG_OFF     (1u << 1)
#define BQ_FETCTRL_CHG_OFF      (1u << 2)
#define BQ_FETCTRL_PCHG_OFF     (1u << 3)

/* Enabled Protections A (0x9261) bit masks */
#define BQ_PROTA_SCD            (1u << 7)
#define BQ_PROTA_OCD2           (1u << 6)
#define BQ_PROTA_OCD1           (1u << 5)
#define BQ_PROTA_OCC            (1u << 4)
#define BQ_PROTA_COV            (1u << 3)
#define BQ_PROTA_CUV            (1u << 2)

/* Enabled Protections B (0x9262) bit masks */
#define BQ_PROTB_OTF            (1u << 7)
#define BQ_PROTB_OTINT          (1u << 6)
#define BQ_PROTB_OTD            (1u << 5)
#define BQ_PROTB_OTC            (1u << 4)
#define BQ_PROTB_UTINT          (1u << 2)
#define BQ_PROTB_UTD            (1u << 1)
#define BQ_PROTB_UTC            (1u << 0)
#define BQ_PROTB_ALL_TEMP       (BQ_PROTB_OTC | BQ_PROTB_OTD | BQ_PROTB_OTF | \
                                 BQ_PROTB_UTC | BQ_PROTB_UTD)


/* ============================================================
 * DATA MEMORY + DATA — Settings:Configuration (TRM 13.3.2)
 * ============================================================ */
#define BQ_MEM_POWER_CONFIG       0x9234  // H2, 2 bytes
#define BQ_MEM_REG12_CONFIG       0x9236  // H1, 1 byte
#define BQ_MEM_CFETOFF_PIN_CFG    0x92FA  // H1, 1 byte
#define BQ_MEM_DFETOFF_PIN_CFG    0x92FB  // H1, 1 byte
#define BQ_MEM_DA_CONFIG          0x9303  // H1, 1 byte
#define BQ_MEM_VCELL_MODE         0x9304  // H2, 2 bytes

#define BQ_VAL_POWER_CONFIG       0x2895
#define BQ_VAL_REG12_CONFIG       0x00
#define BQ_VAL_CFETOFF_PIN_CFG    0x8A
#define BQ_VAL_DFETOFF_PIN_CFG    0x8A
#define BQ_VAL_DA_CONFIG          0x06    // USER_VOLTS=centivolt, USER_AMPS=10mA
#define BQ_VAL_VCELL_MODE         0x8007  // 4 cell: Cell1,2,3,16


/* ============================================================
 * DATA MEMORY + DATA — Settings:Protection (TRM 13.3.3)
 * ============================================================ */
#define BQ_MEM_ENABLED_PROT_A     0x9261  // H1
#define BQ_MEM_ENABLED_PROT_B     0x9262  // H1
#define BQ_MEM_ENABLED_PROT_C     0x9263  // H1
#define BQ_MEM_CHG_FET_PROT_A     0x9265  // H1
#define BQ_MEM_CHG_FET_PROT_B     0x9266  // H1
#define BQ_MEM_CHG_FET_PROT_C     0x9267  // H1
#define BQ_MEM_DSG_FET_PROT_A     0x9269  // H1
#define BQ_MEM_DSG_FET_PROT_B     0x926A  // H1
#define BQ_MEM_DSG_FET_PROT_C     0x926B  // H1

#define BQ_VAL_ENABLED_PROT_A     0xBC
#define BQ_VAL_ENABLED_PROT_B     0x00
#define BQ_VAL_ENABLED_PROT_C     0x00
#define BQ_VAL_CHG_FET_PROT_A     0x98
#define BQ_VAL_CHG_FET_PROT_B     0x00
#define BQ_VAL_CHG_FET_PROT_C     0x00
#define BQ_VAL_DSG_FET_PROT_A     0xE4
#define BQ_VAL_DSG_FET_PROT_B     0x00
#define BQ_VAL_DSG_FET_PROT_C     0x00


/* ============================================================
 * DATA MEMORY + DATA — Settings:FET (TRM 13.3.6)
 * ============================================================ */
#define BQ_MEM_FET_OPTIONS        0x9308  // H1
#define BQ_MEM_PREDCHG_TIMEOUT    0x930E  // U1, 10ms/step
#define BQ_MEM_PREDCHG_STOP_DELTA 0x930F  // U1, 10mV/step

#define BQ_VAL_FET_OPTIONS        0x3F
#define BQ_VAL_PREDCHG_TIMEOUT    100     // 1000ms
#define BQ_VAL_PREDCHG_STOP_DELTA 80      // 800mV


/* ============================================================
 * DATA MEMORY + DATA — Settings:Current Thresholds (TRM 13.3.7)
 * ============================================================ */
#define BQ_MEM_DSG_CURR_THRESH    0x9310  // I2, userA
#define BQ_MEM_CHG_CURR_THRESH    0x9312  // I2, userA

#define BQ_VAL_DSG_CURR_THRESH    15000   // 150A
#define BQ_VAL_CHG_CURR_THRESH    1000    // 10A


/* ============================================================
 * DATA MEMORY + DATA — Settings:Cell Balancing (TRM 13.3.11)
 * ============================================================ */
#define BQ_MEM_CB_CONFIG          0x9335  // H1

#define BQ_VAL_CB_CONFIG          0x00


/* ============================================================
 * DATA MEMORY + DATA — Protections:CUV (TRM 13.6.1)
 * ============================================================ */
#define BQ_MEM_CUV_THRESHOLD      0x9275  // U1, 50.6mV/step
#define BQ_MEM_CUV_DELAY          0x9276  // U2, 3.3ms/step

#define BQ_VAL_CUV_THRESHOLD      50
#define BQ_VAL_CUV_DELAY          74


/* ============================================================
 * DATA MEMORY + DATA — Protections:COV (TRM 13.6.2)
 * ============================================================ */
#define BQ_MEM_COV_THRESHOLD      0x9278  // U1, 50.6mV/step
#define BQ_MEM_COV_DELAY          0x9279  // U2, 3.3ms/step

#define BQ_VAL_COV_THRESHOLD      83
#define BQ_VAL_COV_DELAY          74


/* ============================================================
 * DATA MEMORY + DATA — Protections:OCC (TRM 13.6.4)
 * ============================================================ */
#define BQ_MEM_OCC_THRESHOLD      0x9280  // U1, 2mV/step
#define BQ_MEM_OCC_DELAY          0x9281  // U1, 3.3ms/step

#define BQ_VAL_OCC_THRESHOLD      2
#define BQ_VAL_OCC_DELAY          4


/* ============================================================
 * DATA MEMORY + DATA — Protections:OCD1 (TRM 13.6.5)
 * ============================================================ */
#define BQ_MEM_OCD1_THRESHOLD     0x9282  // U1, 2mV/step
#define BQ_MEM_OCD1_DELAY         0x9283  // U1, 3.3ms/step

#define BQ_VAL_OCD1_THRESHOLD     28
#define BQ_VAL_OCD1_DELAY         1


/* ============================================================
 * DATA MEMORY + DATA — Protections:SCD (TRM 13.6.7)
 * ============================================================ */
#define BQ_MEM_SCD_THRESHOLD      0x9286  // U1, enum 0-15
#define BQ_MEM_SCD_DELAY          0x9287  // U1, 15us/step
#define BQ_MEM_SCD_RECOV_TIME     0x9294  // U1, s

#define BQ_VAL_SCD_THRESHOLD      3
#define BQ_VAL_SCD_DELAY          2
#define BQ_VAL_SCD_RECOV_TIME     0x00


/* ============================================================
 * DATA MEMORY + DATA — Protections:OTC (TRM 13.6.12)
 * ============================================================ */
#define BQ_MEM_OTC_THRESHOLD      0x929A  // I1, C
#define BQ_MEM_OTC_DELAY          0x929B  // U1, s
#define BQ_MEM_OTC_RECOVERY       0x929C  // I1, C

#define BQ_VAL_OTC_THRESHOLD      55
#define BQ_VAL_OTC_DELAY          5
#define BQ_VAL_OTC_RECOVERY       45


/* ============================================================
 * DATA MEMORY + DATA — Protections:OTD (TRM 13.6.13)
 * ============================================================ */
#define BQ_MEM_OTD_THRESHOLD      0x929D  // I1, C
#define BQ_MEM_OTD_DELAY          0x929E  // U1, s
#define BQ_MEM_OTD_RECOVERY       0x929F  // I1, C

#define BQ_VAL_OTD_THRESHOLD      60
#define BQ_VAL_OTD_DELAY          5
#define BQ_VAL_OTD_RECOVERY       48


/* ============================================================
 * DATA MEMORY + DATA — Protections:UTC (TRM 13.6.16)
 * ============================================================ */
#define BQ_MEM_UTC_THRESHOLD      0x92A6  // I1, C
#define BQ_MEM_UTC_DELAY          0x92A7  // U1, s
#define BQ_MEM_UTC_RECOVERY       0x92A8  // I1, C

#define BQ_VAL_UTC_THRESHOLD      0
#define BQ_VAL_UTC_DELAY          5
#define BQ_VAL_UTC_RECOVERY       5


/* ============================================================
 * DATA MEMORY + DATA — Protections:UTD (TRM 13.6.17)
 * ============================================================ */
#define BQ_MEM_UTD_THRESHOLD      0x92A9  // I1, C
#define BQ_MEM_UTD_DELAY          0x92AA  // U1, s
#define BQ_MEM_UTD_RECOVERY       0x92AB  // I1, C

#define BQ_VAL_UTD_THRESHOLD      0
#define BQ_VAL_UTD_DELAY          5
#define BQ_VAL_UTD_RECOVERY       5


/* ============================================================
 * DATA MEMORY + DATA — Protections:Recovery (TRM 13.6.19)
 * ============================================================ */
#define BQ_MEM_RECOVERY_TIME      0x92AF  // U1, s

#define BQ_VAL_RECOVERY_TIME      0x00


/* ============================================================
 * Shunt
 * ============================================================ */
#define BQ_SENSE_RESISTOR_mOhm   1.0f

#define BQ_MEM_MFG_STATUS_INIT    0x9343  // H1
#define BQ_VAL_MFG_STATUS_INIT    0x50    // PF_EN=1, FET_EN=1


/* ============================================================
 * DATA STRUCTURES
 * ============================================================ */

typedef enum {
    BQ_OK               = 0x00,
    BQ_ERR_I2C          = 0x01,
    BQ_ERR_TIMEOUT      = 0x02,
    BQ_ERR_INVALID_DATA = 0x03,
    BQ_ERR_CFG_MODE     = 0x04,
} BQ76952_Status_t;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    int16_t  cell_mV[16];
    uint16_t stack_voltage;
    uint16_t pack_voltage;
    int16_t  current;
    int16_t  temp_internal_dC;
    int16_t  temp_ts1_dC;
    int16_t  temp_ts2_dC;
    int16_t  temp_ts3_dC;

    uint8_t  fet_status;
    uint16_t battery_status;
    uint16_t alarm_status;
    uint8_t mfg_status;
    uint8_t chg_pump;
    uint16_t alarm_raw;

    uint8_t  safety_alert_a;
    uint8_t  safety_alert_b;
    uint8_t  safety_alert_c;
    uint8_t  safety_status_a;
    uint8_t  safety_status_b;
    uint8_t  safety_status_c;
} BQ76952_Data_t;


/* ============================================================
 * DEBUG STRUCT — readback toàn bộ mem đã ghi
 * ============================================================ */
typedef struct {

    /* ── CONFIG READBACK — Settings ───────────────────────── */
    uint8_t  cfg_ok;              // 1 = tất cả step OK
    uint16_t cfg_power_config;    // 0x9234
    uint8_t  cfg_reg12;           // 0x9236
    uint8_t  cfg_cfetoff;         // 0x92FA
    uint8_t  cfg_dfetoff;         // 0x92FB
    uint8_t  cfg_da_config;       // 0x9303
    uint16_t cfg_vcell_mode;      // 0x9304

    /* ── CONFIG READBACK — Protection enables ─────────────── */
    uint8_t  cfg_enabled_prot_a;  // 0x9261
    uint8_t  cfg_enabled_prot_b;  // 0x9262
    uint8_t  cfg_enabled_prot_c;  // 0x9263
    uint8_t  cfg_chg_fet_prot_a;  // 0x9265
    uint8_t  cfg_chg_fet_prot_b;  // 0x9266
    uint8_t  cfg_chg_fet_prot_c;  // 0x9267
    uint8_t  cfg_dsg_fet_prot_a;  // 0x9269
    uint8_t  cfg_dsg_fet_prot_b;  // 0x926A
    uint8_t  cfg_dsg_fet_prot_c;  // 0x926B

    /* ── CONFIG READBACK — FET ────────────────────────────── */
    uint8_t  cfg_fet_options;     // 0x9308
    uint8_t  cfg_predchg_timeout; // 0x930E
    uint8_t  cfg_predchg_delta;   // 0x930F

    /* ── CONFIG READBACK — Current Thresholds ─────────────── */
    uint16_t cfg_dsg_curr_thresh; // 0x9310
    uint16_t cfg_chg_curr_thresh; // 0x9312

    /* ── CONFIG READBACK — Cell Balancing ─────────────────── */
    uint8_t  cfg_cb_config;       // 0x9335

    /* ── CONFIG READBACK — Protection thresholds ──────────── */
    uint8_t  cfg_cuv_threshold;   // 0x9275
    uint16_t cfg_cuv_delay;       // 0x9276
    uint8_t  cfg_cov_threshold;   // 0x9278
    uint16_t cfg_cov_delay;       // 0x9279
    uint8_t  cfg_occ_threshold;   // 0x9280
    uint8_t  cfg_occ_delay;       // 0x9281
    uint8_t  cfg_ocd1_threshold;  // 0x9282
    uint8_t  cfg_ocd1_delay;      // 0x9283
    uint8_t  cfg_scd_threshold;   // 0x9286
    uint8_t  cfg_scd_delay;       // 0x9287
    uint8_t  cfg_scd_recov_time;  // 0x9294
    uint8_t  cfg_otc_threshold;   // 0x929A
    uint8_t  cfg_otc_delay;       // 0x929B
    uint8_t  cfg_otc_recovery;    // 0x929C
    uint8_t  cfg_otd_threshold;   // 0x929D
    uint8_t  cfg_otd_delay;       // 0x929E
    uint8_t  cfg_otd_recovery;    // 0x929F
    uint8_t  cfg_utc_threshold;   // 0x92A6
    uint8_t  cfg_utc_delay;       // 0x92A7
    uint8_t  cfg_utc_recovery;    // 0x92A8
    uint8_t  cfg_utd_threshold;   // 0x92A9
    uint8_t  cfg_utd_delay;       // 0x92AA
    uint8_t  cfg_utd_recovery;    // 0x92AB
    uint8_t  cfg_recovery_time;   // 0x92AF

    /* ── STEP TRACKING ───────────────────────────────────── */
    uint8_t  step_enter_cfg;
    uint8_t  step_power_config;
    uint8_t  step_reg12;
    uint8_t  step_da_config;
    uint8_t  step_vcell_mode;
    uint8_t  step_cfetoff;
    uint8_t  step_dfetoff;
    uint8_t  step_enabled_prot;
    uint8_t  step_chg_fet_prot;
    uint8_t  step_dsg_fet_prot;
    uint8_t  step_fet_options;
    uint8_t  step_curr_thresh;
    uint8_t  step_predchg;
    uint8_t  step_cov;
    uint8_t  step_cuv;
    uint8_t  step_occ;
    uint8_t  step_ocd1;
    uint8_t  step_scd;
    uint8_t  step_otc;
    uint8_t  step_otd;
    uint8_t  step_utc;
    uint8_t  step_utd;
    uint8_t  step_recovery;
    uint8_t  step_cb_config;
    uint8_t  step_exit_cfg;
    uint8_t  step_fet_enable;

    /* ── RAW CELL BYTES ──────────────────────────────────── */
    uint8_t  raw_cell[16][2];

    /* ── RUNTIME ─────────────────────────────────────────── */
    uint32_t read_count;
    uint32_t read_err_count;
    uint32_t i2c_err_count;
    uint8_t  last_read_status;

    /* ── LIVE STATUS — đọc trực tiếp từ IC ─────────────── */
    uint8_t  fet_status;          // 0x7F — bit0:CHG, bit1:PCHG, bit2:DSG, bit3:PDSG, bit4:DCHG_PIN, bit5:DDSG_PIN, bit6:ALRT_PIN
    uint16_t battery_status;      // 0x12
    uint16_t alarm_status;        // 0x62
    uint16_t alarm_raw;           // 0x64
    uint8_t  mfg_status;          // subcmd 0x0057 — bit4:FET_EN, bit6:PF_EN
    uint8_t  chg_pump;            // 0x9309 — bit0:CPEN phải = 1

    uint8_t  safety_alert_a;      // 0x02
    uint8_t  safety_alert_b;      // 0x04
    uint8_t  safety_alert_c;      // 0x06
    uint8_t  safety_status_a;     // 0x03
    uint8_t  safety_status_b;     // 0x05
    uint8_t  safety_status_c;     // 0x07

    /* ── DECODED FLAGS — tính từ live status ──────────────── */
    uint8_t  chg_fet_on;          // fet_status bit0
    uint8_t  dsg_fet_on;          // fet_status bit2
    uint8_t  pdsg_fet_on;         // fet_status bit3
    uint8_t  dchg_pin;            // fet_status bit4 — IC đang drive CHG gate
    uint8_t  ddsg_pin;            // fet_status bit5 — IC đang drive DSG gate
    uint8_t  fet_en;              // mfg_status bit4
    uint8_t  in_cfg_mode;         // battery_status bit0 (CFGUPDATE)
    uint8_t  sleep_en;            // battery_status bit2
    uint8_t  por;                 // battery_status bit3 — POR chưa clear
    uint8_t  in_sleep_mode;       // battery_status bit15
    uint8_t  safety_fault;        // battery_status bit11 (SS)
    uint8_t  pf_fault;            // battery_status bit12 (PF)
    uint8_t  safety_triggered;    // alert a|b|c != 0
    uint8_t  estop_active;        // FET tắt + không có fault

} BQ76952_Debug_t;


/* ============================================================
 * DIAGNOSTIC
 * ============================================================ */
typedef enum {
    BQ_DIAG_PING = 0, BQ_DIAG_PING_ALT, BQ_DIAG_BUS_RESET,
    BQ_DIAG_SLOW_CLK, BQ_DIAG_I2C_STATE, BQ_DIAG_SCAN_ALL,
} BQ76952_DiagMode_t;

typedef struct {
    HAL_StatusTypeDef i2c_status;
    uint32_t i2c_error;
    uint32_t sr1;
    uint32_t sr2;
    uint8_t  found_addrs[25];
    uint8_t  found_count;
} BQ76952_DiagResult_t;

void BQ76952_Diagnose(BQ76952_Data_t *dev, BQ76952_DiagMode_t mode,
                      GPIO_TypeDef *scl_port, uint16_t scl_pin,
                      BQ76952_DiagResult_t *result);


/* ============================================================
 * FUNCTION PROTOTYPES
 * ============================================================ */

// Init & Config
BQ76952_Status_t BQ76952_Init(I2C_HandleTypeDef *hi2c, BQ76952_Data_t *dev, BQ76952_Debug_t *dbg);
BQ76952_Status_t BQ76952_ConfigAll(BQ76952_Data_t *dev, BQ76952_Debug_t *dbg);

// Low-level I2C
BQ76952_Status_t BQ76952_ReadRegister(BQ76952_Data_t *dev, uint8_t reg, uint8_t *buf, uint8_t len);
BQ76952_Status_t BQ76952_WriteRegister(BQ76952_Data_t *dev, uint8_t reg, uint8_t *buf, uint8_t len);
BQ76952_Status_t BQ76952_SendSubcommand(BQ76952_Data_t *dev, uint16_t subcmd);
BQ76952_Status_t BQ76952_ReadDataMemory(BQ76952_Data_t *dev, uint16_t addr, uint8_t *buf, uint8_t len);
BQ76952_Status_t BQ76952_WriteDataMemory(BQ76952_Data_t *dev, uint16_t addr, uint8_t *buf, uint8_t len);

// Config mode
BQ76952_Status_t BQ76952_EnterConfigMode(BQ76952_Data_t *dev);
BQ76952_Status_t BQ76952_ExitConfigMode(BQ76952_Data_t *dev);

// FET Control
BQ76952_Status_t BQ76952_EnableFET(BQ76952_Data_t *dev);    // Toggle FET_EN
BQ76952_Status_t BQ76952_AllFetsOn(BQ76952_Data_t *dev);    // 0x0096 — clear all latch
BQ76952_Status_t BQ76952_AllFetsOff(BQ76952_Data_t *dev);   // 0x0095 — latch all off
BQ76952_Status_t BQ76952_ChgFetOff(BQ76952_Data_t *dev);    // 0x0094 — latch CHG+PCHG off
BQ76952_Status_t BQ76952_DsgFetOff(BQ76952_Data_t *dev);    // 0x0093 — latch DSG+PDSG off
BQ76952_Status_t BQ76952_FetControl(BQ76952_Data_t *dev, uint8_t ctl);  // 0x0097 — per-FET

// Protection Control
BQ76952_Status_t BQ76952_DisableAllProtections(BQ76952_Data_t *dev);
BQ76952_Status_t BQ76952_DisableTempProtections(BQ76952_Data_t *dev);
BQ76952_Status_t BQ76952_EnableTempProtections(BQ76952_Data_t *dev);

// Read data
BQ76952_Status_t BQ76952_ReadAllData(BQ76952_Data_t *dev);
BQ76952_Status_t BQ76952_ReadCellVoltages(BQ76952_Data_t *dev);
BQ76952_Status_t BQ76952_ReadCurrent(BQ76952_Data_t *dev);
BQ76952_Status_t BQ76952_ReadTemperatures(BQ76952_Data_t *dev);
BQ76952_Status_t BQ76952_ReadStatus(BQ76952_Data_t *dev);

// Debug
void BQ76952_UpdateDebug(BQ76952_Data_t *dev, BQ76952_Debug_t *dbg);
void BQ76952_VerifyConfig(BQ76952_Data_t *dev, BQ76952_Debug_t *dbg);

// Utility
float BQ76952_GetStackVoltage_V(BQ76952_Data_t *dev);
float BQ76952_GetCurrent_A(BQ76952_Data_t *dev);
float BQ76952_GetTemp_C(int16_t temp_dC);
bool  BQ76952_HasFault(BQ76952_Data_t *dev);
bool  BQ76952_IsCHG_FET_ON(BQ76952_Data_t *dev);
bool  BQ76952_IsDSG_FET_ON(BQ76952_Data_t *dev);

#endif /* BQ76952_H */
