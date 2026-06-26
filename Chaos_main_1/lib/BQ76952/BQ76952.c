/* ============================================================
 * BQ76952 DRIVER - Source File
 * ============================================================ */

#include "BQ76952.h"
#include <string.h>

/* ============================================================
 * PRIVATE HELPERS
 * ============================================================ */

static uint8_t _calc_checksum(uint8_t *data, uint8_t len) {
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++) sum += data[i];
    return ~sum;
}

static BQ76952_Status_t _write1(BQ76952_Data_t *dev, uint16_t addr,
                                 uint8_t val, uint8_t *step) {
    uint8_t buf = val;
    BQ76952_Status_t ret = BQ76952_WriteDataMemory(dev, addr, &buf, 1);
    if (step) *step = (uint8_t)ret;
    return ret;
}

static BQ76952_Status_t _write2(BQ76952_Data_t *dev, uint16_t addr,
                                 uint16_t val, uint8_t *step) {
    uint8_t buf[2] = { (uint8_t)(val & 0xFF), (uint8_t)(val >> 8) };
    BQ76952_Status_t ret = BQ76952_WriteDataMemory(dev, addr, buf, 2);
    if (step) *step = (uint8_t)ret;
    return ret;
}

static uint8_t _read1(BQ76952_Data_t *dev, uint16_t addr) {
    uint8_t buf = 0xFF;
    BQ76952_ReadDataMemory(dev, addr, &buf, 1);
    return buf;
}

static uint16_t _read2(BQ76952_Data_t *dev, uint16_t addr) {
    uint8_t buf[2] = {0xAD, 0xDE};
    if (BQ76952_ReadDataMemory(dev, addr, buf, 2) == BQ_OK)
        return (uint16_t)(buf[0] | (buf[1] << 8));
    return 0xDEAD;
}


/* ============================================================
 * LOW-LEVEL I2C
 * ============================================================ */

BQ76952_Status_t BQ76952_ReadRegister(BQ76952_Data_t *dev, uint8_t reg,
                                       uint8_t *buf, uint8_t len) {
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(
        dev->hi2c, BQ76952_ADDR_WRITE, reg,
        I2C_MEMADD_SIZE_8BIT, buf, len, BQ76952_I2C_TIMEOUT);
    return (ret == HAL_OK) ? BQ_OK : BQ_ERR_I2C;
}

BQ76952_Status_t BQ76952_WriteRegister(BQ76952_Data_t *dev, uint8_t reg,
                                        uint8_t *buf, uint8_t len) {
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Write(
        dev->hi2c, BQ76952_ADDR_WRITE, reg,
        I2C_MEMADD_SIZE_8BIT, buf, len, BQ76952_I2C_TIMEOUT);
    return (ret == HAL_OK) ? BQ_OK : BQ_ERR_I2C;
}

BQ76952_Status_t BQ76952_SendSubcommand(BQ76952_Data_t *dev, uint16_t subcmd) {
    uint8_t buf[2] = {(uint8_t)(subcmd & 0xFF), (uint8_t)(subcmd >> 8)};
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Write(
        dev->hi2c, BQ76952_ADDR_WRITE, BQ_SUBCMD_ADDR,
        I2C_MEMADD_SIZE_8BIT, buf, 2, BQ76952_I2C_TIMEOUT);
    return (ret == HAL_OK) ? BQ_OK : BQ_ERR_I2C;
}

BQ76952_Status_t BQ76952_ReadDataMemory(BQ76952_Data_t *dev, uint16_t addr,
                                         uint8_t *buf, uint8_t len) {
    BQ76952_Status_t ret = BQ76952_SendSubcommand(dev, addr);
    if (ret != BQ_OK) return ret;
    HAL_Delay(5);
    HAL_StatusTypeDef hal_ret = HAL_I2C_Mem_Read(
        dev->hi2c, BQ76952_ADDR_WRITE, BQ_SUBCMD_DATA_ADDR,
        I2C_MEMADD_SIZE_8BIT, buf, len, BQ76952_I2C_TIMEOUT);
    return (hal_ret == HAL_OK) ? BQ_OK : BQ_ERR_I2C;
}

BQ76952_Status_t BQ76952_WriteDataMemory(BQ76952_Data_t *dev, uint16_t addr,
                                          uint8_t *buf, uint8_t len) {
    HAL_StatusTypeDef ret;

    BQ76952_Status_t s = BQ76952_SendSubcommand(dev, addr);
    if (s != BQ_OK) return s;

    ret = HAL_I2C_Mem_Write(dev->hi2c, BQ76952_ADDR_WRITE,
                             BQ_SUBCMD_DATA_ADDR, I2C_MEMADD_SIZE_8BIT,
                             buf, len, BQ76952_I2C_TIMEOUT);
    if (ret != HAL_OK) return BQ_ERR_I2C;

    uint8_t cs_src[2 + 32];
    cs_src[0] = (uint8_t)(addr & 0xFF);
    cs_src[1] = (uint8_t)(addr >> 8);
    memcpy(&cs_src[2], buf, len);
    uint8_t checksum = _calc_checksum(cs_src, 2 + len);
    uint8_t cs_len[2] = { checksum, (uint8_t)(4 + len) };

    ret = HAL_I2C_Mem_Write(dev->hi2c, BQ76952_ADDR_WRITE,
                             BQ_SUBCMD_CHECKSUM_ADDR, I2C_MEMADD_SIZE_8BIT,
                             cs_len, 2, BQ76952_I2C_TIMEOUT);
    if (ret != HAL_OK) return BQ_ERR_I2C;

    HAL_Delay(10);
    return BQ_OK;
}


/* ============================================================
 * CONFIG MODE
 * ============================================================ */

BQ76952_Status_t BQ76952_EnterConfigMode(BQ76952_Data_t *dev) {
    BQ76952_Status_t ret = BQ76952_SendSubcommand(dev, BQ_SUBCMD_ENTER_CFG);
    HAL_Delay(20);
    return ret;
}

BQ76952_Status_t BQ76952_ExitConfigMode(BQ76952_Data_t *dev) {
    BQ76952_Status_t ret = BQ76952_SendSubcommand(dev, BQ_SUBCMD_EXIT_CFG);
    HAL_Delay(20);
    return ret;
}


/* ============================================================
 * FET CONTROL
 * ============================================================ */

BQ76952_Status_t BQ76952_EnableFET(BQ76952_Data_t *dev) {
    BQ76952_Status_t ret = BQ76952_SendSubcommand(dev, BQ_SUBCMD_FET_ENABLE);
    HAL_Delay(20);
    return ret;
}

BQ76952_Status_t BQ76952_AllFetsOn(BQ76952_Data_t *dev) {
    BQ76952_Status_t ret = BQ76952_SendSubcommand(dev, BQ_SUBCMD_ALL_FETS_ON);
    HAL_Delay(20);
    return ret;
}

BQ76952_Status_t BQ76952_AllFetsOff(BQ76952_Data_t *dev) {
    BQ76952_Status_t ret = BQ76952_SendSubcommand(dev, BQ_SUBCMD_ALL_FETS_OFF);
    HAL_Delay(20);
    return ret;
}

BQ76952_Status_t BQ76952_ChgFetOff(BQ76952_Data_t *dev) {
    BQ76952_Status_t ret = BQ76952_SendSubcommand(dev, BQ_SUBCMD_CHG_PCHG_OFF);
    HAL_Delay(20);
    return ret;
}

BQ76952_Status_t BQ76952_DsgFetOff(BQ76952_Data_t *dev) {
    BQ76952_Status_t ret = BQ76952_SendSubcommand(dev, BQ_SUBCMD_DSG_PDSG_OFF);
    HAL_Delay(20);
    return ret;
}

BQ76952_Status_t BQ76952_FetControl(BQ76952_Data_t *dev, uint8_t ctl) {
    BQ76952_Status_t ret = BQ76952_SendSubcommand(dev, BQ_SUBCMD_FET_CONTROL);
    if (ret != BQ_OK) return ret;

    HAL_StatusTypeDef hal = HAL_I2C_Mem_Write(
        dev->hi2c, BQ76952_ADDR_WRITE, BQ_SUBCMD_DATA_ADDR,
        I2C_MEMADD_SIZE_8BIT, &ctl, 1, BQ76952_I2C_TIMEOUT);
    if (hal != HAL_OK) return BQ_ERR_I2C;

    uint8_t cs_src[3] = { 0x97, 0x00, ctl };
    uint8_t checksum = _calc_checksum(cs_src, 3);
    uint8_t cs_len[2] = { checksum, 5 };

    hal = HAL_I2C_Mem_Write(dev->hi2c, BQ76952_ADDR_WRITE,
                             BQ_SUBCMD_CHECKSUM_ADDR, I2C_MEMADD_SIZE_8BIT,
                             cs_len, 2, BQ76952_I2C_TIMEOUT);
    if (hal != HAL_OK) return BQ_ERR_I2C;

    HAL_Delay(10);
    return BQ_OK;
}


/* ============================================================
 * PROTECTION CONTROL
 * ============================================================ */

BQ76952_Status_t BQ76952_DisableAllProtections(BQ76952_Data_t *dev) {
    BQ76952_Status_t ret = BQ76952_EnterConfigMode(dev);
    if (ret != BQ_OK) return ret;
    _write1(dev, BQ_MEM_ENABLED_PROT_A, 0x00, NULL);
    _write1(dev, BQ_MEM_ENABLED_PROT_B, 0x00, NULL);
    _write1(dev, BQ_MEM_ENABLED_PROT_C, 0x00, NULL);

	_write1(dev, BQ_MEM_CHG_FET_PROT_A, 0x00, NULL);
	_write1(dev, BQ_MEM_CHG_FET_PROT_B, 0x00, NULL);
	_write1(dev, BQ_MEM_CHG_FET_PROT_C, 0x00, NULL);

	_write1(dev, BQ_MEM_DSG_FET_PROT_A, 0x00, NULL);
	_write1(dev, BQ_MEM_DSG_FET_PROT_B, 0x00, NULL);
	_write1(dev, BQ_MEM_DSG_FET_PROT_C, 0x00, NULL);
    return BQ76952_ExitConfigMode(dev);
}

BQ76952_Status_t BQ76952_DisableTempProtections(BQ76952_Data_t *dev) {
    BQ76952_Status_t ret = BQ76952_EnterConfigMode(dev);
    if (ret != BQ_OK) return ret;
    uint8_t val = _read1(dev, BQ_MEM_ENABLED_PROT_B);
    val &= ~BQ_PROTB_ALL_TEMP;
    _write1(dev, BQ_MEM_ENABLED_PROT_B, val, NULL);
    return BQ76952_ExitConfigMode(dev);
}

BQ76952_Status_t BQ76952_EnableTempProtections(BQ76952_Data_t *dev) {
    BQ76952_Status_t ret = BQ76952_EnterConfigMode(dev);
    if (ret != BQ_OK) return ret;
    _write1(dev, BQ_MEM_ENABLED_PROT_B, BQ_VAL_ENABLED_PROT_B, NULL);
    return BQ76952_ExitConfigMode(dev);
}


/* ============================================================
 * CONFIG ALL
 * ============================================================ */

BQ76952_Status_t BQ76952_ConfigAll(BQ76952_Data_t *dev, BQ76952_Debug_t *dbg) {
    BQ76952_Status_t ret;

    ret = BQ76952_EnterConfigMode(dev);
    dbg->step_enter_cfg = (uint8_t)ret;
    if (ret != BQ_OK) return BQ_ERR_CFG_MODE;

    /* 1. Settings:Configuration */
    ret = _write2(dev, BQ_MEM_POWER_CONFIG, BQ_VAL_POWER_CONFIG, &dbg->step_power_config);
    if (ret != BQ_OK) goto bail;
    ret = _write1(dev, BQ_MEM_REG12_CONFIG, BQ_VAL_REG12_CONFIG, &dbg->step_reg12);
    if (ret != BQ_OK) goto bail;
    ret = _write1(dev, BQ_MEM_DA_CONFIG, BQ_VAL_DA_CONFIG, &dbg->step_da_config);
    if (ret != BQ_OK) goto bail;
    ret = _write2(dev, BQ_MEM_VCELL_MODE, BQ_VAL_VCELL_MODE, &dbg->step_vcell_mode);
    if (ret != BQ_OK) goto bail;
    ret = _write1(dev, BQ_MEM_CFETOFF_PIN_CFG, BQ_VAL_CFETOFF_PIN_CFG, &dbg->step_cfetoff);
    if (ret != BQ_OK) goto bail;
    ret = _write1(dev, BQ_MEM_DFETOFF_PIN_CFG, BQ_VAL_DFETOFF_PIN_CFG, &dbg->step_dfetoff);
    if (ret != BQ_OK) goto bail;

    /* 2. Settings:Protection */
    ret = _write1(dev, BQ_MEM_ENABLED_PROT_A, BQ_VAL_ENABLED_PROT_A, &dbg->step_enabled_prot);
    if (ret != BQ_OK) goto bail;
    ret = _write1(dev, BQ_MEM_ENABLED_PROT_B, BQ_VAL_ENABLED_PROT_B, NULL);
    if (ret != BQ_OK) goto bail;
    ret = _write1(dev, BQ_MEM_ENABLED_PROT_C, BQ_VAL_ENABLED_PROT_C, NULL);
    if (ret != BQ_OK) goto bail;
    ret = _write1(dev, BQ_MEM_CHG_FET_PROT_A, BQ_VAL_CHG_FET_PROT_A, &dbg->step_chg_fet_prot);
    if (ret != BQ_OK) goto bail;
    ret = _write1(dev, BQ_MEM_CHG_FET_PROT_B, BQ_VAL_CHG_FET_PROT_B, NULL);
    if (ret != BQ_OK) goto bail;
    ret = _write1(dev, BQ_MEM_CHG_FET_PROT_C, BQ_VAL_CHG_FET_PROT_C, NULL);
    if (ret != BQ_OK) goto bail;
    ret = _write1(dev, BQ_MEM_DSG_FET_PROT_A, BQ_VAL_DSG_FET_PROT_A, &dbg->step_dsg_fet_prot);
    if (ret != BQ_OK) goto bail;
    ret = _write1(dev, BQ_MEM_DSG_FET_PROT_B, BQ_VAL_DSG_FET_PROT_B, NULL);
    if (ret != BQ_OK) goto bail;
    ret = _write1(dev, BQ_MEM_DSG_FET_PROT_C, BQ_VAL_DSG_FET_PROT_C, NULL);
    if (ret != BQ_OK) goto bail;

    /* 3. Settings:FET */
    ret = _write1(dev, BQ_MEM_FET_OPTIONS, BQ_VAL_FET_OPTIONS, &dbg->step_fet_options);
    if (ret != BQ_OK) goto bail;
    ret = _write1(dev, BQ_MEM_PREDCHG_TIMEOUT, BQ_VAL_PREDCHG_TIMEOUT, &dbg->step_predchg);
    if (ret != BQ_OK) goto bail;
    ret = _write1(dev, BQ_MEM_PREDCHG_STOP_DELTA, BQ_VAL_PREDCHG_STOP_DELTA, NULL);
    if (ret != BQ_OK) goto bail;

    /* 4. Current Thresholds */
    ret = _write2(dev, BQ_MEM_DSG_CURR_THRESH, BQ_VAL_DSG_CURR_THRESH, &dbg->step_curr_thresh);
    if (ret != BQ_OK) goto bail;
    ret = _write2(dev, BQ_MEM_CHG_CURR_THRESH, BQ_VAL_CHG_CURR_THRESH, NULL);
    if (ret != BQ_OK) goto bail;

    /* 5. Protection thresholds */
    ret = _write1(dev, BQ_MEM_CUV_THRESHOLD, (uint8_t)BQ_VAL_CUV_THRESHOLD, &dbg->step_cuv);
    if (ret != BQ_OK) goto bail;
    ret = _write2(dev, BQ_MEM_CUV_DELAY, BQ_VAL_CUV_DELAY, NULL);
    if (ret != BQ_OK) goto bail;

    ret = _write1(dev, BQ_MEM_COV_THRESHOLD, (uint8_t)BQ_VAL_COV_THRESHOLD, &dbg->step_cov);
    if (ret != BQ_OK) goto bail;
    ret = _write2(dev, BQ_MEM_COV_DELAY, BQ_VAL_COV_DELAY, NULL);
    if (ret != BQ_OK) goto bail;

    ret = _write1(dev, BQ_MEM_OCC_THRESHOLD, (uint8_t)BQ_VAL_OCC_THRESHOLD, &dbg->step_occ);
    if (ret != BQ_OK) goto bail;
    ret = _write1(dev, BQ_MEM_OCC_DELAY, (uint8_t)BQ_VAL_OCC_DELAY, NULL);
    if (ret != BQ_OK) goto bail;

    ret = _write1(dev, BQ_MEM_OCD1_THRESHOLD, (uint8_t)BQ_VAL_OCD1_THRESHOLD, &dbg->step_ocd1);
    if (ret != BQ_OK) goto bail;
    ret = _write1(dev, BQ_MEM_OCD1_DELAY, (uint8_t)BQ_VAL_OCD1_DELAY, NULL);
    if (ret != BQ_OK) goto bail;

    ret = _write1(dev, BQ_MEM_SCD_THRESHOLD, (uint8_t)BQ_VAL_SCD_THRESHOLD, &dbg->step_scd);
    if (ret != BQ_OK) goto bail;
    ret = _write1(dev, BQ_MEM_SCD_DELAY, (uint8_t)BQ_VAL_SCD_DELAY, NULL);
    if (ret != BQ_OK) goto bail;
    ret = _write1(dev, BQ_MEM_SCD_RECOV_TIME, (uint8_t)BQ_VAL_SCD_RECOV_TIME, NULL);
    if (ret != BQ_OK) goto bail;

    ret = _write1(dev, BQ_MEM_OTC_THRESHOLD, (uint8_t)(int8_t)BQ_VAL_OTC_THRESHOLD, &dbg->step_otc);
    if (ret != BQ_OK) goto bail;
    ret = _write1(dev, BQ_MEM_OTC_DELAY, (uint8_t)BQ_VAL_OTC_DELAY, NULL);
    if (ret != BQ_OK) goto bail;
    ret = _write1(dev, BQ_MEM_OTC_RECOVERY, (uint8_t)(int8_t)BQ_VAL_OTC_RECOVERY, NULL);
    if (ret != BQ_OK) goto bail;

    ret = _write1(dev, BQ_MEM_OTD_THRESHOLD, (uint8_t)(int8_t)BQ_VAL_OTD_THRESHOLD, &dbg->step_otd);
    if (ret != BQ_OK) goto bail;
    ret = _write1(dev, BQ_MEM_OTD_DELAY, (uint8_t)BQ_VAL_OTD_DELAY, NULL);
    if (ret != BQ_OK) goto bail;
    ret = _write1(dev, BQ_MEM_OTD_RECOVERY, (uint8_t)(int8_t)BQ_VAL_OTD_RECOVERY, NULL);
    if (ret != BQ_OK) goto bail;

    ret = _write1(dev, BQ_MEM_UTC_THRESHOLD, (uint8_t)(int8_t)BQ_VAL_UTC_THRESHOLD, &dbg->step_utc);
    if (ret != BQ_OK) goto bail;
    ret = _write1(dev, BQ_MEM_UTC_DELAY, (uint8_t)BQ_VAL_UTC_DELAY, NULL);
    if (ret != BQ_OK) goto bail;
    ret = _write1(dev, BQ_MEM_UTC_RECOVERY, (uint8_t)(int8_t)BQ_VAL_UTC_RECOVERY, NULL);
    if (ret != BQ_OK) goto bail;

    ret = _write1(dev, BQ_MEM_UTD_THRESHOLD, (uint8_t)(int8_t)BQ_VAL_UTD_THRESHOLD, &dbg->step_utd);
    if (ret != BQ_OK) goto bail;
    ret = _write1(dev, BQ_MEM_UTD_DELAY, (uint8_t)BQ_VAL_UTD_DELAY, NULL);
    if (ret != BQ_OK) goto bail;
    ret = _write1(dev, BQ_MEM_UTD_RECOVERY, (uint8_t)(int8_t)BQ_VAL_UTD_RECOVERY, NULL);
    if (ret != BQ_OK) goto bail;

    /* 6. Recovery Time */
    ret = _write1(dev, BQ_MEM_RECOVERY_TIME, (uint8_t)BQ_VAL_RECOVERY_TIME, &dbg->step_recovery);
    if (ret != BQ_OK) goto bail;

    /* 7. Cell Balancing */
    ret = _write1(dev, BQ_MEM_CB_CONFIG, BQ_VAL_CB_CONFIG, &dbg->step_cb_config);
    if (ret != BQ_OK) goto bail;

    /* Exit */
    ret = BQ76952_ExitConfigMode(dev);
    dbg->step_exit_cfg = (uint8_t)ret;
    if (ret != BQ_OK) return ret;

    HAL_Delay(100);
    dbg->cfg_ok = 1;
    return BQ_OK;

bail:
    BQ76952_ExitConfigMode(dev);
    dbg->cfg_ok = 0;
    return ret;
}

BQ76952_Status_t BQ76952_Init(I2C_HandleTypeDef *hi2c, BQ76952_Data_t *dev,
                               BQ76952_Debug_t *dbg) {
    dev->hi2c = hi2c;
    memset(dbg, 0, sizeof(*dbg));
    HAL_Delay(500);

    BQ76952_Status_t ret = BQ76952_ConfigAll(dev, dbg);
    if (ret != BQ_OK) return ret;

    /* EnableFET toggle FET_EN — chỉ gọi 1 lần duy nhất ở đây.
     * KHÔNG gọi lại trong main, vì toggle lần 2 sẽ tắt FET. */
    ret = BQ76952_EnableFET(dev);
    dbg->step_fet_enable = (uint8_t)ret;
    if (ret != BQ_OK) return ret;

    dev->mfg_status = 0xFF;
    BQ76952_ReadDataMemory(dev, 0x0057, &dev->mfg_status, 1);

    dev->chg_pump  = 0xFF;
	BQ76952_ReadDataMemory(dev, 0x9309, &dev->chg_pump , 1);

    HAL_Delay(20);
    BQ76952_VerifyConfig(dev, dbg);
    return BQ_OK;
}


/* ============================================================
 * READ REALTIME
 * ============================================================ */

BQ76952_Status_t BQ76952_ReadCellVoltages(BQ76952_Data_t *dev) {
    uint8_t buf[2];
    static const uint8_t cell_regs[16] = {
        0x14, 0x16, 0x18, 0x1A, 0x1C, 0x1E, 0x20, 0x22,
        0x24, 0x26, 0x28, 0x2A, 0x2C, 0x2E, 0x30, 0x32
    };
    for (uint8_t i = 0; i < 16; i++) {
        BQ76952_Status_t ret = BQ76952_ReadRegister(dev, cell_regs[i], buf, 2);
        if (ret != BQ_OK) return ret;
        dev->cell_mV[i] = (int16_t)(buf[0] | (buf[1] << 8));
    }
    BQ76952_ReadRegister(dev, BQ_CMD_STACK_VOLTAGE, buf, 2);
    dev->stack_voltage = (uint16_t)(buf[0] | (buf[1] << 8));
    BQ76952_ReadRegister(dev, BQ_CMD_PACK_VOLTAGE, buf, 2);
    dev->pack_voltage = (uint16_t)(buf[0] | (buf[1] << 8));
    return BQ_OK;
}

BQ76952_Status_t BQ76952_ReadCurrent(BQ76952_Data_t *dev) {
    uint8_t buf[2];
    BQ76952_Status_t ret = BQ76952_ReadRegister(dev, BQ_CMD_CC2_CURRENT, buf, 2);
    if (ret != BQ_OK) return ret;
    dev->current = (int16_t)(buf[0] | (buf[1] << 8));
    return BQ_OK;
}

BQ76952_Status_t BQ76952_ReadTemperatures(BQ76952_Data_t *dev) {
    uint8_t buf[2];
    BQ76952_ReadRegister(dev, BQ_CMD_TEMP_INTERNAL, buf, 2);
    dev->temp_internal_dC = (int16_t)((buf[0] | (buf[1] << 8)) - 2731);
    BQ76952_ReadRegister(dev, BQ_CMD_TEMP_TS1, buf, 2);
    dev->temp_ts1_dC = (int16_t)((buf[0] | (buf[1] << 8)) - 2731);
    BQ76952_ReadRegister(dev, BQ_CMD_TEMP_TS2, buf, 2);
    dev->temp_ts2_dC = (int16_t)((buf[0] | (buf[1] << 8)) - 2731);
    BQ76952_ReadRegister(dev, BQ_CMD_TEMP_TS3, buf, 2);
    dev->temp_ts3_dC = (int16_t)((buf[0] | (buf[1] << 8)) - 2731);
    return BQ_OK;
}

BQ76952_Status_t BQ76952_ReadStatus(BQ76952_Data_t *dev) {
    uint8_t buf[2];
    BQ76952_ReadRegister(dev, BQ_CMD_FET_STATUS, buf, 1);
    dev->fet_status = buf[0];
    BQ76952_ReadRegister(dev, BQ_CMD_BATTERY_STATUS, buf, 2);
    dev->battery_status = (uint16_t)(buf[0] | (buf[1] << 8));
    BQ76952_ReadRegister(dev, BQ_CMD_ALARM_STATUS, buf, 2);
    dev->alarm_status = (uint16_t)(buf[0] | (buf[1] << 8));
    BQ76952_ReadRegister(dev, BQ_CMD_ALARM_RAW_STATUS, buf, 2);
    dev->alarm_raw = (uint16_t)(buf[0] | (buf[1] << 8));
    BQ76952_ReadRegister(dev, BQ_CMD_SAFETY_ALERT_A, buf, 1);  dev->safety_alert_a = buf[0];
    BQ76952_ReadRegister(dev, BQ_CMD_SAFETY_ALERT_B, buf, 1);  dev->safety_alert_b = buf[0];
    BQ76952_ReadRegister(dev, BQ_CMD_SAFETY_ALERT_C, buf, 1);  dev->safety_alert_c = buf[0];
    BQ76952_ReadRegister(dev, BQ_CMD_SAFETY_STATUS_A, buf, 1); dev->safety_status_a = buf[0];
    BQ76952_ReadRegister(dev, BQ_CMD_SAFETY_STATUS_B, buf, 1); dev->safety_status_b = buf[0];
    BQ76952_ReadRegister(dev, BQ_CMD_SAFETY_STATUS_C, buf, 1); dev->safety_status_c = buf[0];
    return BQ_OK;
}

BQ76952_Status_t BQ76952_ReadAllData(BQ76952_Data_t *dev) {
    BQ76952_Status_t ret;
    ret = BQ76952_ReadCellVoltages(dev); if (ret != BQ_OK) return ret;
    ret = BQ76952_ReadCurrent(dev);      if (ret != BQ_OK) return ret;
    ret = BQ76952_ReadTemperatures(dev); if (ret != BQ_OK) return ret;
    ret = BQ76952_ReadStatus(dev);       if (ret != BQ_OK) return ret;
    return BQ_OK;
}


/* ============================================================
 * DIAGNOSTIC
 * ============================================================ */

void BQ76952_Diagnose(BQ76952_Data_t *dev, BQ76952_DiagMode_t mode,
                      GPIO_TypeDef *scl_port, uint16_t scl_pin,
                      BQ76952_DiagResult_t *result) {
    memset(result, 0, sizeof(*result));
    result->sr1 = dev->hi2c->Instance->SR1;
    result->sr2 = dev->hi2c->Instance->SR2;
    switch (mode) {
        case BQ_DIAG_PING:
            result->i2c_status = HAL_I2C_IsDeviceReady(dev->hi2c, BQ76952_ADDR_WRITE, 5, 100);
            break;
        case BQ_DIAG_PING_ALT:
            result->i2c_status = HAL_I2C_IsDeviceReady(dev->hi2c, 0x08, 5, 100);
            break;
        case BQ_DIAG_BUS_RESET: {
            GPIO_InitTypeDef g = {0};
            g.Pin = scl_pin; g.Mode = GPIO_MODE_OUTPUT_OD;
            g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_LOW;
            HAL_GPIO_Init(scl_port, &g);
            for (uint8_t i = 0; i < 9; i++) {
                HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_SET);   HAL_Delay(1);
                HAL_GPIO_WritePin(scl_port, scl_pin, GPIO_PIN_RESET); HAL_Delay(1);
            }
            HAL_I2C_DeInit(dev->hi2c); HAL_I2C_Init(dev->hi2c);
            result->i2c_status = HAL_I2C_IsDeviceReady(dev->hi2c, BQ76952_ADDR_WRITE, 5, 100);
            break;
        }
        case BQ_DIAG_SLOW_CLK:
            dev->hi2c->Init.ClockSpeed = 50000;
            HAL_I2C_DeInit(dev->hi2c); HAL_I2C_Init(dev->hi2c);
            result->i2c_status = HAL_I2C_IsDeviceReady(dev->hi2c, BQ76952_ADDR_WRITE, 5, 200);
            break;
        case BQ_DIAG_I2C_STATE:
            result->i2c_status = (HAL_StatusTypeDef)HAL_I2C_GetState(dev->hi2c);
            result->i2c_error = HAL_I2C_GetError(dev->hi2c);
            return;
        case BQ_DIAG_SCAN_ALL:
            result->i2c_status = HAL_ERROR;
            for (uint16_t addr = 0; addr <= 0xFF; addr++) {
                HAL_I2C_DeInit(dev->hi2c); HAL_I2C_Init(dev->hi2c);
                if (HAL_I2C_IsDeviceReady(dev->hi2c, addr, 3, 100) == HAL_OK) {
                    if (result->found_count < 25)
                        result->found_addrs[result->found_count++] = (uint8_t)addr;
                    result->i2c_status = HAL_OK;
                }
            }
            break;
        default:
            result->i2c_status = HAL_ERROR;
            break;
    }
    result->i2c_error = HAL_I2C_GetError(dev->hi2c);
}


/* ============================================================
 * UTILITY
 * ============================================================ */

float BQ76952_GetStackVoltage_V(BQ76952_Data_t *dev) {
    return dev->stack_voltage * 0.01f;
}

float BQ76952_GetCurrent_A(BQ76952_Data_t *dev) {
    return dev->current * 0.01f;
}

float BQ76952_GetTemp_C(int16_t temp_dC) {
    return temp_dC * 0.1f;
}

bool BQ76952_HasFault(BQ76952_Data_t *dev) {
    return (dev->safety_alert_a | dev->safety_alert_b | dev->safety_alert_c) != 0;
}

bool BQ76952_IsCHG_FET_ON(BQ76952_Data_t *dev) {
    return (dev->fet_status & 0x01) != 0;
}

bool BQ76952_IsDSG_FET_ON(BQ76952_Data_t *dev) {
    return (dev->fet_status & 0x04) != 0;
}


/* ============================================================
 * DEBUG — Đọc lại toàn bộ mem đã ghi để verify trên debugger
 * ============================================================ */

void BQ76952_VerifyConfig(BQ76952_Data_t *dev, BQ76952_Debug_t *dbg) {
    /* Settings:Configuration */
    dbg->cfg_power_config   = _read2(dev, BQ_MEM_POWER_CONFIG);
    dbg->cfg_reg12          = _read1(dev, BQ_MEM_REG12_CONFIG);
    dbg->cfg_cfetoff        = _read1(dev, BQ_MEM_CFETOFF_PIN_CFG);
    dbg->cfg_dfetoff        = _read1(dev, BQ_MEM_DFETOFF_PIN_CFG);
    dbg->cfg_da_config      = _read1(dev, BQ_MEM_DA_CONFIG);
    dbg->cfg_vcell_mode     = _read2(dev, BQ_MEM_VCELL_MODE);

    /* Settings:Protection */
    dbg->cfg_enabled_prot_a = _read1(dev, BQ_MEM_ENABLED_PROT_A);
    dbg->cfg_enabled_prot_b = _read1(dev, BQ_MEM_ENABLED_PROT_B);
    dbg->cfg_enabled_prot_c = _read1(dev, BQ_MEM_ENABLED_PROT_C);
    dbg->cfg_chg_fet_prot_a = _read1(dev, BQ_MEM_CHG_FET_PROT_A);
    dbg->cfg_chg_fet_prot_b = _read1(dev, BQ_MEM_CHG_FET_PROT_B);
    dbg->cfg_chg_fet_prot_c = _read1(dev, BQ_MEM_CHG_FET_PROT_C);
    dbg->cfg_dsg_fet_prot_a = _read1(dev, BQ_MEM_DSG_FET_PROT_A);
    dbg->cfg_dsg_fet_prot_b = _read1(dev, BQ_MEM_DSG_FET_PROT_B);
    dbg->cfg_dsg_fet_prot_c = _read1(dev, BQ_MEM_DSG_FET_PROT_C);

    /* Settings:FET */
    dbg->cfg_fet_options    = _read1(dev, BQ_MEM_FET_OPTIONS);
    dbg->cfg_predchg_timeout = _read1(dev, BQ_MEM_PREDCHG_TIMEOUT);
    dbg->cfg_predchg_delta  = _read1(dev, BQ_MEM_PREDCHG_STOP_DELTA);

    /* Settings:Current Thresholds */
    dbg->cfg_dsg_curr_thresh = _read2(dev, BQ_MEM_DSG_CURR_THRESH);
    dbg->cfg_chg_curr_thresh = _read2(dev, BQ_MEM_CHG_CURR_THRESH);

    /* Settings:Cell Balancing */
    dbg->cfg_cb_config      = _read1(dev, BQ_MEM_CB_CONFIG);

    /* Protections thresholds */
    dbg->cfg_cuv_threshold  = _read1(dev, BQ_MEM_CUV_THRESHOLD);
    dbg->cfg_cuv_delay      = _read2(dev, BQ_MEM_CUV_DELAY);
    dbg->cfg_cov_threshold  = _read1(dev, BQ_MEM_COV_THRESHOLD);
    dbg->cfg_cov_delay      = _read2(dev, BQ_MEM_COV_DELAY);
    dbg->cfg_occ_threshold  = _read1(dev, BQ_MEM_OCC_THRESHOLD);
    dbg->cfg_occ_delay      = _read1(dev, BQ_MEM_OCC_DELAY);
    dbg->cfg_ocd1_threshold = _read1(dev, BQ_MEM_OCD1_THRESHOLD);
    dbg->cfg_ocd1_delay     = _read1(dev, BQ_MEM_OCD1_DELAY);
    dbg->cfg_scd_threshold  = _read1(dev, BQ_MEM_SCD_THRESHOLD);
    dbg->cfg_scd_delay      = _read1(dev, BQ_MEM_SCD_DELAY);
    dbg->cfg_scd_recov_time = _read1(dev, BQ_MEM_SCD_RECOV_TIME);
    dbg->cfg_otc_threshold  = _read1(dev, BQ_MEM_OTC_THRESHOLD);
    dbg->cfg_otc_delay      = _read1(dev, BQ_MEM_OTC_DELAY);
    dbg->cfg_otc_recovery   = _read1(dev, BQ_MEM_OTC_RECOVERY);
    dbg->cfg_otd_threshold  = _read1(dev, BQ_MEM_OTD_THRESHOLD);
    dbg->cfg_otd_delay      = _read1(dev, BQ_MEM_OTD_DELAY);
    dbg->cfg_otd_recovery   = _read1(dev, BQ_MEM_OTD_RECOVERY);
    dbg->cfg_utc_threshold  = _read1(dev, BQ_MEM_UTC_THRESHOLD);
    dbg->cfg_utc_delay      = _read1(dev, BQ_MEM_UTC_DELAY);
    dbg->cfg_utc_recovery   = _read1(dev, BQ_MEM_UTC_RECOVERY);
    dbg->cfg_utd_threshold  = _read1(dev, BQ_MEM_UTD_THRESHOLD);
    dbg->cfg_utd_delay      = _read1(dev, BQ_MEM_UTD_DELAY);
    dbg->cfg_utd_recovery   = _read1(dev, BQ_MEM_UTD_RECOVERY);
    dbg->cfg_recovery_time  = _read1(dev, BQ_MEM_RECOVERY_TIME);
}

void BQ76952_UpdateDebug(BQ76952_Data_t *dev, BQ76952_Debug_t *dbg) {
    uint8_t buf[2];
    dbg->read_count++;

    /* ── Raw cell bytes ──────────────────────────────────── */
    static const uint8_t cell_regs[16] = {
        0x14, 0x16, 0x18, 0x1A, 0x1C, 0x1E, 0x20, 0x22,
        0x24, 0x26, 0x28, 0x2A, 0x2C, 0x2E, 0x30, 0x32
    };
    for (uint8_t i = 0; i < 16; i++) {
        if (BQ76952_ReadRegister(dev, cell_regs[i], buf, 2) == BQ_OK) {
            dbg->raw_cell[i][0] = buf[0];
            dbg->raw_cell[i][1] = buf[1];
        } else {
            dbg->raw_cell[i][0] = 0xEE;
            dbg->raw_cell[i][1] = 0xEE;
            dbg->i2c_err_count++;
        }
    }

    /* ── Live status — đọc trực tiếp từ IC ───────────────── */
    if (BQ76952_ReadRegister(dev, BQ_CMD_FET_STATUS, buf, 1) == BQ_OK)
        dbg->fet_status = buf[0];
    if (BQ76952_ReadRegister(dev, BQ_CMD_BATTERY_STATUS, buf, 2) == BQ_OK)
        dbg->battery_status = (uint16_t)(buf[0] | (buf[1] << 8));
    if (BQ76952_ReadRegister(dev, BQ_CMD_ALARM_STATUS, buf, 2) == BQ_OK)
        dbg->alarm_status = (uint16_t)(buf[0] | (buf[1] << 8));
    if (BQ76952_ReadRegister(dev, BQ_CMD_ALARM_RAW_STATUS, buf, 2) == BQ_OK)
        dbg->alarm_raw = (uint16_t)(buf[0] | (buf[1] << 8));

    // Manufacturing Status (subcommand 0x0057)
    dbg->mfg_status = _read1(dev, 0x0057);

    // Charge Pump Control (0x9309)
    dbg->chg_pump = _read1(dev, 0x9309);

    // Safety alerts & status
    if (BQ76952_ReadRegister(dev, BQ_CMD_SAFETY_ALERT_A, buf, 1) == BQ_OK)
        dbg->safety_alert_a = buf[0];
    if (BQ76952_ReadRegister(dev, BQ_CMD_SAFETY_ALERT_B, buf, 1) == BQ_OK)
        dbg->safety_alert_b = buf[0];
    if (BQ76952_ReadRegister(dev, BQ_CMD_SAFETY_ALERT_C, buf, 1) == BQ_OK)
        dbg->safety_alert_c = buf[0];
    if (BQ76952_ReadRegister(dev, BQ_CMD_SAFETY_STATUS_A, buf, 1) == BQ_OK)
        dbg->safety_status_a = buf[0];
    if (BQ76952_ReadRegister(dev, BQ_CMD_SAFETY_STATUS_B, buf, 1) == BQ_OK)
        dbg->safety_status_b = buf[0];
    if (BQ76952_ReadRegister(dev, BQ_CMD_SAFETY_STATUS_C, buf, 1) == BQ_OK)
        dbg->safety_status_c = buf[0];

    /* ── Decoded flags ───────────────────────────────────── */
    dbg->chg_fet_on       = (dbg->fet_status & (1u << 0)) ? 1 : 0;
    dbg->dsg_fet_on       = (dbg->fet_status & (1u << 2)) ? 1 : 0;
    dbg->pdsg_fet_on      = (dbg->fet_status & (1u << 3)) ? 1 : 0;
    dbg->dchg_pin         = (dbg->fet_status & (1u << 4)) ? 1 : 0;
    dbg->ddsg_pin         = (dbg->fet_status & (1u << 5)) ? 1 : 0;
    dbg->fet_en           = (dbg->mfg_status & (1u << 4)) ? 1 : 0;
    dbg->in_cfg_mode      = (dbg->battery_status & (1u << 0)) ? 1 : 0;  // CFGUPDATE = bit0
    dbg->sleep_en         = (dbg->battery_status & (1u << 2)) ? 1 : 0;
    dbg->por              = (dbg->battery_status & (1u << 3)) ? 1 : 0;
    dbg->in_sleep_mode    = (dbg->battery_status & (1u << 15)) ? 1 : 0;
    dbg->safety_fault     = (dbg->battery_status & (1u << 11)) ? 1 : 0;
    dbg->pf_fault         = (dbg->battery_status & (1u << 12)) ? 1 : 0;
    dbg->safety_triggered = (dbg->safety_alert_a | dbg->safety_alert_b | dbg->safety_alert_c) ? 1 : 0;
    dbg->estop_active     = (!dbg->chg_fet_on && !dbg->dsg_fet_on && !dbg->safety_triggered) ? 1 : 0;
}
