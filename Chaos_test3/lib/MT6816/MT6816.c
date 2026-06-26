/*  * bldc_encoder.c
 *
 *  Created on: Jan 9, 2026
 *      Author: Ngoc
 */

#include "MT6816.h"

// Global encoder instance
Encoder_t encoder;

// Register addresses (separate from struct)
const uint8_t reg_addrs[2] = {0x83, 0x84};

// Static buffer for DMA
static uint8_t tx_buf[2] = {0x83, 0x00};

// ---------------- Inline Helper Functions ----------------
static inline bool checkEvenParity(uint16_t data) {
    // Use XOR to count bits faster
    data ^= data >> 8;
    data ^= data >> 4;
    data ^= data >> 2;
    data ^= data >> 1;
    return !(data & 1); // Even parity
}

static inline float wrap_pm_180(float deg) {
	float wraped_deg = deg - (360.0f * floorf((deg + 180.0f) / 360.0f));
	return wraped_deg;
}

// ---------------- Public Functions ----------------
void MT6816_Config(GPIO_TypeDef *CS_Port, uint16_t CS_Pin, SPI_HandleTypeDef *spi) {
	encoder.rawData = 0;
	encoder.rawAngle = 0;
	encoder.magCheck = 0;
	encoder.parityCheck = 0;

	encoder.prevMechTheta = 0.0f;
	encoder.mechTheta = 0.0f;
	encoder.vel = 0.0f;

	encoder.rx_8Bit[0] = 0;
	encoder.rx_8Bit[1] = 0;
	encoder.complete_16Bit[0] = 0;
	encoder.complete_16Bit[1] = 0;
	encoder.read_flag = 0;
	encoder.check = 0;
	encoder.initialized = 0;
	encoder.validCount = 0;
	encoder.parityErrorCount = 0;
	encoder.spikeRejectCount = 0;
	encoder.spiBusyCount = 0;
	encoder.spiErrorCount = 0;
	encoder.lastUpdateMs = 0;

	encoder.CS_Port = CS_Port;
	encoder.CS_Pin = CS_Pin;
	encoder.spi = spi;

	encoder.multiturnAngleRound = 0;
}

inline void MT6816_Select(void) {
	encoder.CS_Port->BSRR = (encoder.CS_Pin << 16); // GPIO_PIN_RESET
}

inline void MT6816_DeSelect(void) {
	encoder.CS_Port->BSRR = encoder.CS_Pin; // GPIO_PIN_SET
}

/* -------------------------------- utils -------------------------------- */

void MT6816_Start() {
	MT6816_DeSelect();
	HAL_Delay(20);
	Spi_Transmit_Receive_Data_8Bit(reg_addrs[0]);
//	encoder.check = 1;
}

void MT6816_Get_RPM(foc_t *foc, float dt_us) {
    // Optimized angle wrap-around
    float dtheta = wrap_pm_180(encoder.mechTheta - encoder.prevMechTheta);

    encoder.prevMechTheta = encoder.mechTheta;
    encoder.dtheta = dtheta;

    // Calculate instantaneous RPM (optimized constants)
    const float dt_sec = dt_us * 1e-6f;  // Convert microseconds to seconds
    const float rpm_conversion = 60.0f / 360.0f;  // Degrees/sec to RPM (1 rev = 360°, 1 min = 60 sec)
    float rpm_instant = (dtheta / dt_sec) * rpm_conversion;

    // Two-stage spike rejection
#ifdef USE_RPM_SPIKE_REJECTION
    float rpm_delta = rpm_instant - encoder.prevRPM;
    float abs_delta = fabsf(rpm_delta);

    if (abs_delta > MAX_RPM_JUMP) {
        float limited_delta = copysignf(fminf(abs_delta * 0.5f, MAX_RPM_JUMP), rpm_delta);
        rpm_instant = encoder.prevRPM + limited_delta;
    }
#endif

    // IIR low-pass filter
#ifdef USE_RPM_FILTER
    encoder.vel = encoder.vel * (1.0f - ANGLE_FILTER_ALPHA) + rpm_instant * ANGLE_FILTER_ALPHA;
#else
    encoder.vel = rpm_instant;
#endif

    // Very low RPM clamping
    if (fabsf(encoder.vel) < 0.1f) {
        encoder.vel = 0.0f;
    }

    // Update state for next iteration
#if defined(USE_RPM_SPIKE_REJECTION) || defined(USE_RPM_FILTER)
    encoder.prevRPM = rpm_instant;
#endif

    // update foc
    foc->encd_rpm = encoder.vel;
}

/* -------------------------------- 8-bit version -------------------------------- */
void Spi_Transmit_Receive_Data_8Bit(uint8_t reg) {
    tx_buf[0] = reg;

    if (encoder.check || HAL_SPI_GetState(encoder.spi) != HAL_SPI_STATE_READY) {
    	encoder.spiBusyCount++;
    	return;
    }

    MT6816_Select();
    if (HAL_SPI_TransmitReceive_DMA(encoder.spi, tx_buf, encoder.rx_8Bit, 2) != HAL_OK) {
    	MT6816_DeSelect();
    	encoder.spiErrorCount++;
    	encoder.check = 0;
    	return;
    }

    encoder.check = 1;
//    encoder.check ++;
}

void MT6816_Update_Angle_8Bit(foc_t *foc) {
	// Combine 2 bytes
    encoder.rawData = ((uint16_t)(encoder.complete_16Bit[0]) << 8) | (encoder.complete_16Bit[1]);

    encoder.parityCheck = checkEvenParity(encoder.rawData);

	// If parity fails → keep previous filtered angle
	if (!encoder.parityCheck) {
		encoder.parityErrorCount++;
		return;
	}

	encoder.rawAngle = encoder.rawData >> 2;  // 14-bit angle
	encoder.magCheck = (bool)(encoder.rawData & 0x02);

	// Convert raw angle to mechanical degrees
	float rawTheta = encoder.rawAngle * (360.0f / MT6816_RESOLUTION);

	if (!encoder.initialized) {
		encoder.prevRawTheta = rawTheta;
		encoder.mechTheta = rawTheta;
		encoder.prevMechTheta = rawTheta;
		encoder.spikeCounter = 0;
		encoder.initialized = 1;
	}

	/* ============================ 1. Spike / jump rejection ============================ */
	float rawDiff = wrap_pm_180(rawTheta - encoder.prevRawTheta);

	if (fabsf(rawDiff) > MAX_ANGLE_JUMP_DEG) {
		if (++encoder.spikeCounter < SPIKE_REJECT_COUNT) {
			// keep the previous states - do nothing
			encoder.spikeRejectCount++;
			return;
		}
		encoder.spikeCounter = 0;
	} else {
		encoder.spikeCounter = 0;
	}

	encoder.prevRawTheta = rawTheta;

	/* ============================ 2. IIR low-pass (wrap-aware) ============================ */
	float filtDiff = wrap_pm_180(rawTheta - encoder.mechTheta);
	encoder.mechTheta += ANGLE_FILTER_ALPHA * filtDiff;

	// Normalize to [0, 360)
	if (encoder.mechTheta >= 360.0f)
		encoder.mechTheta -= 360.0f;
	else if (encoder.mechTheta < 0.0f)
		encoder.mechTheta += 360.0f;

	/* ============================ Update FOC ============================ */
	foc->m_rad_raw = DEG_TO_RAD(encoder.mechTheta);
	encoder.validCount++;
	encoder.lastUpdateMs = HAL_GetTick();
}


void MT6816_get_multiturn_degree(foc_t *foc) { // get the multiturn angle
    const float m_current_angle = encoder.mechTheta;
	float angle_dif = (m_current_angle - encoder.prevMultiturnAngle);

	if (angle_dif< -180) {
		encoder.multiturnAngleRound++;
	}
	else if (angle_dif> 180) {
		encoder.multiturnAngleRound--;
	}
	else if (encoder.multiturnAngleRound != 0 && foc->check_mul_round) {
		encoder.multiturnAngleRound = 0;
		foc->check_mul_round = 0;
	}
	float out_deg = (m_current_angle + encoder.multiturnAngleRound * 360.0 - RAD_TO_DEG(foc->m_rad_offset));
    encoder.multiturnAngle = ((1.0f - ACTUAL_ANGLE_FILTER_ALPHA) * encoder.multiturnAngle + ACTUAL_ANGLE_FILTER_ALPHA * out_deg);
	foc->mutil_turn_deg = encoder.multiturnAngle;
    encoder.prevMultiturnAngle = m_current_angle;
}
