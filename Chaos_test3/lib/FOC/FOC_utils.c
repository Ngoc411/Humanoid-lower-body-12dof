/*
 * FOC_utils.c
 *
 *  Created on: May 31, 2025
 *      Author: munir
 */

#include "FOC_utils.h"
#include "flash.h"
#include <string.h>
#include <math.h>

// extern from main.c
extern motor_config_t m_config;

_Bool foc_ready = 0;

float Vd_buff[MAX_I_SAMPLE];
float Vq_buff[MAX_I_SAMPLE];
float Id_buff[MAX_I_SAMPLE];
float Iq_buff[MAX_I_SAMPLE];

float error_temp[ERROR_LUT_SIZE] = {0};

/**
 * @brief Initialize PWM output pointers for FOC controller
 * @param foc Pointer to FOC controller structure
 * @param pwm_a Pointer to PWM channel A register (e.g., &TIM1->CCR1)
 * @param pwm_b Pointer to PWM channel B register (e.g., &TIM1->CCR2)
 * @param pwm_c Pointer to PWM channel C register (e.g., &TIM1->CCR3)
 * @param pwm_res PWM resolution
 */
void foc_pwm_init(foc_t *foc, volatile uint32_t *pwm_a, volatile uint32_t *pwm_b, volatile uint32_t *pwm_c, uint32_t pwm_res)
{
    // Validate pointers
    if(foc == NULL || pwm_a == NULL || pwm_b == NULL || pwm_c == NULL) {
        // Error handling (could be an assertion or error code)
        return;
    }

    foc->pwm_a = pwm_a;
    foc->pwm_b = pwm_b;
    foc->pwm_c = pwm_c;
    foc->pwm_res = pwm_res;

    // Optional: Initialize PWM values to 0
    *foc->pwm_a = 0;
    *foc->pwm_b = 0;
    *foc->pwm_c = 0;
}

void foc_motor_init(foc_t *foc, uint8_t pole_pairs, float kv)
{
    if (foc == NULL || pole_pairs == 0 || kv <= 0) {
        return;
    }

    foc->pole_pairs = pole_pairs;
    foc->kv = kv;
}

void foc_sensor_init(foc_t *foc, float m_rad_offset, dir_mode_t sensor_dir)
{
    if (foc == NULL)
        return;

    foc->m_rad_offset = m_rad_offset;
    foc->sensor_dir = sensor_dir;
}

void foc_gear_reducer_init(foc_t *foc, float gear_ratio)
{
    if (foc == NULL)
        return;

    foc->gear_ratio = gear_ratio;
}

void foc_set_limit_current(foc_t *foc, float max_current)
{
    if (foc == NULL)
        return;

    foc->max_current = max_current;
}

void foc_current_control_update(foc_t *foc)
{
//    if (foc == NULL || foc->control_mode == AUDIO_MODE) {
//        foc->id_ctrl.integral = 0.0f;
//        foc->id_ctrl.last_error = 0.0f;
//        foc->iq_ctrl.integral = 0.0f;
//        foc->iq_ctrl.last_error = 0.0f;
//        return;
//    }

    float id_ref = foc->id_ref;
    float iq_ref = foc->iq_ref;

    // Hard limit references
    id_ref = CONSTRAIN(id_ref, -foc->max_current, foc->max_current);
    iq_ref = CONSTRAIN(iq_ref, -foc->max_current, foc->max_current);

    // pre calculate sin & cos
    float sin_theta, cos_theta;
    pre_calc_sin_cos(foc->e_rad, &sin_theta, &cos_theta);

    // Get measured currents — ADC đo phase A và C, tính lại ib = -ia - ic (Kirchhoff)
    float ib_calc = -foc->ia - foc->ic;
    clarke_transform(foc->ia, ib_calc, &foc->i_alpha, &foc->i_beta);
    park_transform(foc->i_alpha, foc->i_beta, sin_theta, cos_theta, &foc->id, &foc->iq);

    // LPF id & iq
    const float alpha_i_filt = 0.5f;
    foc->id_filtered = (1.0f - alpha_i_filt) * foc->id_filtered + alpha_i_filt * foc->id;
    foc->iq_filtered = (1.0f - alpha_i_filt) * foc->iq_filtered + alpha_i_filt * foc->iq;

    // set dynamic max output vd and vq
    foc->id_ctrl.out_max = foc->id_ctrl.out_max_dynamic * foc->v_bus;
    foc->iq_ctrl.out_max = foc->iq_ctrl.out_max_dynamic * foc->v_bus;

    // Continue normal FOC
    float vd_ref = pi_control(&foc->id_ctrl, id_ref - foc->id);
    float vq_ref = pi_control(&foc->iq_ctrl, iq_ref - foc->iq);

    uint32_t da, db, dc;
    inverse_park_transform(vd_ref, vq_ref, sin_theta, cos_theta, &foc->v_alpha, &foc->v_beta);
    svpwm(foc->v_alpha, foc->v_beta, foc->v_bus, foc->pwm_res, &da, &db, &dc);

    // pwm limit
    *(foc->pwm_a) = CONSTRAIN(da, 0, foc->pwm_res);
    *(foc->pwm_b) = CONSTRAIN(db, 0, foc->pwm_res);
    *(foc->pwm_c) = CONSTRAIN(dc, 0, foc->pwm_res);
}

void foc_speed_control_update(foc_t *foc, float rpm_reference)
{
    if (foc == NULL || (foc->control_mode != SPEED_CONTROL_MODE && foc->control_mode != POSITION_CONTROL_MODE)) {
        foc->speed_ctrl.integral = 0.0f;
        foc->speed_ctrl.last_error = 0.0f;
        return;
    }

    foc->id_ref = 0.0f;
    foc->iq_ref = pid_control(&foc->speed_ctrl, rpm_reference - foc->actual_rpm);
}

void foc_position_control_update(foc_t *foc, float deg_reference)
{
    if (foc == NULL || foc->control_mode != POSITION_CONTROL_MODE) {
        foc->pos_ctrl.integral = 0.0f;
        foc->pos_ctrl.last_error = 0.0f;
        return;
    }

    foc->id_ref = 0.0f;
    foc->iq_ref = pid_control(&foc->pos_ctrl, deg_reference - *(foc->actual_angle_general));
}

void foc_calc_electric_angle(foc_t *foc) // calculate electric angle
{
    // Check for NULL pointer and invalid parameters
    if (foc == NULL || foc->pole_pairs <= 0) {
        return;
    }

    // Normalize mechanical angle
    foc->m_rad = foc->m_rad_raw - foc->m_rad_offset;
    norm_angle_rad(&foc->m_rad);

    // Calculate raw electric angle
    float e_rad = foc->m_rad * foc->pole_pairs;

    // Handle sensor direction
    if (foc->sensor_dir == REVERSE_DIR) {
        e_rad = TWO_PI - e_rad;
    }

    foc->e_rad = e_rad;

    // Calculate LUT index with wrap-around
    float lut_idx_f = (foc->m_rad / TWO_PI) * ERROR_LUT_SIZE;
    lut_idx_f = fmodf(lut_idx_f, ERROR_LUT_SIZE); // get the float remainder of lut_idx_f/ERROR_LUT_SIZE
    if (lut_idx_f < 0) {
        lut_idx_f += ERROR_LUT_SIZE;
    }

    // Get neighboring indices with wrap-around
    int idx0 = (int)lut_idx_f % ERROR_LUT_SIZE;
    int idx1 = (idx0 + 1) % ERROR_LUT_SIZE;
    float frac = lut_idx_f - (float)idx0;

    // Linear interpolation
    float encoder_error = m_config.encd_error_comp[idx0] * (1.0f - frac) + m_config.encd_error_comp[idx1] * frac;

    e_rad += encoder_error;

    // Normalize final electric angle
    norm_angle_rad(&e_rad);
    foc->e_rad_comp = e_rad;
}

inline void foc_calc_mech_rpm_encoder(foc_t *foc) // calculate mechanical rpm
{
    if (foc->sensor_dir == REVERSE_DIR) {
        foc->actual_rpm = -foc->encd_rpm / foc->gear_ratio;
    } else {
        foc->actual_rpm = foc->encd_rpm / foc->gear_ratio;
    }
}

inline void foc_calc_mech_pos_encoder(foc_t *foc) // calculate mechanical position
{
    if (foc->sensor_dir == REVERSE_DIR) {
        foc->actual_angle = (-foc->mutil_turn_deg / foc->gear_ratio) - foc->actual_angle_offset;
    } else {
        foc->actual_angle = (foc->mutil_turn_deg / foc->gear_ratio) - foc->actual_angle_offset;
    }
}

void foc_cal_encoder_misalignment(foc_t *foc) // calibration encoder mechanical misalignment
{
    open_loop_voltage_control(foc, VD_CAL, VQ_CAL, 0.0f);
    HAL_Delay(1000);

    float rad_offset = 0.0f;
    for (int i = 0; i < CAL_ITERATION; i++) {
        rad_offset += DEG_TO_RAD(*foc->mech_theta);
        HAL_Delay(1);
    }

    open_loop_voltage_control(foc, 0.0f, 0.0f, 0.0f);

    rad_offset = rad_offset / (float)CAL_ITERATION;
    foc->m_rad_offset = rad_offset;
    m_config.encd_offset = rad_offset;
}

void foc_cal_encoder(foc_t *foc) // calibration encoder electrical
{
    memset(error_temp, 0, sizeof(error_temp));
    foc_cal_encoder_misalignment(foc);

    for (int i = 0; i < ERROR_LUT_SIZE; i++) {
    	if (i == 100) foc->check_mul_round = 0;
        float mech_deg = (float)i * (360.0f / (float)ERROR_LUT_SIZE);
        float elec_rad = DEG_TO_RAD(mech_deg * foc->pole_pairs);

        open_loop_voltage_control(foc, VD_CAL, VQ_CAL, elec_rad);
        HAL_Delay(5);

        float mech_rad = foc->m_rad;
        float raw_delta = elec_rad - foc->e_rad;
        float delta = elec_rad - foc->e_rad_comp;

        raw_delta -= TWO_PI * floorf((raw_delta + PI) / TWO_PI); // convert to scope of -pi and pi
        delta -= TWO_PI * floorf((delta + PI) / TWO_PI); // convert to scope of -pi and pi

        float lut_pos = (mech_rad / TWO_PI) * ERROR_LUT_SIZE;
        int index = (int)(lut_pos); // scope 0 to 1023 (never reach to 1024)

        while (index < 0) {
            index += ERROR_LUT_SIZE;
        }
        index %= ERROR_LUT_SIZE;

        error_temp[index] = raw_delta;

        // debug
//        float buffer_val[2];
//        buffer_val[0] = delta;
//        send_data_float(buffer_val, 1);
    }

    for (int i = 0; i < ERROR_LUT_SIZE; i++) {
        if (error_temp[i] == 0) {
            int last_i = i - 1;
            int next_i = i + 1;

            if (last_i < 0)
                last_i += ERROR_LUT_SIZE;
            if (next_i > ERROR_LUT_SIZE)
                next_i -= ERROR_LUT_SIZE;

            error_temp[i] = (error_temp[last_i] + error_temp[next_i]) / 2.0f;
        }
    }

    memcpy(m_config.encd_error_comp, error_temp, sizeof(error_temp));
    open_loop_voltage_control(foc, 0.0f, 0.0f, 0.0f);
}

void foc_set_torque_control_bandwidth(foc_t *foc, float bandwidth)
{
    foc->I_ctrl_bandwidth = bandwidth;
}

void open_loop_voltage_control(foc_t *foc, float vd_ref, float vq_ref, float angle_rad)
{
    uint32_t da, db, dc;
    const uint32_t pwm_res = foc->pwm_res;
    float sin_theta, cos_theta;

    pre_calc_sin_cos(angle_rad, &sin_theta, &cos_theta);
    inverse_park_transform(vd_ref, vq_ref, sin_theta, cos_theta, &foc->v_alpha, &foc->v_beta);
    svpwm(foc->v_alpha, foc->v_beta, foc->v_bus, pwm_res, &da, &db, &dc);

    *(foc->pwm_a) = CONSTRAIN(da, 0, pwm_res);
    *(foc->pwm_b) = CONSTRAIN(db, 0, pwm_res);
    *(foc->pwm_c) = CONSTRAIN(dc, 0, pwm_res);
}

void meas_inj_dq_process(foc_t *foc, float ts)
{
    const uint32_t wt = 256; // waiting time

    if (foc->meas_inj_start_flag) {
        float vd = 0.0f, vq = 0.0f;

        if (foc->meas_inj_target == RS) {
            vd = foc->meas_inj_amp;
            vq = 0.0f;
        } else {
            float angle = foc->meas_inj_omega * foc->meas_inj_n * ts;
            float v_inj = foc->meas_inj_amp * fast_sin(angle);

            if (foc->meas_inj_target == LD) {
                vd = v_inj;
                vq = 0.0f;
            } else if (foc->meas_inj_target == LQ) {
                vd = 0.0f;
                vq = v_inj;
            }
        }

        float theta_e = 0.0f;
        open_loop_voltage_control(foc, vd, vq, theta_e);

        float sin_theta, cos_theta;
        float id, iq;
        pre_calc_sin_cos(theta_e, &sin_theta, &cos_theta);
        float ib_calc2 = -foc->ia - foc->ic;
        clarke_park_transform(foc->ia, ib_calc2, sin_theta, cos_theta, &id, &iq);

        if (foc->meas_inj_n >= wt) {
            if (foc->meas_inj_target == RS || foc->meas_inj_target == LD) {
                Vd_buff[foc->meas_inj_n - wt] = vd;
                Id_buff[foc->meas_inj_n - wt] = id;
            } else if (foc->meas_inj_target == LQ) {
                Vq_buff[foc->meas_inj_n - wt] = vq;
                Iq_buff[foc->meas_inj_n - wt] = iq;
            }
        }

        foc->meas_inj_n++;

        if (foc->meas_inj_n >= (MAX_I_SAMPLE + wt)) {
            foc->meas_inj_n = 0;
            foc->meas_inj_start_flag = 0;
            open_loop_voltage_control(foc, 0, 0, 0);
        }
    }
}

void estimate_resistance(foc_t *foc)
{
    float mean_vd = 0, mean_id = 0;

    for (int i = 0; i < MAX_I_SAMPLE; i++) {
        mean_vd += Vd_buff[i];
        mean_id += Id_buff[i];
    }

    mean_vd /= MAX_I_SAMPLE;
    mean_id /= MAX_I_SAMPLE;

    foc->Rs = mean_vd / mean_id;
}

void estimate_inductance(foc_t *foc, float ts)
{
    float Vc_d = 0, Vs_d = 0, Ic_d = 0, Is_d = 0;
    float Vc_q = 0, Vs_q = 0, Ic_q = 0, Is_q = 0;
    float mean_vd = 0, mean_vq = 0, mean_id = 0, mean_iq = 0;

    // Remove DC offset
    for (int i = 0; i < MAX_I_SAMPLE; i++) {
        mean_vd += Vd_buff[i];
        mean_vq += Vq_buff[i];
        mean_id += Id_buff[i];
        mean_iq += Iq_buff[i];
    }

    mean_vd /= MAX_I_SAMPLE;
    mean_vq /= MAX_I_SAMPLE;
    mean_id /= MAX_I_SAMPLE;
    mean_iq /= MAX_I_SAMPLE;

    // Single-frequency DFT
    for (int i = 0; i < MAX_I_SAMPLE; i++) {
        float angle = foc->meas_inj_omega * i * ts;
        float vd = Vd_buff[i] - mean_vd;
        float vq = Vq_buff[i] - mean_vq;
        float id = Id_buff[i] - mean_id;
        float iq = Iq_buff[i] - mean_iq;

        Vc_d += vd * fast_cos(angle);
        Vs_d += vd * fast_sin(angle);
        Ic_d += id * fast_cos(angle);
        Is_d += id * fast_sin(angle);

        Vc_q += vq * fast_cos(angle);
        Vs_q += vq * fast_sin(angle);
        Ic_q += iq * fast_cos(angle);
        Is_q += iq * fast_sin(angle);
    }

    float norm = 2.0f / MAX_I_SAMPLE;
    Vc_d *= norm;
    Vs_d *= norm;
    Ic_d *= norm;
    Is_d *= norm;

    Vc_q *= norm;
    Vs_q *= norm;
    Ic_q *= norm;
    Is_q *= norm;

    // V & I amplitude
    float Vd_mag = sqrtf(Vc_d * Vc_d + Vs_d * Vs_d);
    float Id_mag = sqrtf(Ic_d * Ic_d + Is_d * Is_d);
    float Vq_mag = sqrtf(Vc_q * Vc_q + Vs_q * Vs_q);
    float Iq_mag = sqrtf(Ic_q * Ic_q + Is_q * Is_q);

    // (phi = arctan(Vs/Vc) - arctan(Is/Ic))
    float phi_d = atan2f(Vs_d, Vc_d) - atan2f(Is_d, Ic_d);
    float phi_q = atan2f(Vs_q, Vc_q) - atan2f(Is_q, Ic_q);

    // Impedansi & parameters
    float Zd_mag = Vd_mag / Id_mag;
    float Zq_mag = Vq_mag / Iq_mag;

    float Ld_est = (Zd_mag * sinf(phi_d)) / foc->meas_inj_omega;
    float Lq_est = (Zq_mag * sinf(phi_q)) / foc->meas_inj_omega;

    foc->Ld = fabs(Ld_est);
    foc->Lq = fabs(Lq_est);
}
