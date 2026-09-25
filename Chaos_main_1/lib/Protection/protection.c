/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : protection.c
  * @brief          : Lớp bảo vệ độ chính xác dữ liệu (CAN / SPI) — cài đặt
  ******************************************************************************
  */
/* USER CODE END Header */

#include "protection.h"
#include <string.h>

/* Union để soi/đổi bit của float mà không vi phạm strict-aliasing. */
typedef union { float f; uint32_t u; } prot_f32_t;

/* ============================================================
 * GIỚI HẠN VỊ TRÍ TỪNG KHỚP (độ, robot frame)
 *   Nguồn: SP_LIMIT_MIN/MAX mỗi NODE_ID trong Chaos_test3/Core/Src/main.c
 *     j0 [-50,50]  j1 [-50,50]  j2 [-15,10]  j3 [-10,15]
 *     j4 [-10,10]  j5 [-10,10]  j6 [-100,0]  j7 [-100,0]
 *     j8..j11 [-45,45]
 * ============================================================ */
_Static_assert(PROT_NUM_JOINTS == 12,
               "Bảng giới hạn khớp dưới đây giả định 12 khớp — cập nhật nếu đổi");

const float PROT_JOINT_POS_MIN[PROT_NUM_JOINTS] = {
    -50.0f, -50.0f, -15.0f, -10.0f, -10.0f, -10.0f,
   -100.0f,-100.0f, -45.0f, -45.0f, -45.0f, -45.0f
};
const float PROT_JOINT_POS_MAX[PROT_NUM_JOINTS] = {
     50.0f,  50.0f,  10.0f,  15.0f,  10.0f,  10.0f,
      0.0f,   0.0f,  45.0f,  45.0f,  45.0f,  45.0f
};

/* ============================================================
 * 0) Tiện ích chung
 * ============================================================ */
void Protection_StatsReset(Protection_Stats_t *st)
{
    if (st) memset(st, 0, sizeof(*st));
}

/* ============================================================
 * Lớp 4a — NaN / Inf
 * ============================================================ */
bool Protection_IsFinite(float v)
{
    prot_f32_t x;
    x.f = v;
    /* Inf/NaN <=> 8 bit mũ toàn 1 (0xFF). Hữu hạn <=> mũ != 0xFF. */
    return (x.u & 0x7F800000u) != 0x7F800000u;
}

bool Protection_ArrayAllFinite(const float *arr, uint32_t n)
{
    if (!arr) return false;
    for (uint32_t i = 0; i < n; i++)
        if (!Protection_IsFinite(arr[i])) return false;
    return true;
}

/* ============================================================
 * Lớp 4b — Clamp / Range
 * ============================================================ */
float Protection_Clamp(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;   /* CHÚ Ý: nếu v là NaN, mọi so sánh = false -> trả về NaN.
                 * Vì vậy luôn gọi Protection_IsFinite trước khi clamp. */
}

bool Protection_InRange(float v, float lo, float hi)
{
    return Protection_IsFinite(v) && (v >= lo) && (v <= hi);
}

float Protection_Sanitize(float v, float lo, float hi, float fallback)
{
    if (!Protection_IsFinite(v)) return fallback;   /* chặn NaN/Inf TRƯỚC */
    return Protection_Clamp(v, lo, hi);
}

/* ============================================================
 * Đọc/ghi float little-endian an toàn
 * ============================================================ */
float Protection_ReadFloatLE(const uint8_t *p)
{
    prot_f32_t x;
    x.u =  (uint32_t)p[0]
        | ((uint32_t)p[1] << 8)
        | ((uint32_t)p[2] << 16)
        | ((uint32_t)p[3] << 24);
    return x.f;
}

void Protection_WriteFloatLE(uint8_t *p, float v)
{
    prot_f32_t x; x.f = v;
    p[0] = (uint8_t)(x.u);
    p[1] = (uint8_t)(x.u >> 8);
    p[2] = (uint8_t)(x.u >> 16);
    p[3] = (uint8_t)(x.u >> 24);
}

/* ============================================================
 * Lớp 3 — CRC32 (zlib-compatible)
 * ============================================================ */
uint32_t Protection_CRC32_Update(uint32_t crc, const void *data, uint32_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    crc = ~crc;
    while (len--)
    {
        crc ^= *p++;
        for (uint8_t k = 0; k < 8; k++)
            crc = (crc & 1u) ? (crc >> 1) ^ 0xEDB88320u : (crc >> 1);
    }
    return ~crc;
}

uint32_t Protection_CRC32(const void *data, uint32_t len)
{
    return Protection_CRC32_Update(0u, data, len);
}

void Protection_AppendCRC32(uint8_t *buf, uint32_t payload_len)
{
    uint32_t crc = Protection_CRC32(buf, payload_len);
    buf[payload_len + 0] = (uint8_t)(crc);
    buf[payload_len + 1] = (uint8_t)(crc >> 8);
    buf[payload_len + 2] = (uint8_t)(crc >> 16);
    buf[payload_len + 3] = (uint8_t)(crc >> 24);
}

bool Protection_CheckTrailingCRC32(const uint8_t *buf, uint32_t total_len)
{
    if (!buf || total_len < 4) return false;
    uint32_t payload_len = total_len - 4u;
    uint32_t want = Protection_CRC32(buf, payload_len);
    uint32_t got  =  (uint32_t)buf[payload_len + 0]
                  | ((uint32_t)buf[payload_len + 1] << 8)
                  | ((uint32_t)buf[payload_len + 2] << 16)
                  | ((uint32_t)buf[payload_len + 3] << 24);
    return want == got;
}

/* ============================================================
 * Lớp 2 — Framing (magic) & Sequence
 * ============================================================ */
bool Protection_CheckMagic(const uint8_t *buf, uint32_t magic)
{
    if (!buf) return false;
    uint32_t got =  (uint32_t)buf[0]
                 | ((uint32_t)buf[1] << 8)
                 | ((uint32_t)buf[2] << 16)
                 | ((uint32_t)buf[3] << 24);
    return got == magic;
}

void Protection_WriteMagic(uint8_t *buf, uint32_t magic)
{
    buf[0] = (uint8_t)(magic);
    buf[1] = (uint8_t)(magic >> 8);
    buf[2] = (uint8_t)(magic >> 16);
    buf[3] = (uint8_t)(magic >> 24);
}

void Protection_SeqInit(Protection_Seq_t *s)
{
    if (!s) return;
    s->last = 0;
    s->initialized = 0;
}

Protection_SeqResult_t Protection_SeqCheck(Protection_Seq_t *s, uint16_t seq,
                                           Protection_Stats_t *st)
{
    if (!s) return PROT_SEQ_OK;

    if (!s->initialized)
    {
        s->initialized = 1;
        s->last = seq;
        return PROT_SEQ_FIRST;
    }

    uint16_t expected = (uint16_t)(s->last + 1u);

    if (seq == expected)
    {
        s->last = seq;
        return PROT_SEQ_OK;
    }

    if (seq == s->last)
    {
        if (st) st->spi_seq_dup++;
        return PROT_SEQ_DUP;          /* lặp / stale — KHÔNG cập nhật last */
    }

    /* Khoảng cách có dấu trên vòng 16-bit: >0 nghĩa là tiến (mất khung),
     * <0 nghĩa là lùi (sai thứ tự). */
    int16_t delta = (int16_t)(seq - s->last);
    if (delta > 0)
    {
        if (st) st->spi_seq_gap++;
        s->last = seq;               /* tiến tới khung mới nhất             */
        return PROT_SEQ_GAP;
    }

    if (st) st->spi_seq_ooo++;
    return PROT_SEQ_OUT_OF_ORDER;     /* khung cũ — KHÔNG cập nhật last      */
}

/* ============================================================
 * Lớp 5 — Freshness / Watchdog
 * ============================================================ */
void Protection_WatchdogInit(Protection_Watchdog_t *w, uint32_t timeout_ms)
{
    if (!w) return;
    w->last_ms    = 0;
    w->timeout_ms = timeout_ms;
    w->armed      = 0;
}

void Protection_WatchdogFeed(Protection_Watchdog_t *w, uint32_t now_ms)
{
    if (!w) return;
    w->last_ms = now_ms;
    w->armed   = 1;
}

bool Protection_WatchdogExpired(const Protection_Watchdog_t *w, uint32_t now_ms)
{
    if (!w || !w->armed) return false;          /* chưa từng được feed       */
    /* Trừ unsigned -> an toàn khi tick HAL_GetTick() tràn (wrap 32-bit).   */
    return (uint32_t)(now_ms - w->last_ms) >= w->timeout_ms;
}

uint16_t Protection_MaskMissing(uint16_t got, uint16_t expected)
{
    return (uint16_t)(expected & ~got);
}

bool Protection_MaskComplete(uint16_t got, uint16_t expected)
{
    return (uint16_t)(got & expected) == expected;
}

/* ============================================================
 * Lớp 4c — Slew-rate limiter
 * ============================================================ */
void Protection_SlewInit(Protection_Slew_t *s, float max_delta)
{
    if (!s) return;
    memset(s->prev, 0, sizeof(s->prev));
    s->max_delta   = max_delta;
    s->initialized = 0;
}

float Protection_SlewStep(Protection_Slew_t *s, uint32_t idx, float target,
                          Protection_Stats_t *st)
{
    if (!s || idx >= PROT_NUM_JOINTS) return target;

    /* Lần đầu: nhận thẳng giá trị để không "kéo" từ 0 về. */
    if (!s->initialized)
    {
        s->prev[idx] = target;
        return target;
    }

    float prev = s->prev[idx];
    float lo   = prev - s->max_delta;
    float hi   = prev + s->max_delta;
    float out  = target;

    if (out > hi)      { out = hi; if (st) st->action_slew++; }
    else if (out < lo) { out = lo; if (st) st->action_slew++; }

    s->prev[idx] = out;
    return out;
}

/* ============================================================
 * VALIDATOR CẤP CAO
 * ============================================================ */
Protection_Result_t Protection_ValidateActions(const float *actions_in,
                                               float *actions_out,
                                               const float *hold,
                                               Protection_Stats_t *st)
{
    if (!actions_in || !actions_out) return PROT_ERR_NULL;

    Protection_Result_t first_err = PROT_OK;

    for (uint32_t i = 0; i < PROT_NUM_JOINTS; i++)
    {
        float lo = PROT_JOINT_POS_MIN[i];   /* dải vị trí riêng từng khớp   */
        float hi = PROT_JOINT_POS_MAX[i];
        float v  = actions_in[i];

        if (!Protection_IsFinite(v))
        {
            /* fail-safe: giữ lệnh hợp lệ trước đó nếu có (an toàn hơn nhảy về 0
             * với các khớp có dải bất đối xứng), kẹp lại cho chắc. */
            actions_out[i] = hold
                ? Protection_Sanitize(hold[i], lo, hi, PROT_ACTION_SAFE)
                : Protection_Clamp(PROT_ACTION_SAFE, lo, hi);
            if (st) st->action_nan++;
            if (first_err == PROT_OK) first_err = PROT_ERR_NAN;
            continue;
        }

        float c = Protection_Clamp(v, lo, hi);
        if (c != v)
        {
            if (st) st->action_clip++;
            if (first_err == PROT_OK) first_err = PROT_ERR_RANGE;
        }
        actions_out[i] = c;
    }

    return first_err;
}

void Protection_ApplyActionSlew(Protection_Slew_t *s, float *actions,
                                Protection_Stats_t *st)
{
    if (!s || !actions) return;

    for (uint32_t i = 0; i < PROT_NUM_JOINTS; i++)
        actions[i] = Protection_SlewStep(s, i, actions[i], st);

    /* Đánh dấu đã có mốc sau khi quét xong cả batch đầu tiên. */
    s->initialized = 1;
}

bool Protection_ValidateJoint(uint32_t joint, float *pos, float *vel,
                              Protection_Stats_t *st)
{
    if (!pos || !vel || joint >= PROT_NUM_JOINTS) return false;

    float plo = PROT_JOINT_POS_MIN[joint];
    float phi = PROT_JOINT_POS_MAX[joint];
    bool  ok  = true;

    if (!Protection_InRange(*pos, plo, phi))
    {
        if (st) st->can_pos_bad++;
        *pos = Protection_Sanitize(*pos, plo, phi, plo);   /* kẹp; lỗi -> biên dưới */
        ok = false;
    }

    if (!Protection_InRange(*vel, -PROT_JOINT_VEL_ABS_MAX, PROT_JOINT_VEL_ABS_MAX))
    {
        if (st) st->can_vel_bad++;
        *vel = Protection_Sanitize(*vel, -PROT_JOINT_VEL_ABS_MAX,
                                   PROT_JOINT_VEL_ABS_MAX, 0.0f);
        ok = false;
    }

    return ok;
}
