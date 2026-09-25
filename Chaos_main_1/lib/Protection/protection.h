/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : protection.h
  * @brief          : Lớp bảo vệ độ chính xác dữ liệu (CAN / SPI) cho STM32 slave
  *
  *  Mục tiêu: chặn dữ liệu sai/hỏng TRƯỚC khi nó đi vào policy (qua SPI lên
  *  Jetson) hoặc ra cơ cấu chấp hành (action xuống motor qua CAN).
  *
  *  Thư viện ĐỘC LẬP, không phụ thuộc HAL/CubeMX:
  *    - Thời gian (tick ms) được truyền vào từ ngoài (HAL_GetTick()).
  *    - Không tự đọc/ghi ngoại vi — chỉ kiểm tra & làm sạch dữ liệu.
  *  Nhờ vậy có thể unit-test trên PC và tái sử dụng ở dự án khác.
  *
  *  Các lớp bảo vệ (defense in depth):
  *    Lớp 2  Framing  : magic header/footer  -> chống SPI desync / byte-slip
  *    Lớp 2  Sequence : số khung tăng dần    -> phát hiện mất/lặp/sai thứ tự
  *    Lớp 3  CRC32    : zlib-compatible       -> phát hiện sai lệch nội dung
  *    Lớp 4a NaN/Inf  : isfinite              -> chặn mẫu bit phi số
  *    Lớp 4b Range    : clamp theo vật lý     -> chặn giá trị phi lý
  *    Lớp 4c Slew     : giới hạn tốc độ đổi   -> chặn spike đơn lẻ
  *    Lớp 5  Freshness: watchdog + bitmask    -> chống dữ liệu cũ (stale)
  *    Lớp 7  Stats    : bộ đếm lỗi            -> bắt lỗi gián đoạn
  *
  *  LƯU Ý: các ngưỡng giới hạn (PROT_*_MIN/MAX) bên dưới là GIÁ TRỊ TẠM —
  *  BẮT BUỘC chỉnh lại theo thông số cơ khí/điện thực tế của robot.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef PROTECTION_H
#define PROTECTION_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * CẤU HÌNH — chỉnh theo robot thực tế
 *
 *  ĐƠN VỊ (xác nhận từ firmware motor "Chaos_test3"):
 *    - Vị trí khớp  : ĐỘ (deg), robot frame (đã nhân AXIS_SIGNS).
 *    - Vận tốc khớp : RPM (vòng/phút), robot frame.
 *    - Action       : lệnh VỊ TRÍ mục tiêu — cùng đơn vị & cùng dải với vị trí.
 *
 *  !! CẢNH BÁO LỆCH ĐƠN VỊ: utils.h ghi joint pos = rad, vel = rad/s, nhưng
 *     dữ liệu THỰC TẾ trên bus CAN là deg / RPM (PublishObsToSpi memcpy thẳng,
 *     không quy đổi). Cần thống nhất lại với phía Jetson.
 * ============================================================ */
#ifndef PROT_NUM_JOINTS
#define PROT_NUM_JOINTS        12
#endif

/* --- Giới hạn vị trí TỪNG KHỚP (độ, robot frame) -----------------------
 * Nguồn: SP_LIMIT_MIN/MAX mỗi NODE_ID trong Chaos_test3/Core/Src/main.c.
 * DÙNG CHUNG cho cả: (a) kiểm vị trí phản hồi từ CAN, (b) clamp action.
 * Định nghĩa giá trị trong protection.c.                                  */
extern const float PROT_JOINT_POS_MIN[PROT_NUM_JOINTS];
extern const float PROT_JOINT_POS_MAX[PROT_NUM_JOINTS];

/* --- Giới hạn vận tốc khớp (RPM, đối xứng |v| <= MAX) ------------------
 * Vòng vị trí giới hạn tốc độ tham chiếu ~10 RPM (pos_out_max = 10),
 * homing ~15-20 RPM. Đặt biên rộng cho overshoot. Chỉnh theo đo thực tế.  */
#define PROT_JOINT_VEL_ABS_MAX  (60.0f)

/* --- Giới hạn tốc độ đổi action mỗi chu kỳ (slew, độ) ------------------
 * |action[t] - action[t-1]| <= PROT_ACTION_SLEW_MAX. Đặt theo tần số vòng
 * điều khiển + tốc độ khớp tối đa. Giá trị tạm — chỉnh theo thực tế.      */
#define PROT_ACTION_SLEW_MAX    (5.0f)

/* --- Giá trị "an toàn" khi action không hợp lệ (fail-safe) -------------
 * Action là VỊ TRÍ mục tiêu nên 0° KHÔNG phải lúc nào cũng an toàn (vd
 * khớp 6/7 dải [-100,0], 0 là biên trên). VÌ VẬY nên ưu tiên truyền
 * 'hold' (lệnh hợp lệ trước đó) vào Protection_ValidateActions; hằng số
 * này chỉ là phương án cuối khi chưa có lệnh hợp lệ nào.                  */
#define PROT_ACTION_SAFE        (0.0f)

/* --- Giới hạn cảm biến IMU (obs) -------------------------------------- */
#define PROT_ANGVEL_ABS_MAX     (50.0f)   /* rad/s */
#define PROT_GRAV_ABS_MAX       (1.0f)    /* [-1,1] đã chuẩn hóa + biên     */

/* ============================================================
 * KẾT QUẢ TRẢ VỀ
 * ============================================================ */
typedef enum {
    PROT_OK = 0,
    PROT_ERR_NULL,        /* con trỏ NULL                              */
    PROT_ERR_FRAMING,     /* magic header/footer sai -> nghi desync    */
    PROT_ERR_CRC,         /* CRC không khớp                            */
    PROT_ERR_SEQ,         /* sequence bất thường (gap/dup/ooo)         */
    PROT_ERR_NAN,         /* có NaN/Inf                                */
    PROT_ERR_RANGE,       /* ngoài tầm vật lý                          */
    PROT_ERR_STALE,       /* dữ liệu quá cũ                            */
} Protection_Result_t;

/* Kết quả kiểm tra sequence */
typedef enum {
    PROT_SEQ_FIRST = 0,   /* khung đầu tiên (chưa có mốc so sánh)      */
    PROT_SEQ_OK,          /* +1 đúng như mong đợi                      */
    PROT_SEQ_GAP,         /* nhảy cách quãng -> mất khung              */
    PROT_SEQ_DUP,         /* trùng số -> khung lặp / stale             */
    PROT_SEQ_OUT_OF_ORDER /* số lùi lại -> sai thứ tự                  */
} Protection_SeqResult_t;

/* ============================================================
 * BỘ ĐẾM LỖI (Lớp 7 — quan trắc lỗi gián đoạn)
 * ============================================================ */
typedef struct {
    uint32_t spi_framing_err;   /* magic sai                          */
    uint32_t spi_crc_err;       /* CRC sai                            */
    uint32_t spi_seq_gap;       /* mất khung                          */
    uint32_t spi_seq_dup;       /* khung lặp                          */
    uint32_t spi_seq_ooo;       /* sai thứ tự                         */
    uint32_t action_nan;        /* action NaN/Inf bị thay bằng safe   */
    uint32_t action_clip;       /* action bị kẹp về biên              */
    uint32_t action_slew;       /* action bị giới hạn tốc độ đổi      */
    uint32_t can_pos_bad;       /* pos khớp NaN/ngoài tầm             */
    uint32_t can_vel_bad;       /* vel khớp NaN/ngoài tầm             */
    uint32_t obs_nan;           /* obs (IMU/cmd) NaN/Inf              */
    uint32_t stale_events;      /* số lần watchdog hết hạn            */
} Protection_Stats_t;

/* ============================================================
 * TRẠNG THÁI CÓ NHỚ
 * ============================================================ */

/* Bộ kiểm sequence */
typedef struct {
    uint16_t last;
    uint8_t  initialized;
} Protection_Seq_t;

/* Watchdog độ tươi (freshness) — tick ms truyền từ ngoài vào */
typedef struct {
    uint32_t last_ms;
    uint32_t timeout_ms;
    uint8_t  armed;
} Protection_Watchdog_t;

/* Bộ giới hạn slew theo từng kênh (action) */
typedef struct {
    float   prev[PROT_NUM_JOINTS];
    float   max_delta;
    uint8_t initialized;
} Protection_Slew_t;

/* ============================================================
 * 0) Tiện ích chung
 * ============================================================ */
void Protection_StatsReset(Protection_Stats_t *st);

/* ============================================================
 * Lớp 4a — NaN / Inf
 * ============================================================ */

/* true nếu v là số hữu hạn (không NaN, không Inf). Dựa trên bit mũ IEEE-754,
 * KHÔNG cần <math.h>, an toàn với mọi mức tối ưu trình biên dịch. */
bool  Protection_IsFinite(float v);

/* true nếu mọi phần tử trong mảng đều hữu hạn. */
bool  Protection_ArrayAllFinite(const float *arr, uint32_t n);

/* ============================================================
 * Lớp 4b — Clamp / Range
 * ============================================================ */
float Protection_Clamp(float v, float lo, float hi);
bool  Protection_InRange(float v, float lo, float hi);

/* Làm sạch 1 giá trị: nếu NaN/Inf -> trả 'fallback'; ngược lại kẹp [lo,hi].
 * LƯU Ý THỨ TỰ: kiểm isfinite TRƯỚC clamp, vì clamp(NaN) trả về NaN. */
float Protection_Sanitize(float v, float lo, float hi, float fallback);

/* ============================================================
 * Đọc/ghi float little-endian an toàn (tránh lỗi căn lề / endianness)
 * ============================================================ */
float Protection_ReadFloatLE(const uint8_t *p);
void  Protection_WriteFloatLE(uint8_t *p, float v);

/* ============================================================
 * Lớp 3 — CRC32 (reflected, poly 0xEDB88320, init/xorout 0xFFFFFFFF)
 *   TƯƠNG THÍCH Python: zlib.crc32(data) & 0xFFFFFFFF
 *   Tự kiểm: CRC32("123456789") == 0xCBF43926
 * ============================================================ */
uint32_t Protection_CRC32(const void *data, uint32_t len);
uint32_t Protection_CRC32_Update(uint32_t crc, const void *data, uint32_t len);

/* Ghi CRC32 (4 byte LE) của 'payload_len' byte đầu vào ngay sau payload. */
void     Protection_AppendCRC32(uint8_t *buf, uint32_t payload_len);

/* Kiểm khung có 4 byte CRC32 LE ở cuối: total_len = payload + 4. */
bool     Protection_CheckTrailingCRC32(const uint8_t *buf, uint32_t total_len);

/* ============================================================
 * Lớp 2 — Framing (magic) & Sequence
 * ============================================================ */

/* So khớp 4 byte đầu buf với 'magic' (đọc LE). */
bool Protection_CheckMagic(const uint8_t *buf, uint32_t magic);
void Protection_WriteMagic(uint8_t *buf, uint32_t magic);

void                    Protection_SeqInit(Protection_Seq_t *s);
/* Kiểm + cập nhật mốc sequence. Tự cộng dồn vào 'st' nếu st != NULL. */
Protection_SeqResult_t  Protection_SeqCheck(Protection_Seq_t *s, uint16_t seq,
                                            Protection_Stats_t *st);

/* ============================================================
 * Lớp 5 — Freshness / Watchdog & validity mask
 * ============================================================ */
void Protection_WatchdogInit(Protection_Watchdog_t *w, uint32_t timeout_ms);
void Protection_WatchdogFeed(Protection_Watchdog_t *w, uint32_t now_ms);
bool Protection_WatchdogExpired(const Protection_Watchdog_t *w, uint32_t now_ms);

/* Trả bitmask các kênh THIẾU (mong đợi nhưng chưa nhận). 0 = đủ. */
uint16_t Protection_MaskMissing(uint16_t got, uint16_t expected);
bool     Protection_MaskComplete(uint16_t got, uint16_t expected);

/* ============================================================
 * Lớp 4c — Slew-rate limiter (per-channel)
 * ============================================================ */
void  Protection_SlewInit(Protection_Slew_t *s, float max_delta);
/* Giới hạn 1 kênh idx về [prev-Δ, prev+Δ]; cập nhật prev. */
float Protection_SlewStep(Protection_Slew_t *s, uint32_t idx, float target,
                          Protection_Stats_t *st);

/* ============================================================
 * VALIDATOR CẤP CAO — gắn trực tiếp vào luồng dữ liệu của project
 * ============================================================ */

/* --- Action từ Jetson (SPI) trước khi forward xuống motor (CAN) ---------
 * actions_in : PROT_NUM_JOINTS giá trị thô (vd spi_rx_buffer[1..12]), đơn vị độ.
 * actions_out: kết quả đã làm sạch — kẹp về dải [POS_MIN[i], POS_MAX[i]] từng khớp.
 * hold       : (tùy chọn) lệnh hợp lệ trước đó dùng làm fail-safe khi gặp
 *              NaN/Inf. Nếu NULL -> dùng hằng PROT_ACTION_SAFE.
 *              Khuyến nghị truyền prev_actions để "giữ lệnh cũ" thay vì nhảy về 0.
 * Có thể trùng con trỏ in/out (xử lý tại chỗ).
 * Trả PROT_OK nếu không phải sửa gì; ngược lại trả mã lỗi đầu tiên
 * (dữ liệu vẫn được làm sạch để dùng an toàn). */
Protection_Result_t Protection_ValidateActions(const float *actions_in,
                                               float *actions_out,
                                               const float *hold,
                                               Protection_Stats_t *st);

/* Áp slew lên mảng action ĐÃ làm sạch (gọi sau ValidateActions). */
void Protection_ApplyActionSlew(Protection_Slew_t *s, float *actions,
                                Protection_Stats_t *st);

/* --- Feedback 1 khớp từ CAN (pos, vel) ---------------------------------
 * joint : chỉ số khớp 0..PROT_NUM_JOINTS-1 (để tra giới hạn vị trí riêng).
 * pos   : vị trí (độ) — kẹp về [POS_MIN[joint], POS_MAX[joint]].
 * vel   : vận tốc (RPM) — kẹp về [-VEL_ABS_MAX, +VEL_ABS_MAX].
 * Làm sạch tại chỗ nếu NaN/Inf hoặc ngoài tầm, đánh dấu lỗi vào stats.
 * Trả true nếu cả pos & vel hợp lệ (hữu hạn & trong tầm). */
bool Protection_ValidateJoint(uint32_t joint, float *pos, float *vel,
                              Protection_Stats_t *st);

#ifdef __cplusplus
}
#endif

#endif /* PROTECTION_H */
