/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : utils.h
  * @brief          : Application-level utility declarations
  *                   (CAN, SPI, GPIO callbacks, Jetson watchdog)
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef UTILS_H
#define UTILS_H

#include "main.h"
#include "BQ76952.h"
#include "MPU6050.h"
#include "protection.h"
#include <string.h>

/* ============================== Peripheral handles (defined in main.c) ============================== */
extern CAN_HandleTypeDef  hcan2;
extern SPI_HandleTypeDef  hspi3;
extern I2C_HandleTypeDef  hi2c1;
extern TIM_HandleTypeDef  htim2;
extern TIM_HandleTypeDef  htim3;
extern TIM_HandleTypeDef  htim4;
extern TIM_HandleTypeDef  htim5;   /* init-done handshake (5s poll) */

/* ============================== Build mode ============================== */
#define TEST        0
#define TEST_NO_TIM 0   /* 1 = strip TIM2/3/4 logic, run SPI+CAN bare */

/* ============================== Protection ==============================
 *  PROT_ENABLE_ACTION_SLEW: giới hạn tốc độ đổi LỆNH VỊ TRÍ mỗi chu kỳ.
 *  Mặc định TẮT (0): slew trên position-command = rate-limit quỹ đạo, có thể
 *  cản dáng đi nhanh (vd khớp gối dải ±100°). Chỉ BẬT (1) sau khi đo biên độ
 *  thay đổi action thực tế và chỉnh PROT_ACTION_SLEW_MAX cho phù hợp.
 *  Lớp NaN (4a) + range per-joint (4b) LUÔN bật, không phụ thuộc cờ này.    */
#define PROT_ENABLE_ACTION_SLEW   0

/* Watchdog: coi như mất lệnh từ Jetson nếu quá ngần này (ms) không có khung
 * action hợp lệ. 50 Hz -> chu kỳ 20 ms; 60 ms ~ lỡ 3 khung.                */
#define PROT_ACTION_TIMEOUT_MS    60u

/* Watchdog SPI: nếu quá ngần này (ms) Jetson KHÔNG clock SPI (không lấy obs/
 * không gửi action) -> coi như mất link -> về "chưa init" (STATUS=WAIT) +
 * broadcast CAN 150 cho node homing. Đặt rộng để không kích nhầm khi Jetson
 * tạm dừng ngắn (50Hz -> 20ms/khung; 1000ms ~ lỡ 50 khung).                 */
#define SPI_TIMEOUT_MS            1000u

/* ============================== LED ============================== */
#define LED_PORT    GPIOE
#define LED_L2      GPIO_PIN_4
#define LED_L3      GPIO_PIN_3
#define LED_L4      GPIO_PIN_2

/* ============================== BQ76952 ============================== */
#define BQ_ALERT_PORT     GPIOC
#define BQ_ALERT_PIN      GPIO_PIN_1
#define BQ_RST_SHUNT_PORT GPIOC
#define BQ_RST_SHUNT_PIN  GPIO_PIN_2

/* ============================== Jetson ↔ STM32 GPIO handshake ==============================
 *
 *  PD6  INPUT   Jetson → STM32 : rising edge = "start collecting, I'm ready"
 *  PD5  OUTPUT  STM32 → Jetson : HIGH = "obs buffer ready, you may clock SPI"
 */
#define JETSON_IN_PORT   GPIOD
#define JETSON_IN_PIN    GPIO_PIN_6
#define JETSON_OUT_PORT  GPIOD
#define JETSON_OUT_PIN   GPIO_PIN_5

/* ============================== Timer roles ==============================
 *
 *  TIM2  PD5 watchdog (one-shot, 10 ms)
 *        Started in PublishObsToSpi when PD5 goes HIGH.
 *        If SPI T1 never completes within 10 ms, TIM2 ISR forces PD5 LOW.
 *        Stopped in HAL_SPI_TxRxCpltCallback on normal SPI completion.
 *        tim2_tick counts 1 ms ticks; triggers at 10.
 *
 *  TIM3  Cycle timer (1 ms ticks, reset on every valid EXTI)
 *        Tick  5 ms : abort previous action chain if still running, then send SYNC.
 *        Tick 10 ms+: collecting timeout — 10 ms measured from sync_start_ms
 *                     (Option A: timeout from CAN_SendSync call, not from EXTI).
 *        tim3_tick counts 1 ms ticks.
 *
 *  TIM4  EXTI gate (15 ms)
 *        Started on every valid EXTI.  While running, new EXTI triggers are
 *        rejected as glitch / too-soon.  Clears exti_gate_active on expiry.
 *        tim4_tick counts 1 ms ticks; triggers at 15.
 */

/* ============================== STATE_COLLECTING timeout ==============================
 *
 *  If not all 12 joints reply within COLLECTING_TIMEOUT_MS milliseconds (10 ms),
 *  STM32 force-publishes partial obs. Missing joints keep the value from the
 *  previous cycle (can_rx_data is never memset to 0 between cycles).
 *  10 ms gives motors ~8ms of slack over normal CAN round-trip (~1-2 ms).
 */
#define COLLECTING_TIMEOUT_MS  10u       /* 10 ms                               */

/* ============================== Obs thiếu khớp — chính sách an toàn ==============================
 *
 *  Mỗi chu kỳ, đếm số khớp KHÔNG trả lời (missing).
 *    missing <= OBS_MAX_MISSING        : gửi obs bình thường (khớp thiếu xài số cũ).
 *    missing >  OBS_MAX_MISSING        : KHÔNG gửi obs vòng đó (PD5 thấp -> motor tự giữ).
 *        nếu lặp >= OBS_SEVERE_ENTER vòng liền : KHÓA (ngừng nhận lệnh, đứng yên).
 *    khi đang khóa: đủ OBS_RECOVER_GOOD vòng tốt (<=MAX) liên tiếp -> mở khóa.
 *  Không bao giờ cắt FET — chỉ giữ nguyên.
 */
#define OBS_MAX_MISSING    3u   /* cho phép thiếu tối đa 3 khớp mà vẫn gửi obs   */
#define OBS_SEVERE_ENTER   4u   /* thiếu 4+ khớp 4 vòng liền -> khóa             */
#define OBS_RECOVER_GOOD   5u   /* 5 vòng (≤3 thiếu) liên tiếp -> mở khóa        */
#define OBS_REINIT_CYCLES  250u /* ~5s mất dữ liệu liên tục -> reset về init      */

/* ============================== Phát hiện ngã (từ projected gravity) ==============================
 *
 *  Thẳng đứng = 90° so mặt đất. "Sắp ngã" khi góc so mặt đất < FALL_GROUND_ANGLE_DEG (70°),
 *  tức nghiêng khỏi phương thẳng đứng > (90-70)=20°.
 *
 *  KHỚP công thức Jetson:  tilt = atan2(hypot(gx,gy), -gz) > 20°  (SIGN-AWARE).
 *  Tương đương (h = hypot(gx,gy), gz = trục đứng, đứng thẳng gz≈-1):
 *     NGÃ  <=>  gz >= 0            (nghiêng >= 90°, gần lật/úp)
 *           OR  h^2 > gz^2 * tan^2(20°)   (khi gz<0)
 *  Dùng gz CÓ DẤU (không phải gz^2) để phân biệt đứng (-z) với lật (+z).
 *  FALL_TILT_TAN2 = tan^2(90 - FALL_GROUND_ANGLE_DEG). NẾU đổi 70° -> đổi số này.
 */
#define FALL_GROUND_ANGLE_DEG  70.0f    /* < ngưỡng này (so mặt đất) = sắp ngã   */
#define FALL_TILT_TAN2         0.13247f /* = tan^2(20°); đổi nếu đổi góc trên     */
#define FALL_ENTER_CYCLES      3u      /* nghiêng liên tiếp ngần này vòng -> ngã (chống nhiễu) */
#define FALL_EXIT_CYCLES       10u     /* thẳng lại ngần này vòng -> hết ngã     */
#define FALLING_BURST_CYCLES   10u     /* gửi CAN 150 theo CỤM (10 vòng) ở cạnh ngã,
                                        * KHÔNG gửi liên tục -> node reset 1 lần (không lặp) */

/* ============================== Arming — cổng ổn định trước khi xuất action ==============================
 *
 *  Sau init xong / sau khi đứng thẳng lại từ ngã / sau mọi gián đoạn, STM KHÔNG
 *  forward action ngay. Phải thấy RUN + obs tốt + action từ Jetson tươi LIÊN TỤC
 *  trong STABILIZE_MS thì mới "armed" (cho phép xuất action xuống node).
 *  Bất kỳ gián đoạn nào (ngã, mất khớp, mất action, bus-off, chưa đủ node) -> reset.
 *
 *  Jetson cũng tự đợi ~5s sau khi đứng thẳng mới gửi action -> tổng ~10s khi reset.
 */
#define STABILIZE_MS           5000u   /* 5 s ổn định mới cho xuất action (init=ngã) */

/* "Ổn định" = obs ÍT DAO ĐỘNG: mỗi vòng, mọi khớp đổi < STABLE_DELTA_DEG so với
 * vòng trước (robot đứng yên, không rung) + đủ data + action tươi.
 * Giá trị tạm — chỉnh theo độ rung thực tế khi robot đứng yên.               */
#define STABLE_DELTA_DEG       1.0f

/* ============================== SPI frame layout  (46 floats × 4 bytes = 184 bytes) ==============================
 *
 *  spi_rx_buffer  — data received FROM Jetson (master→slave)
 *  spi_tx_buffer  — data sent     TO   Jetson (slave→master)
 *
 *  !!! ĐỔI KHUNG: 45 -> 46 float (thêm trường STATUS). JETSON PHẢI dùng 46 float
 *      (184 byte) khớp y hệt, nếu không SPI sẽ lệch khung (chưa có CRC).
 *
 *  ĐƠN VỊ (thực tế trên dây): joint pos = ĐỘ (deg), joint vel = RPM.
 *      (rad / rad/s chỉ ở Jetson; IMU angular velocity vẫn là rad/s.)
 *
 *  RX layout (Jetson → STM32):
 *   [0]      : frame type  2.0f = CMD_ACTION
 *   [1..12]  : joint actions a0..a11 (lệnh vị trí, deg)
 *   [13..45] : reserved / padding (ignored by STM32)
 *
 *  TX layout (STM32 → Jetson):  46 floats total
 *   [0  –  2] base angular velocity  (3)   ← MPU6050 angvel_x/y/z   (rad/s)
 *   [3  –  5] projected gravity      (3)   ← MPU6050 grav_x/y/z     (normalised)
 *   [6  –  8] velocity command       (3)   ← set by user/operator
 *   [9  – 20] joint position         (12)  ← CAN feedback pos joint 0..11  (DEG)
 *   [21 – 32] joint velocity         (12)  ← CAN feedback vel joint 0..11  (RPM)
 *   [33 – 44] prev actions           (12)  ← last CMD_ACTION echoed back
 *   [45]      STATUS / OK-id         (1)   ← STM_STATUS_WAIT / STM_STATUS_READY
 *             WAIT  = init/đang ngã/recovery -> Jetson GIỮ, không xuất action.
 *             READY = đã homing xong + đứng thẳng -> Jetson được chạy lại.
 *             (Jetson chỉ dùng READY khi CHÍNH NÓ cũng đã phát hiện ngã -> khóa chéo.)
 */
#define SPI_PAYLOAD_FLOATS  46
#define SPI_IDX_ANG_VEL      0   /* [0  – 2 ] base angular velocity  (rad/s)  */
#define SPI_IDX_GRAV         3   /* [3  – 5 ] projected gravity       (norm)  */
#define SPI_IDX_CMD          6   /* [6  – 8 ] velocity command vx/vy/vyaw     */
#define SPI_IDX_JOINT_POS    9   /* [9  – 20] joint positions         (deg)   */
#define SPI_IDX_JOINT_VEL   21   /* [21 – 32] joint velocities        (RPM)   */
#define SPI_IDX_ACTIONS     33   /* [33 – 44] prev actions echoed back        */
#define SPI_IDX_STATUS      45   /* [45]      STM status / OK-id              */

/* ============================== SPI Frame Type IDs ============================== */
#define SPI_CMD_OBS_REQUEST  1.0f   /* Jetson requests observation              */
#define SPI_CMD_ACTION       2.0f   /* Jetson sends action[0..11]               */

/* ============================== STM status (trường [45], báo Jetson) ============================== */
#define STM_STATUS_WAIT      0.0f   /* chưa sẵn sàng (init/ngã/recovery)        */
#define STM_STATUS_READY     1.0f   /* homing xong + đứng thẳng -> Jetson chạy lại */

/* ============================== CAN ============================== */
#define FILTER_ID         0x000u  /* Base ID = 0                               */
#define FILTER_MASK       0x780u  /* Check 4 high bits                         */

#define CAN_ID_SYNC       0x032u  /* 50 decimal — broadcast sync               */
#define CAN_OBS_ID_BASE   0x001u  /* node 0x001 = joint 0                      */
#define CAN_OBS_ID_MAX    0x00Cu  /* node 0x00C = joint 11                     */
#define NUM_JOINTS        12

/* Action frame: STM32 → motor node
 *   StdId = CAN_ACTION_ID_BASE + joint_index   (0x033..0x03E)
 *   DLC   = 4 bytes = 1 float
 */
#define CAN_ACTION_ID_BASE  0x033u  /* 51 decimal                              */

/* Init-done handshake (đối ứng với Chaos_test3):
 *   CAN_ID_CHECK_INIT_DONE : main board → broadcast hỏi "đã init xong chưa?"
 *   CAN_ID_INIT_DONE_BASE+n: node n → báo đã homing/init xong  (101..112)
 * Main board gom đủ 12 bit (INIT_DONE_ALL_MASK) mới cho phép giao tiếp Jetson. */
#define CAN_ID_CHECK_INIT_DONE  100u   /* 0x64 — cố định, chung mọi node          */
#define CAN_ID_INIT_DONE_BASE   101u   /* node0 = 101                            */
#define CAN_ID_INIT_DONE_MAX    (CAN_ID_INIT_DONE_BASE + NUM_JOINTS - 1u)  /* 112 */
#define INIT_DONE_ALL_MASK      0x0FFFu /* 12 bit = đủ 12 node                    */

/* Falling: main board broadcast -> mọi node tự vào trạng thái "falling" riêng. */
#define CAN_ID_FALLING          150u   /* 0x96 — broadcast báo robot đang ngã     */

/* ============================== Shared state (defined in utils.c) ============================== */

/* SPI buffers */
extern float          spi_tx_buffer[SPI_PAYLOAD_FLOATS];
extern float          spi_rx_buffer[SPI_PAYLOAD_FLOATS];
extern volatile int   spi_callback;

/* CAN TX */
extern CAN_TxHeaderTypeDef  TxHeader;
extern uint8_t              can_tx_data[8];
extern uint32_t             txMailbox;
extern volatile uint8_t     action_idx;
extern volatile uint8_t     sync_pending;
extern volatile uint8_t     action_pending;
extern int                  sent_sync;

/* CAN RX */
extern CAN_RxHeaderTypeDef  RxHeader;
extern float                can_rx_data[NUM_JOINTS * 2];
extern volatile uint16_t    rx_joints_received;
extern volatile int         rx_ok;

/* Jetson GPIO shadow flags */
extern volatile uint8_t     jetson_out;
extern volatile int         jetson_in;

/* EXTI gate flag — set on valid EXTI, cleared by TIM4 ISR after 15 ms */
extern volatile uint8_t     exti_gate_active;

/* Timestamp (ms) when last CAN_SendSync was called; TIM3 compares against this */
extern volatile uint32_t    sync_start_ms;

/* SPI DMA re-arm failed — main loop retries */
extern volatile uint8_t     spi_dma_error;

/* Mốc thời gian (ms) lần SPI TxRx hoàn tất gần nhất — cho watchdog SPI ở main loop */
extern volatile uint32_t    spi_last_ms;

/* CAN error / bus-off counters (readable in debugger) */
extern volatile uint32_t    can_error_count;
extern volatile uint8_t     can_busoff;

/* Collecting timeout debug */
extern volatile uint32_t    collecting_timeout_count;  /* increments each time 10ms fires short */
extern volatile uint16_t    last_timeout_joints;       /* rx_joints_received at last timeout    */

/* Velocity command (user / operator) */
extern volatile float       vel_cmd[3];

/* MPU6050 */
extern MPU6050_Data_t       mpu;
extern volatile uint8_t     mpu_update_flag;

/* Protection (định nghĩa trong utils.c, đọc được trong debugger) */
extern Protection_Stats_t     prot_stats;        /* bộ đếm lỗi mọi lớp        */
extern Protection_Slew_t      action_slew;       /* slew-limit cho action     */
extern Protection_Watchdog_t  jetson_action_wd;  /* watchdog link action      */
extern volatile uint8_t       jetson_action_link_ok; /* 1=đang nhận, 0=mất    */
extern volatile uint32_t      dbg_action_timeout_ms;  /* tick lúc action wd expire */
extern volatile uint32_t      dbg_action_full_sent;   /* số lần gửi đủ 12 action  */
extern volatile uint32_t      dbg_action_tx_timeout;  /* số lần bị kill lúc 5ms   */

/* NRF action gate (định nghĩa trong main.c) */
extern uint8_t nrf_action;   /* 0=block, 1=cho phép bắt đầu arming */

/* Init-done handshake state (định nghĩa trong utils.c) */
extern volatile uint16_t      init_done_mask;    /* bit n = node n đã init xong */
extern volatile uint8_t       all_nodes_ready;   /* 1 = đủ 12 node -> cho Jetson */

/* Obs thiếu-khớp state (định nghĩa trong utils.c, đọc trong debugger) */
extern volatile uint8_t       obs_locked;        /* 1 = đang khóa, không nhận lệnh */
extern volatile uint8_t       obs_bad_streak;    /* số vòng thiếu 4+ liên tiếp     */
extern volatile uint8_t       obs_good_streak;   /* số vòng tốt liên tiếp          */
extern volatile uint32_t      obs_severe_count;  /* tổng số vòng thiếu 4+          */
extern volatile uint16_t      last_severe_joints;/* bitmask khớp nhận được lúc severe */

/* Falling state (định nghĩa trong utils.c, đọc trong debugger) */
extern volatile uint8_t       robot_falling;     /* 1 = đang ngã (đã báo node)     */

/* Arming state (cổng ổn định 5s; định nghĩa trong utils.c) */
extern volatile uint8_t       action_armed;      /* 1 = đã ổn định, được forward action */

/* State machine */
typedef enum {
    STATE_IDLE = 0,     /* waiting for Jetson GPIO trigger                  */
    STATE_COLLECTING,   /* SYNC sent, waiting for all 12 CAN replies        */
    STATE_OBS_READY,    /* obs complete → copy to SPI buf, raise PD5        */
} MainState_t;

extern volatile MainState_t main_state;

/* CAN TX state */
typedef enum {
    CAN_IDLE,
    CAN_SENDING_SYNC,
    CAN_SENDING_ACTIONS
} CAN_TxState_t;

extern volatile CAN_TxState_t can_tx_state;

/* ============================== Trạng thái an toàn thống nhất ==============================
 *  Gộp mọi nguồn lỗi thành MỘT cấp độ. Mọi nơi quyết định "có forward action
 *  không" chỉ nhìn vào đây -> không bao giờ mâu thuẫn (luôn lấy mức nguy hiểm nhất).
 *
 *    SAFE_RUN      : mọi thứ ổn -> forward action bình thường.
 *    SAFE_HOLD     : đứng yên, KHÔNG forward lệnh mới, motor giữ tư thế (KHÔNG cắt điện).
 *    SAFE_PROTECT  : (để dành) hạ người/thu gọn có kiểm soát — IMU sắp ngã.
 *    SAFE_SHUTDOWN : (để dành) cắt FET có kiểm soát — chỉ lỗi pin/nhiệt.
 *
 *  Hiện các nguồn đẩy vào: chưa đủ node (HOLD), mất khớp kéo dài (HOLD).
 *  Sẽ thêm: bus-off, IMU ngã, BMS — khi triển khai L3/L4.                       */
typedef enum {
    SAFE_RUN = 0,
    SAFE_HOLD,
    SAFE_PROTECT,
    SAFE_SHUTDOWN
} RobotSafeLevel_t;

/* Tính cấp độ an toàn hiện tại từ tất cả các nguồn lỗi (mức nguy hiểm nhất). */
RobotSafeLevel_t Robot_SafeLevel(void);

/* ============================== Function prototypes ============================== */

/* CAN */
uint8_t CAN_SendSync(void);   /* returns 1 if SYNC queued OK, 0 if mailbox full */
void    CAN_SendAction(uint8_t joint_index, float action_value);
uint8_t CAN_SendCheckInitDone(void);  /* broadcast CHECK_INIT_DONE; 1 nếu vào mailbox */
uint8_t CAN_SendFalling(void);        /* broadcast CAN_ID_FALLING (150) cho các node  */

/* SPI */
void PublishObsToSpi(void);
void ProcessSpiFrame(void);

/* HAL Callbacks (called from main.c via HAL weak overrides) */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan);

#endif /* UTILS_H */
