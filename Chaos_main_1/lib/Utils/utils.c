/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : utils.c
  * @brief          : Application-level utility implementations
  *                   (CAN, SPI, GPIO callbacks, Jetson watchdog)
  ******************************************************************************
  */
/* USER CODE END Header */

#include "utils.h"

/* ============================== Global state definitions ============================== */

/*  =========================== SPI buffers (46 float = 184 bytes each) =========================== */
float          spi_tx_buffer[SPI_PAYLOAD_FLOATS] = {0};
float          spi_rx_buffer[SPI_PAYLOAD_FLOATS] = {0};
volatile int   spi_callback = 0;

/*  =========================== CAN TX =========================== */
CAN_TxHeaderTypeDef  TxHeader       = {0};
uint8_t              can_tx_data[8] = {0};
uint32_t             txMailbox      = 0;
volatile uint8_t     action_idx     = 0;
volatile uint8_t     sync_pending   = 0;
volatile uint8_t     action_pending = 0;

volatile CAN_TxState_t can_tx_state = CAN_IDLE;

/*  =========================== CAN RX joint data ===========================
 *   [0 ..11] = joint position
 *   [12..23] = joint velocity
 */
CAN_RxHeaderTypeDef  RxHeader              = {0};
float                can_rx_data[NUM_JOINTS * 2] = {0};
volatile uint16_t    rx_joints_received    = 0;
volatile int         rx_ok                 = 0;

/*  =========================== Init-done handshake ===========================
 *  init_done_mask  : bit n = node n đã gửi CAN_ID_INIT_DONE (101+n).
 *  all_nodes_ready : 1 khi đủ 12 bit -> dừng TIM5, cho phép giao tiếp Jetson.
 *  Khởi tạo 0 trong main.c trước khi start CAN + TIM5.                        */
volatile uint16_t    init_done_mask  = 0;
volatile uint8_t     all_nodes_ready = 0;

/*  =========================== Obs thiếu-khớp policy ===========================
 *  obs_locked      : 1 = đang khóa (thiếu 4+ kéo dài) -> ProcessSpiFrame bỏ lệnh.
 *  obs_bad_streak  : số vòng thiếu >OBS_MAX_MISSING liên tiếp.
 *  obs_good_streak : số vòng thiếu <=OBS_MAX_MISSING liên tiếp.
 *  obs_severe_count/last_severe_joints : quan trắc (debugger).                 */
volatile uint8_t     obs_locked         = 0;
volatile uint8_t     obs_bad_streak     = 0;
volatile uint8_t     obs_good_streak    = 0;
volatile uint32_t    obs_severe_count   = 0;
volatile uint16_t    last_severe_joints = 0;

/*  =========================== Falling detection ===========================
 *  robot_falling      : 1 = đang ngã (đã/đang broadcast CAN_ID_FALLING).
 *  fall_tilt_streak   : số vòng nghiêng liên tiếp (vào trạng thái ngã khi đủ).
 *  fall_upright_streak: số vòng thẳng liên tiếp (thoát ngã khi đủ).            */
volatile uint32_t    dbg_action_full_sent  = 0;  /* số lần gửi đủ 12 action xuống motor */
volatile uint32_t    dbg_action_tx_timeout = 0;  /* số lần CAN action bị kill lúc 5ms   */
volatile uint8_t     robot_falling       = 0;
static   uint8_t     fall_tilt_streak    = 0;
static   uint8_t     fall_upright_streak = 0;
static   uint8_t     falling_tx_burst    = 0;   /* số vòng còn phải gửi 150 (cụm cạnh ngã) */

/*  =========================== Arming (cổng ổn định 5s) ===========================
 *  action_armed       : 1 = đã ổn định đủ STABILIZE_MS -> được forward action.
 *  stabilize_since_ms : mốc thời gian bắt đầu chuỗi ổn định (reset khi gián đoạn). */
volatile uint8_t     action_armed        = 0;
static   uint32_t    stabilize_since_ms  = 0;
static   float       prev_obs_pos[NUM_JOINTS] = {0};  /* vị trí khớp vòng trước (đo dao động) */

/*  =========================== GPIO shadow flags =========================== */
volatile uint8_t  jetson_out = 0;
volatile int      jetson_in  = 0;

/*  =========================== Timer tick counters (all timers at 1 ms) ===========================
 *  All three timers run at 1 kHz (ARR=9, PSC=8399, 84 MHz APB1 timer clock).
 *  Software counters provide the actual timeout durations.
 *
 *  tim2_tick: PD5 watchdog — stop+trigger at 10 (10 ms).
 *  tim3_tick: cycle timer  — action abort at 5 (5 ms), collecting check every tick.
 *  tim4_tick: EXTI gate   — clear gate at 15 (15 ms).
 */
static volatile uint8_t tim2_tick = 0;
static volatile uint8_t tim3_tick = 0;
static volatile uint8_t tim4_tick = 0;

/*  =========================== EXTI gate flag ===========================
 *  Set to 1 on a valid EXTI trigger; cleared by TIM4 ISR after 15 ms.
 *  Any EXTI arriving while this flag is set is silently discarded as a glitch.
 */
volatile uint8_t exti_gate_active = 0;

/*  =========================== STATE_COLLECTING timeout ===========================
 *  HAL_GetTick() snapshot taken at CAN_SendSync time.
 *  TIM3 callback checks (HAL_GetTick() - sync_start_ms) >= COLLECTING_TIMEOUT_MS.
 */
volatile uint32_t sync_start_ms = 0;

/*  =========================== SPI DMA error flag ===========================
 *  Set in HAL_SPI_TxRxCpltCallback if HAL_SPI_TransmitReceive_DMA fails.
 *  Cleared and retried from the main loop.
 */
volatile uint8_t  spi_dma_error = 0;

/* Mốc thời gian lần SPI TxRx hoàn tất gần nhất (cập nhật trong callback) */
volatile uint32_t spi_last_ms = 0;

/*  =========================== CAN error / bus-off counters ===========================
 *  can_error_count: total error interrupts (watch in debugger).
 *  can_busoff: 1 while bus-off recovery is in progress.
 */
volatile uint32_t can_error_count = 0;
volatile uint8_t  can_busoff      = 0;

/*  =========================== Collecting timeout debug ===========================
 *  collecting_timeout_count : number of times the 10 ms window expired before all
 *                             12 joints replied.  Non-zero means some joints are
 *                             consistently slow or missing.
 *  last_timeout_joints      : snapshot of rx_joints_received at the moment the last
 *                             timeout fired.  Missing bits = joints that did not reply.
 *                             e.g. 0x0FFB = bit 2 missing = joint 2 timed out.
 *                             Complement: (~last_timeout_joints & 0x0FFF).
 */
volatile uint32_t collecting_timeout_count = 0;
volatile uint16_t last_timeout_joints      = 0;

/*  =========================== Prev actions shadow buffer ===========================
 * Keeps a copy of the last 12 actions received from Jetson.
 * Written by ProcessSpiFrame on every CMD_ACTION frame.
 * Read by PublishObsToSpi to fill spi_tx_buffer[33..44].
 * Separate from spi_rx_buffer so CAN_SendSync's memset cannot corrupt it.
 */
static float prev_actions[NUM_JOINTS] = {0};

/*  =========================== Velocity command (user/operator) ===========================
 *   vel_cmd[0] = vx    (m/s)
 *   vel_cmd[1] = vy    (m/s)
 *   vel_cmd[2] = vyaw  (rad/s)
 */
volatile float vel_cmd[3] = {0.0f, 0.0f, 0.0f};

/*  =========================== MPU6050 =========================== */
MPU6050_Data_t  mpu            = {0};
volatile uint8_t mpu_update_flag = 0;

/*  =========================== Protection ===========================
 *  prot_stats         : bộ đếm lỗi tất cả các lớp (soi trong debugger).
 *  action_slew        : trạng thái slew-limit cho 12 action (per-joint prev).
 *  jetson_action_wd   : watchdog phát hiện mất lệnh từ Jetson.
 *  jetson_action_link_ok : cờ trạng thái link action (cập nhật ở main loop).
 *  Khởi tạo trong main.c (USER CODE BEGIN 2) trước khi start SPI/CAN.        */
Protection_Stats_t     prot_stats        = {0};
Protection_Slew_t      action_slew       = {0};
Protection_Watchdog_t  jetson_action_wd  = {0};
volatile uint8_t       jetson_action_link_ok = 0;
volatile uint32_t      dbg_action_timeout_ms = 0;  /* tick lúc action watchdog vừa expire (cạnh xuống) */

/*  obs_finite: obs phải TRUNG THỰC -> KHÔNG clamp.
 *  Chỉ chặn số rác: NaN/Inf -> 0 (số vô nghĩa, không phải số đo thật) + đếm.
 *  Giá trị hợp lệ (kể cả ngoài tầm) được giữ NGUYÊN để policy thấy đúng thực tại. */
static inline float obs_finite(float v)
{
    if (!Protection_IsFinite(v)) { prot_stats.obs_nan++; return 0.0f; }
    return v;
}

/*  =========================== State machine =========================== */
volatile MainState_t main_state = STATE_IDLE;

/* ============================== Private helpers (forward decls) ============================== */
static uint8_t CAN_TxCompleteHandler(void);
static void    CAN_KickNext(void);
static void    CAN_TxCompleteCommon(void);
static void    ObsFinishCycle(void);
static void    CheckFalling(void);

/* ============================== CAN ============================== */

/*  =========================== CAN: send SYNC broadcast ===========================
 * Queues a 1-byte frame with ID CAN_ID_SYNC.
 * All motor nodes respond with pos+vel frames after receiving this.
 * MPU6050 is read here (blocking, ~200 µs) for a time-aligned IMU snapshot.
 */
uint8_t CAN_SendSync(void)
{
    TxHeader.StdId              = CAN_ID_SYNC;
    TxHeader.ExtId              = 0;
    TxHeader.IDE                = CAN_ID_STD;
    TxHeader.RTR                = CAN_RTR_DATA;
    TxHeader.DLC                = 1;
    TxHeader.TransmitGlobalTime = DISABLE;
    can_tx_data[0]              = 0x01;

    MPU6050_Update(&mpu);

    /* Phát hiện ngã từ projected gravity vừa cập nhật; nếu ngã -> broadcast 150. */
    CheckFalling();

    if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, can_tx_data, &txMailbox) != HAL_OK)
        return 0;   /* mailbox full — caller quyết định retry hay drop */

    can_tx_state       = CAN_SENDING_SYNC;
    rx_joints_received = 0;
    sync_start_ms      = HAL_GetTick();

    memset(spi_tx_buffer, 0, sizeof(spi_tx_buffer));
    memcpy(&spi_tx_buffer[SPI_IDX_ACTIONS], prev_actions, NUM_JOINTS * sizeof(float));

    main_state = STATE_COLLECTING;
    return 1;
}

/*  =========================== CAN: forward action to a single motor node ===========================
 *
 * Frame format (4 bytes = 1 float):
 *   StdId = CAN_ACTION_ID_BASE + joint_index
 *   DLC   = 4
 *   bytes [0:3] = action value (float, little-endian)
 */
void CAN_SendAction(uint8_t joint_index, float action_value)
{
    uint8_t raw[4];
    memcpy(raw, &action_value, sizeof(float));

    TxHeader.StdId              = CAN_ACTION_ID_BASE + joint_index;
    TxHeader.ExtId              = 0;
    TxHeader.IDE                = CAN_ID_STD;
    TxHeader.RTR                = CAN_RTR_DATA;
    TxHeader.DLC                = 4;
    TxHeader.TransmitGlobalTime = DISABLE;

    /* Non-blocking: silently drops frame if all mailboxes are full */
    HAL_CAN_AddTxMessage(&hcan2, &TxHeader, raw, &txMailbox);
}

/*  =========================== CAN: broadcast CHECK_INIT_DONE ===========================
 * Hỏi tất cả node "đã init/homing xong chưa". Node nào xong sẽ đáp lại bằng
 * CAN_ID_INIT_DONE (101+n). Dùng header CỤC BỘ để không đụng TxHeader của
 * state machine SYNC/action. Trả 1 nếu vào được mailbox.
 */
uint8_t CAN_SendCheckInitDone(void)
{
    CAN_TxHeaderTypeDef h = {0};
    uint8_t  d[1] = {0};        /* payload rác — chỉ ID mới mang thông tin */
    uint32_t mb;

    h.StdId              = CAN_ID_CHECK_INIT_DONE;
    h.ExtId              = 0;
    h.IDE                = CAN_ID_STD;
    h.RTR                = CAN_RTR_DATA;
    h.DLC                = 1;
    h.TransmitGlobalTime = DISABLE;

    return (HAL_CAN_AddTxMessage(&hcan2, &h, d, &mb) == HAL_OK) ? 1 : 0;
}

/*  =========================== CAN: broadcast FALLING ===========================
 * Báo mọi node "robot đang ngã" -> node tự vào trạng thái falling riêng.
 * Header cục bộ (không đụng state machine SYNC/action). Trả 1 nếu vào mailbox.
 */
uint8_t CAN_SendFalling(void)
{
    CAN_TxHeaderTypeDef h = {0};
    uint8_t  d[1] = {0};        /* payload rác — chỉ ID 150 mới mang thông tin */
    uint32_t mb;

    h.StdId              = CAN_ID_FALLING;
    h.ExtId              = 0;
    h.IDE                = CAN_ID_STD;
    h.RTR                = CAN_RTR_DATA;
    h.DLC                = 1;
    h.TransmitGlobalTime = DISABLE;

    return (HAL_CAN_AddTxMessage(&hcan2, &h, d, &mb) == HAL_OK) ? 1 : 0;
}

/*  =========================== CAN TX chaining (private) =========================== */

static uint8_t CAN_TxCompleteHandler(void)
{
    if (action_idx < NUM_JOINTS)
    {
        /* Dùng prev_actions thay vì spi_rx_buffer — prev_actions là stable copy,
         * không bị DMA overwrite nếu Jetson kick Transaction tiếp theo sớm. */
        CAN_SendAction(action_idx, prev_actions[action_idx]);
        action_idx++;
        return 0;
    }
    dbg_action_full_sent++;
    return 1;  /* all 12 joints sent */
}

static void CAN_KickNext(void)
{
    if (sync_pending)
    {
        if (CAN_SendSync())
            sync_pending = 0;   /* chỉ clear nếu SYNC thực sự vào mailbox */
        /* nếu fail: sync_pending còn đó, TX complete tiếp theo sẽ retry */
    }
    else if (action_pending)
    {
        action_pending = 0;
        action_idx     = 0;
        can_tx_state   = CAN_SENDING_ACTIONS;
        CAN_TxCompleteHandler();
        CAN_TxCompleteHandler();
        CAN_TxCompleteHandler();
    }
    else
    {
        can_tx_state = CAN_IDLE;
    }
}

static void CAN_TxCompleteCommon(void)
{
    if (can_tx_state == CAN_SENDING_SYNC)
        CAN_KickNext();
    else if (can_tx_state == CAN_SENDING_ACTIONS)
    {
        if (CAN_TxCompleteHandler())   /* returns 1 when all 12 done */
            CAN_KickNext();
    }
    else if (sync_pending)
    {
        /* Orphaned TX complete from an action chain aborted by TIM3 at 5 ms.
         * can_tx_state is already CAN_IDLE; a mailbox just freed — retry SYNC. */
        if (CAN_SendSync()) sync_pending = 0;
    }
}

/* ============================== SPI ============================== */

/*  =========================== Copy obs data → SPI TX buffer, then raise PD5 ===========================
 *
 *  TX buffer filled here (46 floats, [45]=STATUS):
 *   [0  –  2] base angular velocity  ← MPU6050 angvel_x/y/z  (rad/s)
 *   [3  –  5] projected gravity      ← MPU6050 grav_x/y/z    (normalised)
 *   [6  –  8] velocity command       ← vel_cmd[0..2], set by user/operator
 *   [9  – 20] joint position         ← CAN feedback pos joint 0..11  (rad)
 *   [21 – 32] joint velocity         ← CAN feedback vel joint 0..11  (rad/s)
 *   [33 – 44] prev actions           ← last CMD_ACTION echoed back
 */
void PublishObsToSpi(void)
{
    __disable_irq();

    /* OBS TRUNG THỰC: chỉ chặn NaN (obs_finite), KHÔNG clamp giá trị thật. */
    spi_tx_buffer[SPI_IDX_ANG_VEL + 0] = obs_finite(mpu.angvel_x);
    spi_tx_buffer[SPI_IDX_ANG_VEL + 1] = obs_finite(mpu.angvel_y);
    spi_tx_buffer[SPI_IDX_ANG_VEL + 2] = obs_finite(mpu.angvel_z);

    spi_tx_buffer[SPI_IDX_GRAV + 0]    = obs_finite(mpu.grav_x);
    spi_tx_buffer[SPI_IDX_GRAV + 1]    = obs_finite(mpu.grav_y);
    spi_tx_buffer[SPI_IDX_GRAV + 2]    = obs_finite(mpu.grav_z);

    spi_tx_buffer[SPI_IDX_CMD + 0]     = obs_finite(vel_cmd[0]); /* vx   */
    spi_tx_buffer[SPI_IDX_CMD + 1]     = obs_finite(vel_cmd[1]); /* vy   */
    spi_tx_buffer[SPI_IDX_CMD + 2]     = obs_finite(vel_cmd[2]); /* vyaw */

    memcpy(&spi_tx_buffer[SPI_IDX_JOINT_POS], &can_rx_data[0],
           NUM_JOINTS * sizeof(float));
    memcpy(&spi_tx_buffer[SPI_IDX_JOINT_VEL], &can_rx_data[NUM_JOINTS],
           NUM_JOINTS * sizeof(float));
    memcpy(&spi_tx_buffer[SPI_IDX_ACTIONS],    prev_actions,
           NUM_JOINTS * sizeof(float));

    /* STATUS / OK-id: READY khi đã homing xong (đủ node) VÀ đứng thẳng (!ngã);
     * ngược lại WAIT -> Jetson giữ, chỉ chạy lại khi thấy READY (và tự nó cũng
     * đã phát hiện ngã trước đó). */
    spi_tx_buffer[SPI_IDX_STATUS] = (all_nodes_ready && !robot_falling)
                                    ? STM_STATUS_READY : STM_STATUS_WAIT;

    __enable_irq();

    HAL_GPIO_WritePin(JETSON_OUT_PORT, JETSON_OUT_PIN, GPIO_PIN_SET);
    jetson_out = 1;

#if !TEST_NO_TIM
    /* Start PD5 watchdog: TIM2 forces PD5 LOW after 10 ms if SPI T1 never fires */
    HAL_TIM_Base_Stop_IT(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
    tim2_tick = 0;
    HAL_TIM_Base_Start_IT(&htim2);
#endif

#if TEST
    jetson_in = 0;
#endif
}

/*  =========================== Kết thúc 1 chu kỳ thu thập obs ===========================
 *
 * Gọi khi chu kỳ collecting kết thúc (đủ 12 khớp HOẶC timeout). 2 pha:
 *
 *  PHA CHƯA SẴN SÀNG (init / đang ngã / recovery — chưa đủ node hoặc đang nghiêng):
 *     LUÔN gửi obs với STATUS=WAIT để Jetson nắm tình hình (IMU + status).
 *     Không áp chính sách khóa, disarm, reset đồng hồ ổn định.
 *
 *  PHA SẴN SÀNG (đã homing xong + đứng thẳng):
 *     - thiếu <= OBS_MAX_MISSING : gửi obs (STATUS=READY); good_streak++; đủ -> mở khóa.
 *     - thiếu >  OBS_MAX_MISSING : KHÔNG gửi; bad_streak++; đủ OBS_SEVERE_ENTER -> khóa.
 *     - arming: RUN + obs đủ + action tươi + ÍT DAO ĐỘNG liên tục 5s -> cho xuất action.
 *
 * Không cắt FET ở bất kỳ nhánh nào — chỉ giữ nguyên.
 */
static void ObsFinishCycle(void)
{
    uint8_t  received = (uint8_t)__builtin_popcount(rx_joints_received & 0x0FFFu);
    uint8_t  missing  = (uint8_t)(NUM_JOINTS - received);
    uint32_t now      = HAL_GetTick();

    /* ---- "ít dao động": mọi khớp đổi < STABLE_DELTA_DEG so với vòng trước? ---- */
    uint8_t settled = 1;
    for (uint32_t i = 0; i < NUM_JOINTS; i++)
    {
        float d = can_rx_data[i] - prev_obs_pos[i];
        if (d < 0) d = -d;
        if (d > STABLE_DELTA_DEG) settled = 0;
        prev_obs_pos[i] = can_rx_data[i];
    }

    /* ===== PHA CHƯA SẴN SÀNG (init / ngã / recovery) ===== */
    if (!all_nodes_ready || robot_falling)
    {
        obs_bad_streak     = 0;
        obs_good_streak    = 0;
        obs_locked         = 0;
        action_armed       = 0;
        stabilize_since_ms = now;           /* chưa sẵn sàng -> đồng hồ ổn định = 0 */
        PublishObsToSpi();                  /* gửi obs + STATUS=WAIT cho Jetson    */
        return;
    }

    /* ===== PHA SẴN SÀNG (vận hành) — chính sách thiếu khớp ===== */
    if (missing <= OBS_MAX_MISSING)
    {
        obs_bad_streak = 0;
        if (obs_good_streak < 0xFF) obs_good_streak++;
        if (obs_locked && obs_good_streak >= OBS_RECOVER_GOOD)
            obs_locked = 0;                 /* đủ 5 vòng tốt -> mở khóa            */
        PublishObsToSpi();                  /* STATUS=READY                       */
    }
    else
    {
        obs_good_streak = 0;
        if (obs_bad_streak < 0xFF) obs_bad_streak++;
        obs_severe_count++;
        last_severe_joints = rx_joints_received;
        if (obs_bad_streak >= OBS_SEVERE_ENTER)
            obs_locked = 1;                 /* xấu kéo dài -> khóa, giữ nguyên     */

        /* Mất dữ liệu quá lâu (~3s) -> coi như node chết, reset về giai đoạn init. */
        if (obs_bad_streak >= OBS_REINIT_CYCLES)
        {
            init_done_mask     = 0;
            all_nodes_ready    = 0;
            obs_bad_streak     = 0;
            obs_good_streak    = 0;
            obs_locked         = 0;
            action_armed       = 0;
            stabilize_since_ms = now;
            HAL_TIM_Base_Stop_IT(&htim5);
            __HAL_TIM_SET_COUNTER(&htim5, 0);
            __HAL_TIM_CLEAR_IT(&htim5, TIM_IT_UPDATE);
            HAL_TIM_Base_Start_IT(&htim5);  /* bắt đầu seek node lại */
        }
        /* KHÔNG publish vòng này -> motor tự giữ */
    }

    /* ---- Cổng ổn định (arming) ----
     * Arm sau khi ổn định 5s liên tục.
     * Một khi đã arm: giữ nguyên cho đến khi robot ngã, nrf_action=0, hoặc init lại. */
    if (robot_falling || !nrf_action)
    {
        action_armed       = 0;
        stabilize_since_ms = now;
    }
    else if (!action_armed)
    {
        uint8_t action_fresh = !Protection_WatchdogExpired(&jetson_action_wd, now);
        uint8_t stable       = (Robot_SafeLevel() == SAFE_RUN)
                            && (missing <= OBS_MAX_MISSING)
                            && action_fresh
                            && settled;
        if (!stable)
            stabilize_since_ms = now;
        else if ((uint32_t)(now - stabilize_since_ms) >= STABILIZE_MS)
            action_armed = 1;
    }
}

/*  =========================== Trạng thái an toàn thống nhất ===========================
 *
 * Gộp mọi nguồn lỗi thành 1 cấp độ (lấy mức nguy hiểm nhất). Tất cả các cổng
 * quyết định "có forward action không" chỉ gọi hàm này.
 *
 * Thứ tự ưu tiên (nguy hiểm nhất trước):
 *   - đang ngã (robot_falling=1)            -> SAFE_PROTECT
 *   - chưa đủ 12 node (all_nodes_ready=0)   -> SAFE_HOLD
 *   - mất khớp kéo dài (obs_locked=1)        -> SAFE_HOLD
 *   - CAN bus-off (can_busoff=1)             -> SAFE_HOLD
 * (Để dành: BMS OT/OC -> SAFE_SHUTDOWN.)
 */
RobotSafeLevel_t Robot_SafeLevel(void)
{
    if (robot_falling)    return SAFE_PROTECT;   /* nghiêm trọng hơn HOLD       */
    if (!all_nodes_ready) return SAFE_HOLD;
    if (obs_locked)       return SAFE_HOLD;
    if (can_busoff)       return SAFE_HOLD;      /* bus-off -> giữ nguyên       */
    return SAFE_RUN;
}

/*  =========================== Phát hiện ngã (CheckFalling) ===========================
 *
 * Gọi mỗi vòng (trong CAN_SendSync, sau khi cập nhật IMU). KHỚP công thức Jetson:
 *   tilt = atan2(hypot(gx,gy), -gz) > 20°  (sign-aware) -> ngã.
 *   tương đương: gz>=0 (gần lật) HOẶC h^2 > gz^2 * tan^2(20°) khi gz<0.
 *   gz = trục thẳng đứng (đứng thẳng gz≈-1); gx,gy = nằm ngang.
 * Có chống nhiễu: phải nghiêng FALL_ENTER_CYCLES vòng liền mới báo ngã;
 * thẳng lại FALL_EXIT_CYCLES vòng liền mới hết. Khi đang ngã -> broadcast 150.
 *
 * LƯU Ý: giả định gz là trục thẳng đứng. Nếu IMU gắn trục khác, đổi gz tương ứng.
 */
static void CheckFalling(void)
{
    float gx = mpu.grav_x, gy = mpu.grav_y, gz = mpu.grav_z;

    /* IMU lỗi (NaN/Inf) -> không cập nhật trạng thái ngã (giữ nguyên). */
    if (!Protection_IsFinite(gx) || !Protection_IsFinite(gy) || !Protection_IsFinite(gz))
        return;

    /* KHỚP Jetson: tilt = atan2(hypot(gx,gy), -gz) > 20°  (SIGN-AWARE theo gz).
     *   gz >= 0  -> nghiêng >= 90° (gần lật/úp) -> chắc chắn ngã.
     *   gz <  0  -> ngã khi h^2 > gz^2 * tan^2(20°).
     * Dùng gz CÓ DẤU để lật ngửa (gz>0) cũng bị coi là ngã (trước đây dùng gz^2 thì sót). */
    float h2 = gx * gx + gy * gy;     /* phần nằm ngang */

    uint8_t tilted = (gz >= 0.0f) || (h2 > (gz * gz) * FALL_TILT_TAN2);

    if (tilted)
    {
        fall_upright_streak = 0;
        if (fall_tilt_streak < 0xFF) fall_tilt_streak++;

        if (fall_tilt_streak >= FALL_ENTER_CYCLES && !robot_falling)
        {
            /* CẠNH LÊN: bắt đầu ngã -> "quay về dạng init".
             * Node sẽ homing khi nhận 150; STM reset handshake + bật lại TIM5
             * để poll homing. Recovery cần: đủ 12 init + đứng thẳng trở lại.   */
            robot_falling    = 1;
            all_nodes_ready  = 0;
            init_done_mask   = 0;
            action_armed     = 0;
            falling_tx_burst = FALLING_BURST_CYCLES;   /* bắt đầu cụm gửi 150 */

            HAL_TIM_Base_Stop_IT(&htim5);
            __HAL_TIM_SET_COUNTER(&htim5, 0);
            __HAL_TIM_CLEAR_IT(&htim5, TIM_IT_UPDATE);
            HAL_TIM_Base_Start_IT(&htim5);   /* poll CHECK_INIT_DONE lại */
        }
    }
    else
    {
        fall_tilt_streak = 0;
        if (fall_upright_streak < 0xFF) fall_upright_streak++;
        if (robot_falling && fall_upright_streak >= FALL_EXIT_CYCLES)
            robot_falling = 0;              /* đứng thẳng ổn định -> hết "ngã"     */
    }

    /* Gửi 150 theo CỤM ở cạnh ngã (không liên tục) -> node reset+homing 1 lần,
     * không bị reset lặp khi robot còn nằm. CAN tin cậy nên cụm 10 frame là đủ. */
    if (falling_tx_burst > 0)
    {
        CAN_SendFalling();
        falling_tx_burst--;
    }
}

/*  =========================== Process a completed SPI RX frame from Jetson ===========================
 *
 * Called from HAL_SPI_TxRxCpltCallback after each 184-byte DMA transfer.
 *
 * spi_rx_buffer[0] == 2.0f  →  CMD_ACTION
 *   spi_rx_buffer[1..12] = 12 joint actions.
 *   Forward each action via CAN immediately.
 *   Echo actions into spi_tx_buffer[33..44] for next obs frame.
 */
void ProcessSpiFrame(void)
{
    /* CỔNG AN TOÀN THỐNG NHẤT: chỉ forward action khi RUN.
     * Gồm: chưa đủ node, mất khớp kéo dài (và sau này: bus-off, ngã, BMS).
     * Khi HOLD -> bỏ lệnh mới, motor tự giữ tư thế (không cắt điện).          */
    if (Robot_SafeLevel() != SAFE_RUN) return;

    /* Lớp framing tối thiểu hiện có: chỉ chấp nhận đúng frame type CMD_ACTION.
     * (Magic/CRC/sequence đầy đủ cần phối hợp layout với Jetson — làm sau.)   */
    if (spi_rx_buffer[0] != SPI_CMD_ACTION) return;

    /* ===== LỚP BẢO VỆ ACTION (chạy trong SPI DMA ISR, ~vài µs) =====
     *  4a NaN/Inf  + 4b clamp theo dải VỊ TRÍ từng khớp (deg).
     *  Fail-safe: dùng prev_actions làm 'hold' -> NaN thì GIỮ lệnh hợp lệ cũ,
     *  không nhảy về 0 (an toàn cho khớp dải bất đối xứng như gối [-100,0]).  */
    float clean[NUM_JOINTS];
    Protection_ValidateActions(&spi_rx_buffer[1], clean, prev_actions, &prot_stats);

#if PROT_ENABLE_ACTION_SLEW
    /* 4c slew: giới hạn tốc độ đổi lệnh vị trí (tùy chọn, xem utils.h). */
    Protection_ApplyActionSlew(&action_slew, clean, &prot_stats);
#endif

    /* Save actions (đã làm sạch) to shadow buffer; survives memset in CAN_SendSync */
    memcpy(prev_actions, clean, NUM_JOINTS * sizeof(float));

    /* Lớp 5: vừa nhận 1 khung action hợp lệ -> feed watchdog link Jetson.
     * (Feed cả khi chưa armed để đo "action ổn định" trong giai đoạn chờ 5s.)   */
    Protection_WatchdogFeed(&jetson_action_wd, HAL_GetTick());

    /* CỔNG ARMING: chưa ổn định đủ 5s -> đã nhận/ghi nhận action nhưng CHƯA forward
     * xuống node (đứng yên chờ ổn định). Áp dụng sau init và sau khi đứng dậy.   */
    if (!action_armed) return;

    if (can_tx_state == CAN_IDLE)
    {
        action_idx   = 0;   /* reset chỉ khi bắt đầu batch ngay lập tức */
        can_tx_state = CAN_SENDING_ACTIONS;
        CAN_TxCompleteHandler();
        CAN_TxCompleteHandler();
        CAN_TxCompleteHandler();
    }
    else
    {
        /* Không reset action_idx ở đây — ongoing chain đang dùng nó.
         * CAN_KickNext sẽ reset khi consume action_pending.           */
        action_pending = 1;
    }
}

/* ============================== HAL Callbacks ============================== */

/*  =========================== SPI3 DMA TxRx Complete Callback ===========================
 *
 * Fires after each full 184-byte transfer (Jetson clocked out / in 46 floats).
 *  1. Lower PD5 (Jetson has consumed the obs frame, or this was an action).
 *  2. Parse the received frame.
 *  3. Re-arm DMA for the next transaction.
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    spi_callback++;
    if (hspi->Instance != SPI3) return;

    spi_last_ms = HAL_GetTick();   /* SPI còn sống -> reset watchdog SPI */

    HAL_GPIO_WritePin(JETSON_OUT_PORT, JETSON_OUT_PIN, GPIO_PIN_RESET);
    jetson_out = 0;
#if !TEST_NO_TIM
    /* Normal SPI completion — disarm PD5 watchdog */
    HAL_TIM_Base_Stop_IT(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    tim2_tick = 0;
#endif

    ProcessSpiFrame();

    if (HAL_SPI_TransmitReceive_DMA(&hspi3,
            (uint8_t *)spi_tx_buffer,
            (uint8_t *)spi_rx_buffer,
            SPI_PAYLOAD_FLOATS * sizeof(float)) != HAL_OK)
    {
        spi_dma_error = 1;   /* main loop will retry */
    }
}

/*  =========================== CAN RX FIFO 0 Pending Callback ===========================
 *
 * Accepts frames from motor nodes (IDs 0x001–0x00C) only while STATE_COLLECTING.
 *
 * Frame format from each node (8 bytes):
 *   bytes [0:3] = position  (float32, little-endian)
 *   bytes [4:7] = velocity  (float32, little-endian)
 *
 * Joint index = StdId - CAN_OBS_ID_BASE  (0x001→0 … 0x00C→11)
 *
 * When all 12 joints have replied (rx_joints_received == 0x0FFF),
 * transitions to STATE_OBS_READY and publishes to SPI.
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL_GPIO_TogglePin(LED_PORT, LED_L3);

    uint8_t raw[8];

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, raw) != HAL_OK)
            break;

        can_busoff = 0;   /* nhận được frame -> bus đã sống lại, tự xóa cờ HOLD */

        /* Init-done handshake: node n báo đã homing xong (ID 101..112).
         * Xử lý TRƯỚC range guard feedback (0x001..0x00C).                    */
        if (RxHeader.StdId >= CAN_ID_INIT_DONE_BASE &&
            RxHeader.StdId <= CAN_ID_INIT_DONE_MAX)
        {
            uint8_t node = (uint8_t)(RxHeader.StdId - CAN_ID_INIT_DONE_BASE);
            init_done_mask |= (uint16_t)(1u << node);

            if (init_done_mask == INIT_DONE_ALL_MASK && !all_nodes_ready)
            {
                all_nodes_ready = 1;
                HAL_TIM_Base_Stop_IT(&htim5);   /* đủ 12 node -> ngừng hỏi */
            }
            continue;
        }

        /* SW range guard: only 0x001–0x00C are motor feedback frames */
        if (RxHeader.StdId < CAN_OBS_ID_BASE || RxHeader.StdId > CAN_OBS_ID_MAX)
            continue;

        if (main_state != STATE_COLLECTING) continue;
        if (RxHeader.DLC < 8)              continue;  /* malformed — need 2 floats */

        rx_ok++;

        uint8_t joint = (uint8_t)(RxHeader.StdId - CAN_OBS_ID_BASE);  /* 0–11 */

        float pos, vel;
        memcpy(&pos, &raw[0], sizeof(float));
        memcpy(&vel, &raw[4], sizeof(float));

        /* FEEDBACK là OBS -> phải TRUNG THỰC: KHÔNG clamp giá trị thật.
         * Chỉ chặn số rác: NaN/Inf -> giữ giá trị cũ (không ghi đè) + đếm.
         * Ngoài tầm vật lý vẫn GIỮ NGUYÊN nhưng đếm để biết (cảnh báo).       */
        if (!Protection_IsFinite(pos)) { prot_stats.can_pos_bad++; pos = can_rx_data[joint]; }
        else if (!Protection_InRange(pos, PROT_JOINT_POS_MIN[joint],
                                          PROT_JOINT_POS_MAX[joint]))
            prot_stats.can_pos_bad++;

        if (!Protection_IsFinite(vel)) { prot_stats.can_vel_bad++; vel = can_rx_data[NUM_JOINTS + joint]; }
        else if (!Protection_InRange(vel, -PROT_JOINT_VEL_ABS_MAX,
                                           PROT_JOINT_VEL_ABS_MAX))
            prot_stats.can_vel_bad++;

        /*  can_rx_data layout:
         *   [0 ..11] positions   →  spi_tx_buffer[9 ..20]
         *   [12..23] velocities  →  spi_tx_buffer[21..32]
         */
        can_rx_data[joint]              = pos;
        can_rx_data[NUM_JOINTS + joint] = vel;

        rx_joints_received |= (1u << joint);

        if (rx_joints_received == 0x0FFFu)
        {
            main_state = STATE_OBS_READY;
            ObsFinishCycle();    /* đủ 12 khớp -> gửi obs (qua chính sách thiếu-khớp) */
            main_state = STATE_IDLE;
#if !TEST_NO_TIM
            /* Collecting done — stop cycle timer */
            HAL_TIM_Base_Stop_IT(&htim3);
            tim3_tick = 0;
#endif
        }
    }
}

/*  =========================== CAN TX Mailbox Callbacks ===========================  */
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) { CAN_TxCompleteCommon(); }
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) { CAN_TxCompleteCommon(); }
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) { CAN_TxCompleteCommon(); }

/*  =========================== EXTI Callback ===========================
 *
 * PD6  Rising edge → Jetson triggers start of collection.
 *   Sends CAN SYNC immediately if idle; queues if CAN TX is busy.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != JETSON_IN_PIN) return;

    /* KHÔNG còn chặn theo all_nodes_ready: chu kỳ obs (SYNC + đọc IMU + thu thập)
     * PHẢI chạy cả lúc init/ngã để đọc IMU (phát hiện đứng thẳng) và gửi obs+status
     * cho Jetson. Việc forward ACTION đã được chặn riêng bởi SafeLevel + armed. */

#if !TEST_NO_TIM
    /* TIM4 gate: reject EXTI arriving within 15 ms of the previous valid one */
    if (exti_gate_active) return;
#endif

    jetson_in++;

#if !TEST_NO_TIM
    /* Arm TIM4 gate — rejects glitches / spurious re-triggers for 15 ms */
    HAL_TIM_Base_Stop_IT(&htim4);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);
    tim4_tick = 0;
    exti_gate_active = 1;
    HAL_TIM_Base_Start_IT(&htim4);
#endif

    /* Safety: force PD5 LOW and disarm PD5 watchdog (new cycle starting) */
    HAL_GPIO_WritePin(JETSON_OUT_PORT, JETSON_OUT_PIN, GPIO_PIN_RESET);
    jetson_out = 0;
#if !TEST_NO_TIM
    HAL_TIM_Base_Stop_IT(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    /* Reset cycle timer TIM3 from the top */
    HAL_TIM_Base_Stop_IT(&htim3);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
    tim3_tick = 0;
    HAL_TIM_Base_Start_IT(&htim3);
#endif

    /* Discard stale pending flags from the previous cycle. */
    sync_pending   = 0;
    action_pending = 0;

    if (can_tx_state == CAN_IDLE)
    {
        if (!CAN_SendSync()) sync_pending = 1;
    }
    else
    {
        sync_pending = 1;
    }
}

/*  =========================== Timer Period Elapsed Callbacks ===========================
 *  All three timers run at 1 kHz (ARR=9, PSC=8399).  Software tick counters
 *  provide the actual timeout durations.
 *
 *  TIM2  PD5 watchdog — triggers at tim2_tick == 10 (10 ms).
 *        Started in PublishObsToSpi when PD5 goes HIGH.
 *        Stopped (tick reset) in HAL_SPI_TxRxCpltCallback on normal SPI completion.
 *        If SPI T1 never fires within 10 ms, forces PD5 LOW so Jetson is not stalled.
 *
 *  TIM3  Cycle timer — reset on every valid EXTI.
 *        tick  5: abort old action chain if running, then send queued SYNC.
 *        every tick: collecting timeout check — 10 ms from sync_start_ms (HAL_GetTick).
 *        Stopped when all 12 joints reply (CAN RX callback) or timeout fires here.
 *
 *  TIM4  EXTI gate — triggers at tim4_tick == 15 (15 ms).
 *        Started on valid EXTI.  Clears exti_gate_active so next EXTI is accepted.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* ---- TIM5: init-done handshake (5s) — luôn chạy, kể cả TEST_NO_TIM ----
     * Mỗi lần đáo hạn, nếu chưa đủ 12 node thì broadcast lại CHECK_INIT_DONE.
     * Khi đủ 12 node, CAN RX callback đã Stop TIM5 nên đây chỉ là phòng hờ.   */
    if (htim->Instance == TIM5)
    {
        if (!all_nodes_ready)
            CAN_SendCheckInitDone();
        return;
    }

#if TEST_NO_TIM
    (void)htim;
    return;
#endif
    /* ---- TIM2: PD5 watchdog (1 ms tick, triggers at 10 ms) ---- */
    if (htim->Instance == TIM2)
    {
        tim2_tick++;
        if (tim2_tick >= 10)
        {
            HAL_GPIO_WritePin(JETSON_OUT_PORT, JETSON_OUT_PIN, GPIO_PIN_RESET);
            jetson_out = 0;
            HAL_TIM_Base_Stop_IT(&htim2);
            __HAL_TIM_SET_COUNTER(&htim2, 0);
            tim2_tick = 0;
        }
        return;
    }

    /* ---- TIM3: cycle timer (1 ms ticks) ---- */
    if (htim->Instance == TIM3)
    {
        tim3_tick++;

        /* 5 ms: abort previous action chain if still running, then send SYNC */
        if (tim3_tick == 5)
        {
            if (can_tx_state == CAN_SENDING_ACTIONS)
            {
                dbg_action_tx_timeout++;
                can_tx_state   = CAN_IDLE;
                action_idx     = 0;
                action_pending = 0;
            }
            if (sync_pending && can_tx_state == CAN_IDLE)
            {
                if (CAN_SendSync()) sync_pending = 0;
                /* if fail: sync_pending stays; CAN_KickNext retries on next TX complete */
            }
        }

        /* Collecting timeout: 10 ms measured from sync_start_ms (Option A —
         * timeout from CAN_SendSync call, not from EXTI).                   */
        if (main_state == STATE_COLLECTING &&
            (HAL_GetTick() - sync_start_ms) >= COLLECTING_TIMEOUT_MS)
        {
            collecting_timeout_count++;
            last_timeout_joints = rx_joints_received;  /* snapshot: which joints replied */
            main_state = STATE_OBS_READY;
            ObsFinishCycle();    /* timeout -> chính sách thiếu-khớp quyết định gửi/khóa */
            main_state = STATE_IDLE;
            HAL_TIM_Base_Stop_IT(&htim3);
            tim3_tick = 0;
        }
        return;
    }

    /* ---- TIM4: EXTI gate (1 ms tick, clears gate at 15 ms) ---- */
    if (htim->Instance == TIM4)
    {
        tim4_tick++;
        if (tim4_tick >= 15)
        {
            exti_gate_active = 0;
            HAL_TIM_Base_Stop_IT(&htim4);
            __HAL_TIM_SET_COUNTER(&htim4, 0);
            tim4_tick = 0;
        }
        return;
    }
}

/*  =========================== CAN error / bus-off recovery ===========================
 *
 * AutoBusOff (ABOM=1) is already enabled in MX_CAN2_Init, so the hardware
 * automatically recovers after 128 × 11 consecutive recessive bits — no need to
 * call HAL_CAN_Stop/Start here (doing so inside an ISR would spin on HAL_GetTick).
 * We only reset the software state machine so pending operations are cleaned up.
 *
 * Track can_error_count in debugger to monitor CAN health.
 */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance != CAN2) return;

    can_error_count++;

    if (HAL_CAN_GetError(hcan) & HAL_CAN_ERROR_BOF)
    {
        can_busoff   = 1;   /* sticky — cleared manually in debugger or main loop */
        can_tx_state = CAN_IDLE;
        main_state   = STATE_IDLE;
    }
}
