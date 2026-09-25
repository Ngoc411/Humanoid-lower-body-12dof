/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  *
  *  Hardware:  STM32G474RETx
  *
  *  SPI1  MT6816 encoder — 3-wire (SPI_DIRECTION_1LINE)
  *          PB3 SCK | PB5 MOSI/SIO | PB6 NSS (SW)
  *
  *  SPI3  DRV8323SRTAT gate driver — 4-wire polling
  *          PC10 SCK | PC11 MISO | PC12 MOSI | PD2 NSS (SW)
  *
  *  TIM1  3-phase center-aligned PWM  (unchanged)
  *          PA8/PA9/PA10  CH1/2/3   PB13/PB14/PB15  CH1N/2N/3N
  *
  *  ADC1  Phase-A current  PB12 (IN11) injected, TIM1_CC4 trigger
  *  ADC2  Phase-B current  PB11 (IN14) auto-injected
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "DRV8323.h"
#include "MT6816.h"
#include "as5600.h"
#include "FOC_utils.h"
#include "FOC_protection.h"
#include "flash.h"

#include <stdio.h>
#include <math.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ------------------------------------------------------------------ */
/*  DRV8323 GPIO                                                       */
/*                                                                     */
/*  EN_GATE and nFAULT are NOT on PC10/PC11 anymore (those are SPI3). */
/*  !! Update the defines below to match your actual board wiring !!  */
/* ------------------------------------------------------------------ */
#define DRV8323_EN_GATE_GPIO_Port   GPIOA
#define DRV8323_EN_GATE_Pin         GPIO_PIN_15

#define DRV8323_NFAULT_GPIO_Port    GPIOC
#define DRV8323_NFAULT_Pin          GPIO_PIN_14

/* DRV8323 SPI3 NSS */
#define DRV8323_NSS_GPIO_Port       GPIOD
#define DRV8323_NSS_Pin             GPIO_PIN_2

/* MT6816 SPI1 NSS (PB6) */
#define MT6816_CS_GPIO_Port         GPIOB
#define MT6816_CS_Pin               GPIO_PIN_6

/* LED pins */
#define LED_Port2              GPIOA
#define LED_L2                    GPIO_PIN_0
#define LED_Port3              GPIOC
#define LED_L3                    GPIO_PIN_3
#define LED_Port4              GPIOC
#define LED_L4                    GPIO_PIN_2

/* ==========================================================================
 * CAN ID Layout
 * ==========================================================================
 *
 * Only change NODE_ID when flashing each joint — all IDs are derived from it.
 *
 *  NODE_ID   : 1 .. 12  (assigned by priority order; lower = higher priority)
 *
 *  Frame type         Direction              ID
 *  ─────────────────────────────────────────────────────────────────────
 *  SYNC               Jetson → ALL       CAN_ID_SYNC       = 0x001
 *                     (broadcast, triggers all nodes to send feedback)
 *
 *  CMD (joint cmd)    Jetson → Node N    CAN_ID_CMD(N)     = 0x010 + N
 *                     Joint 1  -> 0x011
 *                     Joint 2  -> 0x012  ...
 *                     Joint 12 -> 0x01C
 *
 *  FB  (feedback)     Node N  → Jetson   CAN_ID_FB(N)      = 0x100 + N
 *                     Joint 1  -> 0x101  (highest reply priority)
 *                     Joint 2  -> 0x102  ...
 *                     Joint 12 -> 0x10C  (lowest reply priority)
 *
 *  Reason for splitting CMD and FB into separate ID ranges:
 *    - CMD (0x01x) has lower IDs than FB (0x10x) → Jetson commands always
 *      win arbitration over feedback → lower command latency.
 *    - Each node's filter uses DUAL mode: accepts SYNC + its own CMD,
 *      no switch-case needed for classification.
 *
 *  !! ONLY CHANGE THIS LINE WHEN FLASHING EACH JOINT !!
 * ==========================================================================
 */
#define TEST 0
#define TEST_AS5600 0
#define LIMIT -45.0f
#define TEST_SIM 1

#define NODE_ID             6                       /* ← set 0..11 for each joint */

volatile float highest_vel;
volatile float pre_sp_input;

/* === Per-joint position limits (degrees, robot frame, before AXIS_SIGNS) === */
#if   (NODE_ID == 0)
  #define SP_INPUT_HOMING (-10.0f)

  #define SP_LIMIT_MIN  (-50.0f)
  #define SP_LIMIT_MAX  ( 50.0f)

  #define SP_LIMIT_MIN_RAW  (-50.0f)
  #define SP_LIMIT_MAX_RAW  ( 50.0f)

  #define AS5600_OFFSET 20.9179726f
  #define GEAR_RATIO 30.0f
  #define DIR  NORMAL_DIR
  #define WAIT_ANG 5.0f

#elif (NODE_ID == 1)
  #define SP_INPUT_HOMING (-10.0f)

  #define SP_LIMIT_MIN  (-50.0f)
  #define SP_LIMIT_MAX  ( 50.0f)

  #define SP_LIMIT_MIN_RAW  (-50.0f)
  #define SP_LIMIT_MAX_RAW  ( 50.0f)

  #define AS5600_OFFSET 300.9375f
  #define GEAR_RATIO 30.0f
  #define DIR  REVERSE_DIR
  #define WAIT_ANG 5.0f

#elif (NODE_ID == 2)
  #define SP_INPUT_HOMING (0.0f)

  #define SP_LIMIT_MIN  (-15.0f)
  #define SP_LIMIT_MAX  ( 10.0f)

  #define SP_LIMIT_MIN_RAW  (-15.0f)
  #define SP_LIMIT_MAX_RAW  ( 10.0f)

  #define AS5600_OFFSET 251.811737f
  #define GEAR_RATIO 30.0f
  #define DIR  REVERSE_DIR
  #define WAIT_ANG 5.0f

#elif (NODE_ID == 3)
  #define SP_INPUT_HOMING (0.0f)

  #define SP_LIMIT_MIN  (-10.0f)
  #define SP_LIMIT_MAX  ( 15.0f)

  #define SP_LIMIT_MIN_RAW  (-10.0f)
  #define SP_LIMIT_MAX_RAW  ( 15.0f)

  #define AS5600_OFFSET 207.070312f
  #define GEAR_RATIO 30.0f
  #define DIR  REVERSE_DIR
  #define WAIT_ANG 5.0f

#elif (NODE_ID == 4)
  #define SP_INPUT_HOMING (0.0f)

  #define SP_LIMIT_MIN  (-10.0f)
  #define SP_LIMIT_MAX  ( 10.0f)

  #define SP_LIMIT_MIN_RAW  (-10.0f)
  #define SP_LIMIT_MAX_RAW  ( 10.0f)

  #define AS5600_OFFSET 11.953125f
  #define GEAR_RATIO 30.0f
  #define DIR  REVERSE_DIR
  #define WAIT_ANG 5.0f

#elif (NODE_ID == 5)
  #define SP_INPUT_HOMING (0.0f)

  #define SP_LIMIT_MIN  (-10.0f)
  #define SP_LIMIT_MAX  ( 10.0f)

  #define SP_LIMIT_MIN_RAW  (-10.0f)
  #define SP_LIMIT_MAX_RAW  ( 10.0f)

  #define AS5600_OFFSET 234.052734f
  #define GEAR_RATIO 30.0f
  #define DIR  REVERSE_DIR
  #define WAIT_ANG 5.0f

#elif (NODE_ID == 6)
  #define SP_INPUT_HOMING (20.0f)

  #define SP_LIMIT_MIN  (-100.0f)
  #define SP_LIMIT_MAX  (   0.0f)

  #define SP_LIMIT_MIN_RAW  (   0.0f)
  #define SP_LIMIT_MAX_RAW  ( 100.0f)

  #define AS5600_OFFSET 86.6601639f
  #define GEAR_RATIO 30.0f
  #define DIR  REVERSE_DIR
  #define WAIT_ANG 5.0f

#elif (NODE_ID == 7)
  #define SP_INPUT_HOMING (-20.0f)

  #define SP_LIMIT_MIN  (-100.0f)
  #define SP_LIMIT_MAX  (   0.0f)

  #define SP_LIMIT_MIN_RAW  (-100.0f)
  #define SP_LIMIT_MAX_RAW  (   0.0f)

  #define AS5600_OFFSET 326.180054f
  #define GEAR_RATIO 30.0f
  #define DIR  REVERSE_DIR
  #define WAIT_ANG 5.0f

#elif (NODE_ID == 8)
  #define SP_INPUT_HOMING (15.5f)

  #define SP_LIMIT_MIN  (-45.0f)
  #define SP_LIMIT_MAX  ( 45.0f)

  #define SP_LIMIT_MIN_RAW  (-45.0f)
  #define SP_LIMIT_MAX_RAW  ( 45.0f)

  #define AS5600_OFFSET 65.3835068f
  #define GEAR_RATIO 26.0f
  #define DIR  NORMAL_DIR
  #define POS_E_DEADBAND 1.0f
        #define WAIT_ANG 15.0f

#elif (NODE_ID == 9)
  #define SP_INPUT_HOMING (15.5f)

  #define SP_LIMIT_MIN  (-45.0f)
  #define SP_LIMIT_MAX  ( 45.0f)

  #define SP_LIMIT_MIN_RAW  (-45.0f)
  #define SP_LIMIT_MAX_RAW  ( 45.0f)

  #define AS5600_OFFSET 220.869141f
  #define GEAR_RATIO 26.0f
  #define DIR  NORMAL_DIR
  #define POS_E_DEADBAND 1.0f
  #define WAIT_ANG 15.0f

#elif (NODE_ID == 10)
  #define SP_INPUT_HOMING (15.5f)

  #define SP_LIMIT_MIN  (-45.0f)
  #define SP_LIMIT_MAX  ( 45.0f)

  #define SP_LIMIT_MIN_RAW  (-45.0f)
  #define SP_LIMIT_MAX_RAW  ( 45.0f)

  #define AS5600_OFFSET 59.5898399f
  #define GEAR_RATIO 26.0f
  #define DIR  REVERSE_DIR
  #define POS_E_DEADBAND 1.0f
  #define WAIT_ANG 15.0f

#elif (NODE_ID == 11)
  #define SP_INPUT_HOMING (15.5f)

  #define SP_LIMIT_MIN  (-45.0f)
  #define SP_LIMIT_MAX  ( 45.0f)

  #define SP_LIMIT_MIN_RAW  (-45.0f)
  #define SP_LIMIT_MAX_RAW  ( 45.0f)

  #define AS5600_OFFSET 25.3124962f
  #define GEAR_RATIO 26.0f
  #define DIR  REVERSE_DIR
  #define POS_E_DEADBAND 1.0f
  #define WAIT_ANG 15.0f

#else
  #error "NODE_ID out of range [0..11]"
#endif

#if TEST_SIM
volatile float sp_input = 0;
#else
volatile float sp_input = SP_INPUT_HOMING; /* Iq_ref / rpm_ref / position_ref */
#endif

/* Macros to derive IDs from NODE_ID — no further changes needed */
#define CAN_ID_SYNC         	50                      /* Jetson broadcast           */
#define CHECK_INIT_DONE         	100                     /* main board hỏi init xong chưa (broadcast) */
#define CAN_ID_FALLING          	150                     /* main board báo NGÃ -> node homing lại tại chỗ */
#define CAN_OBS_ID_BASE(n)		(1 + (n))               /* Command Jetson → joint n   */
#define CAN_ACTION_ID_BASE(n)	(51 + (n))              /* Feedback joint n → Jetson  */
#define CAN_ID_INIT_DONE_BASE(n)	(101 + (n))         /* joint n báo init xong → Jetson */

/* This node's specific IDs */
#define CAN_OBS_ID     CAN_OBS_ID_BASE(NODE_ID)     /* ID to receive commands     */
#define CAN_ACTION_ID  CAN_ACTION_ID_BASE(NODE_ID)      /* ID to send feedback        */
#define CAN_ID_INIT_DONE  CAN_ID_INIT_DONE_BASE(NODE_ID)  /* ID báo init xong cho node này */

/* === FDCAN DLC Mapping Helper ===
 * FDCAN DataLength does not accept plain integers (1, 2, 3...) directly.
 * Must use the FDCAN_DLC_BYTES_x macros defined by HAL.
 */
static const uint32_t FDCAN_DLC_MAP[9] = {
    FDCAN_DLC_BYTES_0,  /* [0] = 0 bytes */
    FDCAN_DLC_BYTES_1,  /* [1] = 1 byte  */
    FDCAN_DLC_BYTES_2,  /* [2] = 2 bytes */
    FDCAN_DLC_BYTES_3,  /* [3] = 3 bytes */
    FDCAN_DLC_BYTES_4,  /* [4] = 4 bytes */
    FDCAN_DLC_BYTES_5,  /* [5] = 5 bytes */
    FDCAN_DLC_BYTES_6,  /* [6] = 6 bytes */
    FDCAN_DLC_BYTES_7,  /* [7] = 7 bytes */
    FDCAN_DLC_BYTES_8,  /* [8] = 8 bytes */
};

static const int AXIS_SIGNS[12] = {-1,  -1,  1,  1,  1,  1, -1, 1,  1,  1,  1,   1};
/*								   ax0  ax1 ax2 ax3 ax4 ax5 ax6 ax7 ax8 ax9 ax10 ax11*/
/*
 * 0: left hip pitch	-1
 * 1: right hip pitch	-1
 * 2: left hip roll		1
 * 3: right hip roll	1
 * 4: left hip yaw		1
 * 5: right hip yaw		1
 * 6: left knee			1
 * 7: right knee		-1
 * 8: left ankle pitch	1
 * 9: right ankle pitch	1
 * 10: left ankle roll	1
 * 11: right ankle roll	1
 * */

/* PWM channel mapping */
#define PHASE_A CCR3
#define PHASE_B CCR2
#define PHASE_C CCR1

/* Motor / FOC config */
#define BLDC_PWM_FREQ           10000
#define R_SHUNT                 0.01f
#define V_OFFSET_A              1.645f
#define V_OFFSET_C              1.657f
#define POLE_PAIR               7
#define SPEED_CONTROL_CYCLE     10
#define POS_PER_SPD_CYCLE       10
#define FOC_TS                  (1.0f / (float)BLDC_PWM_FREQ)

#define ENCODER_INIT_TIMEOUT_MS        1500U
#define ENCODER_INIT_MIN_VALID_SAMPLES 15U
#define ENCODER_INIT_MAX_ERRORS        5U
#define HOMING_TIMEOUT_MS              7000U
#define HOMING_OFFSET_LIMIT_DEG        360.0f
#define HOMING_VERIFY_TOL_DEG          2.0f
#define ENCODER_FRESH_MS               100U
#define AS5600_REF_MIN_STABLE_SAMPLES  12U
#define AS5600_REF_MAX_SPREAD_DEG      1.5f
#define AS5600_REF_MAX_REJECTS         1U
#define INIT_AWAY_MIN_DELTA_DEG        0.25f
#define INIT_AWAY_MAX_COUNT            6U
#define HOMING_RPM_LIMIT               15.0f

#if !TEST
static const foc_protection_config_t protection_config = {
    .encoder_init_timeout_ms = ENCODER_INIT_TIMEOUT_MS,
    .encoder_init_min_valid_samples = ENCODER_INIT_MIN_VALID_SAMPLES,
    .encoder_init_max_errors = ENCODER_INIT_MAX_ERRORS,
    .encoder_fresh_ms = ENCODER_FRESH_MS,
    .as5600_ref_min_stable_samples = AS5600_REF_MIN_STABLE_SAMPLES,
    .as5600_ref_max_rejects = AS5600_REF_MAX_REJECTS,
    .as5600_ref_max_spread_deg = AS5600_REF_MAX_SPREAD_DEG,
    .wait_ang = WAIT_ANG,
    .homing_rpm_limit = HOMING_RPM_LIMIT,
    /* Giới hạn vị trí duy nhất cho từng node (foc_protection_position_target clamp về dải này) */
    .pos_limit_high = SP_LIMIT_MAX_RAW,
    .pos_limit_low  = SP_LIMIT_MIN_RAW,
    .init_away_min_delta_deg = INIT_AWAY_MIN_DELTA_DEG,
    .init_away_max_count = INIT_AWAY_MAX_COUNT,
    .fault_led_port = LED_Port4,
    .fault_led_pin = LED_L4,
};
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c4;
DMA_HandleTypeDef hdma_i2c4_rx;
DMA_HandleTypeDef hdma_i2c4_tx;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */

volatile uint8_t init_done = 0;
volatile uint8_t rehome_request = 0;   /* 1 = đang homing lại sau khi nhận CAN_ID_FALLING */

volatile uint8_t tim_init = 0;
volatile uint8_t foc_power_stage_armed = 0;

volatile int iq_index;
volatile float iq_mean;

#define SEGMENT_SIZE  150
#define NUM_SEGMENTS  18
#define TEST_SIM_SIZE (SEGMENT_SIZE * NUM_SEGMENTS)  // 2700

typedef struct {
    int     index;          // vị trí ghi hiện tại trong mảng
    int     count;          // số segment đã hoàn thành
    bool    paused;         // đang trong trạng thái tạm dừng giữa segment
    bool    full;           // đã ghi đủ toàn bộ mảng

    uint32_t time_ms[TEST_SIM_SIZE];

    float target_position[TEST_SIM_SIZE];

    float actual_angle[TEST_SIM_SIZE];
    float actual_angle_as5600[TEST_SIM_SIZE];

    float actual_rpm[TEST_SIM_SIZE];
    float actual_acc[TEST_SIM_SIZE];

    float I_ref[TEST_SIM_SIZE];
    float actual_I[TEST_SIM_SIZE];
} TEST_SIM_t;

// dump binary memory D:/test_sim/test_sim.bin &test_sim (&test_sim + 1)

volatile TEST_SIM_t test_sim = {0};

motor_config_t m_config;
foc_t          foc;
DRV8323_t      drv;

volatile int exti_count = 0;
volatile float debug_action = 0;

volatile uint32_t vbus_adc_raw;
volatile float vbus_adc;

/* === FDCAN TX === */
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t  txData[8]      = {0};
volatile int tx_success = 0;
volatile int tx_fail    = 0;

/* === FDCAN RX === */
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t  rxData[8]   = {0};
volatile int rx_ok   = 0;
volatile int rx_lost = 0;

/* === FDCAN shared data ===
 * can_cmd_position: position command from Jetson, written by CAN callback, read by ADC ISR.
 * ADC ISR has higher priority than CAN callback so no critical section is needed —
 * ADC ISR always preempts CAN callback, never the other way around.
 */
/* === Debug === */
volatile uint32_t can_err     = 0;
volatile uint32_t count_tx    = 0;
volatile uint32_t count_tx_cb = 0;
volatile uint32_t free_level  = 0;
volatile uint32_t init_fault_flags = 0;

/* Phân tích nguyên nhân con của INIT_FAULT_BAD_ANGLE (fault 32).
 * Là bitmask: nhiều nguyên nhân có thể cùng đúng một lúc. */
typedef enum {
    BAD_ANGLE_AS5600_INVALID = (1U << 0),  /* foc.actual_angle_as5600 không hợp lệ (NaN/Inf/|x|>360) */
    BAD_ANGLE_MT6816_INVALID = (1U << 1),  /* foc.actual_angle (MT6816) không hợp lệ                 */
    BAD_ANGLE_AS5600_STALE   = (1U << 2),  /* AS5600 quá hạn: tuổi dữ liệu > ENCODER_FRESH_MS         */
    BAD_ANGLE_MT6816_STALE   = (1U << 3),  /* MT6816 quá hạn: tuổi dữ liệu > ENCODER_FRESH_MS         */
} bad_angle_cause_t;

volatile uint32_t bad_angle_cause = 0;     /* != 0 khi fault 32 xảy ra; xem các bit ở trên */

volatile int while_count;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI3_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C4_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
uint8_t FDCAN_Send(uint32_t id, uint8_t *data, uint8_t size);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile int deg_test;

static void motor_power_stage_force_off(void)
{
    foc_power_stage_armed = 0;
    init_done = 0;
//    sp_input = 0.0f;

    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    TIM1->CCER = 0;
    TIM1->BDTR &= ~TIM_BDTR_MOE;

    HAL_GPIO_WritePin(DRV8323_EN_GATE_GPIO_Port, DRV8323_EN_GATE_Pin, GPIO_PIN_RESET);
}

static float rpm_to_rad(float rpm)
{
    return rpm * (2.0f * PI / 60.0f);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5) {
#if !TEST
        if (!init_done && foc.control_mode == POSITION_CONTROL_MODE)
            if (foc_protection_init_moving_away_from_zero(foc.actual_angle_as5600))
                foc_protection_init_fail(INIT_FAULT_HOMING_AWAY);
#endif
    }

    if (htim->Instance == TIM3)
    {
        if (test_sim.index >= SEGMENT_SIZE) {
            HAL_TIM_Base_Stop_IT(&htim3);
            __HAL_TIM_SET_COUNTER(&htim3, 0);
            test_sim.index = 0;  // reset index về 0 cho segment tiếp theo
            test_sim.count++;
        } else {
            // offset để ghi vào đúng vị trí trong mảng 1D
            int i = test_sim.count * SEGMENT_SIZE + test_sim.index;

            test_sim.time_ms[i] = 20 * test_sim.index;  // reset 0..2980ms mỗi segment

            test_sim.target_position[i] = sp_input;

            test_sim.actual_angle[i] = foc.actual_angle;
            test_sim.actual_angle_as5600[i] = foc.actual_angle_as5600;

            test_sim.actual_rpm[i] = foc.actual_rpm;
            float current_vel = rpm_to_rad(test_sim.actual_rpm[i]);
            if (test_sim.index == 0) {
                test_sim.actual_acc[i] = 0;
            } else {
                float prev_vel = rpm_to_rad(test_sim.actual_rpm[i - 1]);
                test_sim.actual_acc[i] = (current_vel - prev_vel) / 0.02f;
            }

            test_sim.I_ref[i] = foc.iq_ref;
            test_sim.actual_I[i] = foc.iq;

            test_sim.index++;
        }
    }
}

volatile DRV8323_dbg_t drv_dbg;

/******************************************************************************/
/*  encoder init                                                       */
/******************************************************************************/

/* Manually toggle SCL up to 9 times to free a slave holding SDA low after
   MCU reset. PC6 = SCL, PC7 = SDA (I2C4 open-drain, pulled high externally). */
static void i2c4_bus_recover(void)
{
    /* Temporarily switch PC6/PC7 to GPIO open-drain output */
    GPIO_InitTypeDef g = {0};
    g.Pin   = GPIO_PIN_6 | GPIO_PIN_7;
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &g);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
    HAL_Delay(1);

    /* Clock up to 9 times until SDA is released */
    for (int i = 0; i < 9; i++) {
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == GPIO_PIN_SET) break;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
        HAL_Delay(1);
    }

    /* STOP condition: SCL high, SDA low → high */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_Delay(1);

    /* Restore PC6/PC7 to I2C4 alternate function */
    g.Mode      = GPIO_MODE_AF_OD;
    g.Alternate = GPIO_AF8_I2C4;
    HAL_GPIO_Init(GPIOC, &g);
}

void magnetic_encoder_init(void)
{
	#if !TEST
	  foc.actual_angle_general = &foc.actual_angle_as5600;
	#else
	  foc.actual_angle_general = &foc.actual_angle;   /* point straight at MT6816 */
	#endif


    MT6816_Config(MT6816_CS_GPIO_Port, MT6816_CS_Pin, &hspi1);
    MT6816_Start();

    i2c4_bus_recover();   /* unlock bus if slave held SDA low after reset */
    AS5600_Config(&hi2c4);
    HAL_Delay(2);         /* AS5600 boot: ~1 ms EEPROM read after power-on */
}

/******************************************************************************/
/*  DRV8323 gate driver init                                                  */
/******************************************************************************/

void drv_init(void)
{
    DRV8323_SPI_config(&drv, &hspi3, DRV8323_NSS_GPIO_Port, DRV8323_NSS_Pin);
    DRV8323_GPIO_ENGATE_config(&drv, DRV8323_EN_GATE_GPIO_Port, DRV8323_EN_GATE_Pin);
    DRV8323_GPIO_NFAULT_config(&drv, DRV8323_NFAULT_GPIO_Port,  DRV8323_NFAULT_Pin);

    DRV8323_TIMER_config(&drv, &htim1, BLDC_PWM_FREQ);
    DRV8323_current_sens_config(&drv, DRV_GAIN_10VV, R_SHUNT, V_OFFSET_A, V_OFFSET_C);
}

/******************************************************************************/
/*  ADC init                                                                  */
/******************************************************************************/

void ADC_Init(void)
{
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);

    HAL_ADC_Start(&hadc2);
    HAL_ADC_Start(&hadc3);

    HAL_ADCEx_InjectedStart_IT(&hadc1);
    HAL_ADCEx_InjectedStart_IT(&hadc2);
    HAL_ADCEx_InjectedStart_IT(&hadc3);

    HAL_Delay(10);
}

/******************************************************************************/
/*  FOC init                                                                  */
/******************************************************************************/

void FOC_init(void)
{
	foc.check_mul_round = 1;
    foc.mech_theta = &encoder.mechTheta;
    foc_set_torque_control_bandwidth(&foc, m_config.I_ctrl_bandwidth);

    foc.id_ctrl.out_max_dynamic = m_config.id_out_max;
    foc.iq_ctrl.out_max_dynamic = m_config.iq_out_max;
    pid_init(&foc.id_ctrl, m_config.id_kp, m_config.id_ki, 0.0f,
             FOC_TS, 10.0f, m_config.id_e_deadband);
    pid_init(&foc.iq_ctrl, m_config.iq_kp, m_config.iq_ki, 0.0f,
             FOC_TS, 10.0f, m_config.iq_e_deadband);

    pid_init(&foc.speed_ctrl, m_config.speed_kp, m_config.speed_ki, 0.0001f,
             FOC_TS * SPEED_CONTROL_CYCLE, m_config.speed_out_max, m_config.speed_e_deadband);
    foc.speed_ctrl.d_alpha_filter = 0.01f;

    pid_init(&foc.pos_ctrl, m_config.pos_kp, m_config.pos_ki, m_config.pos_kd,
             FOC_TS * SPEED_CONTROL_CYCLE, m_config.pos_out_max, m_config.pos_e_deadband);
    foc.pos_ctrl.d_alpha_filter = 0.85f;

    foc_pwm_init(&foc, &(TIM1->PHASE_A), &(TIM1->PHASE_B), &(TIM1->PHASE_C),
                 drv.pwm_resolution);
    foc_motor_init(&foc, POLE_PAIR, 360);

    foc_sensor_init(&foc, m_config.encd_offset, DIR);
    foc_gear_reducer_init(&foc, GEAR_RATIO);
#if !TEST
    foc_set_limit_current(&foc, 10.0f);
#else
    foc_set_limit_current(&foc, 1.0f);
#endif
    foc.v_bus = 12.32f;
    foc.Ld    = m_config.Ld;
    foc.Lq    = m_config.Lq;
}

/******************************************************************************/
/*  Delta-time helper                                                         */
/******************************************************************************/

uint32_t get_dt_us(void)
{
    uint32_t elapsed_us = TIM2->CNT;
    TIM2->CNT = 0;
    return elapsed_us;
}

/******************************************************************************/
/*  Open-loop SVPWM                                                           */
/******************************************************************************/

void test_openloop() {
  open_loop_voltage_control(&foc, VD_CAL, VQ_CAL, 0.0f);
  HAL_Delay(1000);

  for (float i = 0; i < 2*PI*foc.pole_pairs; i+=0.2) {
    open_loop_voltage_control(&foc, VD_CAL, VQ_CAL, i);
    HAL_GPIO_WritePin(LED_Port4, LED_L4, GPIO_PIN_SET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(LED_Port4, LED_L4, GPIO_PIN_RESET);
    HAL_Delay(50);
  }
  open_loop_voltage_control(&foc, 0, 0, 0);
}

/******************************************************************************/
/*  Torque / speed control update                                             */
/******************************************************************************/

static int torque_control_update(void)
{
    static uint8_t speed_cnt = 0;
    static uint8_t pos_cnt   = 0;

    DRV8323_get_current(&drv, &foc.ia, &foc.ic);
    foc.e_rad = foc.e_rad_comp;
    foc_current_control_update(&foc);

    if (++speed_cnt >= SPEED_CONTROL_CYCLE) {
        speed_cnt = 0;
        MT6816_Get_RPM(&foc, get_dt_us());
        foc_calc_mech_rpm_encoder(&foc);

        if (++pos_cnt >= POS_PER_SPD_CYCLE) {
            pos_cnt = 0;
            return 2;
        }
        return 1;
    }
    return 0;
}

/******************************************************************************/
/*  SPI callbacks — MT6816 4-wire DMA chain                                  */
/******************************************************************************/

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi != &hspi1) return;

	MT6816_DeSelect();

	encoder.check = 0;

	// save the important data
	encoder.complete_16Bit[encoder.read_flag] = encoder.rx_8Bit[1];

	if (encoder.read_flag == 1) {
		uint32_t valid_before = encoder.validCount;
		MT6816_Update_Angle_8Bit(&foc);
		if (encoder.validCount != valid_before) {
			foc_calc_electric_angle(&foc);
			MT6816_get_multiturn_degree(&foc);
			foc_calc_mech_pos_encoder(&foc);
		}
	}

	// change the flag state
	encoder.read_flag ^= 1;
	Spi_Transmit_Receive_Data_8Bit(reg_addrs[encoder.read_flag]);

//#if TEST_AS5600
//	Spi_Transmit_Receive_Data_8Bit(reg_addrs[encoder.read_flag]);
//#endif
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
	if (hspi != &hspi1) return;

	MT6816_DeSelect();
	encoder.check = 0;
	encoder.spiErrorCount++;
}


/******************************************************************************/
/*  I2C4 DMA callback — AS5600 angle read chain                               */
/******************************************************************************/

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c != &hi2c4) return;
    AS5600_Update_Angle(&foc, AS5600_OFFSET);
    AS5600_Start_DMA();
//#if TEST_AS5600
//    AS5600_Start_DMA();
//#endif
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c != &hi2c4) return;
    AS5600_Error_Handler();
}

/******************************************************************************/
/*  ADC injected conversion callback — 10 kHz current loop                   */
/******************************************************************************/

volatile int ret = 0;
volatile int change_count = 0;

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {

		DRV8323_set_adc_a(&drv, ADC1->JDR1); // phase A
		DRV8323_set_adc_c(&drv, ADC2->JDR1); // phase C

		const float acc_rpm = 5.0f;

		if (pre_sp_input != sp_input) {
			highest_vel = 0;
			pre_sp_input = sp_input;
		}
		if (fabsf(foc.actual_rpm) > fabsf(highest_vel)) highest_vel = foc.actual_rpm;

#if TEST
//		change_count++;
//		if (change_count >= 25000) {
//			change_count = 0;
//			// Toggle sp_input between 100 and 250
//			if (sp_input <= LIMIT) {  // Use >= for float safety
//				sp_input = 0.0f;
//			} else if (sp_input == 0.0f) {
//				sp_input = LIMIT;
//			}
//		}

		if (iq_index == 30000) {
			iq_index = 0;
			iq_mean = 0;
		}
		iq_index ++;
		iq_mean = iq_mean + (foc.iq - iq_mean) / (float)iq_index;
#endif

//#if !TEST_AS5600
//		Spi_Transmit_Receive_Data_8Bit(reg_addrs[encoder.read_flag]);
//	    AS5600_Start_DMA();
//#endif

		if (!foc_power_stage_armed) {
			foc.id_ref = 0.0f;
			foc.iq_ref = 0.0f;
			DRV8323_set_pwm(&drv, 0, 0, 0);
			return;
		}

		switch (foc.control_mode) {

		case TORQUE_CONTROL_MODE:
			foc.id_ref = 0.0f;
			foc.iq_ref = sp_input;
			torque_control_update();
			break;

		case SPEED_CONTROL_MODE: {
			static float sp_rpm = 0.0f;
			ret = torque_control_update();
			if (ret >= 1) {
				float error = sp_input - sp_rpm;
				float step  = (error > 0.0f) ? acc_rpm : -acc_rpm;
				sp_rpm = (fabsf(error) > acc_rpm) ? (sp_rpm + step) : sp_input;
				foc_speed_control_update(&foc, sp_rpm);
			}
			break;
		}

		case POSITION_CONTROL_MODE: {
			ret = torque_control_update();
			/* Position loop runs every POS_PER_SPD_CYCLE speed cycles (~100 Hz) */
			if (ret >= 1) {
				foc_calc_mech_pos_encoder(&foc);
#if !TEST
#if !TEST_SIM
				float protected_target = foc_protection_position_target(sp_input);
#else
				float protected_target = sp_input;
#endif
				foc_position_control_update(&foc, protected_target);
#else
				foc_position_control_update(&foc, sp_input);
#endif
#if !TEST
				foc_protection_limit_pre_homing_speed();
#endif
			}
			break;
		}

		case POWER_UP_MODE:
			open_loop_voltage_control(&foc, 0.0f, 0.0f, 0.0f);
			break;

		case CALIBRATION_MODE:
			break;

		case TEST_MODE:
			break;

		default:
			break;
		}
    }
    else if (hadc->Instance == ADC3) {
        vbus_adc_raw = ADC3->JDR1;
        vbus_adc = (float)vbus_adc_raw / 4095.0f * 3.3f * 5.7f;
        foc.v_bus = vbus_adc;
    }
}

/******************************************************************************/
/*  Utility                                                                   */
/******************************************************************************/

static void blink(void)
{
    HAL_GPIO_WritePin(LED_Port4, LED_L4, GPIO_PIN_SET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(LED_Port4, LED_L4, GPIO_PIN_RESET);
    HAL_Delay(100);
}

volatile uint16_t spi_verify;

/******************************************************************************/
/*  FDCAN Callbacks & Helper                                                  */
/******************************************************************************/

/**
  * @brief  Callback: new frame received in RX FIFO 0.
  *         Thanks to 2 DUAL filters, only 2 ID types can reach here:
  *           CAN_ID_SYNC     (0x001) — Jetson broadcast → send feedback
  *           CAN_MY_CMD_ID   (0x010 + NODE_ID) — position command for this joint
  *         No complex switch-case needed; hardware has already filtered.
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != RESET)
    {
        rx_lost++;
    }

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, rxData) == HAL_OK)
        {
            rx_ok++;

            if (RxHeader.Identifier == CAN_ID_SYNC)
            {
                /* SYNC from Jetson — send current joint state as feedback:
                 *   bytes [0..3] = position (float, rad, little-endian)
                 *   bytes [4..7] = velocity (float, rpm, little-endian)
                 */
                float pos = foc.actual_angle * AXIS_SIGNS[NODE_ID];
                float vel = foc.actual_rpm * AXIS_SIGNS[NODE_ID];
                memcpy(&txData[0], &pos, sizeof(float));
                memcpy(&txData[4], &vel, sizeof(float));
                FDCAN_Send(CAN_OBS_ID, txData, 8);

                HAL_GPIO_TogglePin(LED_Port4, LED_L4);
            }
            else if (RxHeader.Identifier == CAN_ACTION_ID)
            {
                /* CMD from Jetson — position command for this joint:
                 *   bytes [0..3] = target position (float, rad or deg)
                 * Written to shared variable; ADC ISR reads it on the next cycle.
                 * Safe because ADC ISR (priority 0) always preempts CAN callback (priority 3)
                 * → no race condition: CAN cannot interrupt ADC mid-execution.
                 */
                /* Chỉ nhận action khi node SẴN SÀNG (init_done): lúc đang homing lại
                 * (sau ngã) bỏ qua action để không ghi đè home target. Chặn NaN/Inf. */
                if (init_done && RxHeader.DataLength == FDCAN_DLC_BYTES_4)
                {
                    float action;
                    memcpy(&action, rxData, sizeof(float));
                    if (isfinite(action))
                    {
                        /* Clamp về dải cho phép của khớp (robot frame, trước AXIS_SIGNS) */
                        if      (action > SP_LIMIT_MAX) action = SP_LIMIT_MAX;
                        else if (action < SP_LIMIT_MIN) action = SP_LIMIT_MIN;
                        sp_input = action * AXIS_SIGNS[NODE_ID];
                    }
                }
            }
            else if (RxHeader.Identifier == CHECK_INIT_DONE)
            {
                /* Main board chưa nhận được tín hiệu init-done trước đó → hỏi lại.
                 * Chưa init xong: im lặng (không gửi gì).
                 * Đã init xong: gửi lại CAN_ID_INIT_DONE để xác nhận (data rác). */
                if (init_done)
                {
                    uint8_t init_done_msg[1] = {0};
                    FDCAN_Send(CAN_ID_INIT_DONE, init_done_msg, 1);
                }
            }
            else if (RxHeader.Identifier == CAN_ID_FALLING)
            {
                /* Main board báo ROBOT NGÃ -> node HOMING TẠI CHỖ (KHÔNG reset):
                 *   init_done=0  : "chưa sẵn sàng" lại -> ngừng trả lời CHECK_INIT_DONE,
                 *                  bỏ qua action mới, bật giới hạn tốc độ homing.
                 *   sp_input=home: vòng position-control (ADC ISR) kéo khớp về home.
                 *   giữ calibration + offset cũ (encoder tuyệt đối vẫn đúng).
                 * while(1) theo dõi: về tới home -> init_done=1 -> gửi CAN_ID_INIT_DONE. */
                init_done      = 0;
                sp_input       = SP_INPUT_HOMING;   /* đích position-control = home (BẮT BUỘC) */
                foc_protection_set_homing_target(SP_INPUT_HOMING);
                rehome_request = 1;
            }
            //HAL_GPIO_TogglePin(LED_Port4, LED_L4); /* dbg: every RX msg */
        }

        __HAL_FDCAN_CLEAR_IT(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    }
}

/**
  * @brief  Callback: TX complete — frame has been sent on the bus and ACK received.
  */
void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes)
{
    count_tx_cb++;
    if (BufferIndexes != 0xFFFFFFFF)
    {
        tx_success++;
    }
    //HAL_GPIO_TogglePin(LED_Port2, LED_L2); /* dbg: TX complete */
}

/**
  * @brief  Callback: FDCAN error (Bus-Off, Error Passive, etc.).
  */
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    tx_fail++;
    can_err = HAL_FDCAN_GetError(hfdcan);

    FDCAN_ProtocolStatusTypeDef psr;
    HAL_FDCAN_GetProtocolStatus(hfdcan, &psr);
    if (psr.BusOff) {
        HAL_FDCAN_Stop(hfdcan);
        HAL_FDCAN_Start(hfdcan);
    }
}

/**
  * @brief  Send a CAN frame via TX FIFO Queue.
  * @param  id   : Standard ID (11-bit, 0x000 – 0x7FF)
  * @param  data : Pointer to data array
  * @param  size : Number of bytes (1–8)
  * @retval 1 on success, 0 on failure
  */
uint8_t FDCAN_Send(uint32_t id, uint8_t *data, uint8_t size)
{
    if (size == 0 || size > 8) size = 8;

    free_level = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);
    if (free_level == 0)
    {
        tx_fail++;
        return 0;
    }

    TxHeader.Identifier          = id;
    TxHeader.IdType              = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType         = FDCAN_DATA_FRAME;
    TxHeader.FDFormat            = FDCAN_CLASSIC_CAN;
    TxHeader.BitRateSwitch       = FDCAN_BRS_OFF;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker       = 0;
    TxHeader.DataLength          = FDCAN_DLC_MAP[size];

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, data) != HAL_OK)
    {
        tx_fail++;
        return 0;
    }

    count_tx++;
    return 1;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  MX_SPI3_Init();
  MX_FDCAN1_Init();
  MX_ADC3_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_I2C4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim5);
  motor_power_stage_force_off();
  foc.control_mode = CALIBRATION_MODE;

#if !TEST
  foc_protection_init(&foc, &drv, &init_done, &init_fault_flags, &protection_config);
#endif

  /* === FDCAN Filter Configuration ===
   * Cần nhận 3 ID nên không dùng 1 DUAL filter được nữa.
   * Dùng 3 CLASSIC mask filter, mỗi filter khớp đúng 1 ID (mask = 0x7FF).
   * Hardware tự loại mọi frame khác, không tốn CPU.
   *
   * Filter 0: SYNC            (50)            — Jetson broadcast
   * Filter 1: CMD             (51 + NODE_ID)  — command cho joint này
   * Filter 2: CHECK_INIT_DONE (100)           — main board hỏi init xong chưa
   */
  FDCAN_FilterTypeDef sFilterConfig = {0};
  sFilterConfig.IdType       = FDCAN_STANDARD_ID;
  sFilterConfig.FilterType   = FDCAN_FILTER_MASK;        /* ID1 = giá trị, ID2 = mask */
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID2    = 0x7FF;                    /* mask đủ 11 bit → khớp chính xác */

  sFilterConfig.FilterIndex  = 0;
  sFilterConfig.FilterID1    = CAN_ID_SYNC;              /* 50 */
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    Error_Handler();

  sFilterConfig.FilterIndex  = 1;
  sFilterConfig.FilterID1    = CAN_ACTION_ID;            /* 51 + NODE_ID */
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    Error_Handler();

  sFilterConfig.FilterIndex  = 2;
  sFilterConfig.FilterID1    = CHECK_INIT_DONE;          /* 100 */
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    Error_Handler();

  sFilterConfig.FilterIndex  = 3;
  sFilterConfig.FilterID1    = CAN_ID_FALLING;           /* 150 — main board báo ngã */
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    Error_Handler();

  /* Reject ALL frames that don't match the explicit filter above.
   * Without this, STM32G4 FDCAN default (ANFS=00) accepts every non-matching
   * standard frame into FIFO0, flooding it with other nodes' feedback frames
   * and action frames meant for other joints. */
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
          FDCAN_REJECT,        /* non-matching standard frames → reject */
          FDCAN_REJECT,        /* non-matching extended frames → reject */
          FDCAN_REJECT_REMOTE, /* remote standard frames → reject */
          FDCAN_REJECT_REMOTE) /* remote extended frames → reject */
      != HAL_OK)
    Error_Handler();

  /* === NVIC Priority ===
   * ADC  (FOC current loop) : 0 — highest, strictest real-time requirement
   * DMA  (encoder SPI)      : 1 — FOC needs position for computation
   * FDCAN                   : 3 — 50 Hz, lowest priority
   */
  HAL_NVIC_SetPriority(ADC1_2_IRQn,        0, 0);
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 1, 0);
  HAL_NVIC_SetPriority(ADC3_IRQn,          2, 0);
  HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn,    3, 0);
  HAL_NVIC_SetPriority(FDCAN1_IT1_IRQn,    3, 0);

  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  TIM4->CCR3 = 100;
  flash_default_config(&m_config);
//  if (GEAR_RATIO == 30.0f) {
//      m_config.pos_kp = 3.1f;
//      m_config.pos_ki = 0.0f;
//      m_config.pos_kd = 0.21f;
//  } else {
//      m_config.pos_kp = 2.0f;
//      m_config.pos_ki = 0.0f;
//      m_config.pos_kd = 0.15f;
//  }

  m_config.pos_kp = 0.6f;
  m_config.pos_ki = 0.0f;
  m_config.pos_kd = 0.015f;
#ifdef POS_E_DEADBAND
  m_config.pos_e_deadband = POS_E_DEADBAND;
#endif
  init_trig_lut();

  magnetic_encoder_init();

  HAL_Delay(500);

#if !TEST

  if (!AS5600_Magnet_OK()) {
      foc_protection_init_fail(INIT_FAULT_AS5600_MAGNET);
  }
  float as5600_boot_ref = 0.0f;
  if (!foc_protection_wait_as5600_stable_reference(&as5600_boot_ref)) {
      foc_protection_init_fail(init_fault_flags);
  }
#endif

# if !TEST_AS5600

//  foc.control_mode = TEST_MODE;

  /* 1. Configure DRV8323 handles/timer only; keep EN_GATE low */
  drv_init();

  /* 2. CCR4 triggers ADC at PWM valley (center-aligned) */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, drv.pwm_resolution - 350);

  /* 3. FOC init BEFORE starting PWM — so pwm_a/b/c pointers are ready */
  FOC_init();
  motor_power_stage_force_off();
  foc_protection_motor_output_inhibit();

  /* 4. ADC calibration + start; injected conversions wait for TIM1 trigger */
  ADC_Init();

  #if !TEST
    if (!foc_protection_wait_as5600_init_ready()) {
        foc_protection_init_fail(init_fault_flags);
    }
    if (!foc_protection_wait_mt6816_init_ready()) {
        foc_protection_init_fail(init_fault_flags);
    }
  #endif

  /* 5. Wake and configure DRV8323 only after FOC, ADC, and encoders are ready */
  if (!DRV8323_init(&drv)) {
      Error_Handler();
  }

  /* 6. Start PWM at zero duty, then arm current loop */
  DRV8323_set_pwm(&drv, 0, 0, 0);
  DRV8323_start_pwm(&drv);

  /* 7. Debug snapshot before running open-loop */
  DRV8323_debug_read(&drv, &drv_dbg);

  #if TEST
    /* ===== TEST MODE: no protection, no homing, fixed gains ===== */

    m_config.pos_kp = 1.4f;
    m_config.pos_ki = 0.0f;
    m_config.pos_kd = 0.22f;
    pid_init(&foc.pos_ctrl, m_config.pos_kp, m_config.pos_ki, m_config.pos_kd,
             FOC_TS * SPEED_CONTROL_CYCLE, m_config.pos_out_max, m_config.pos_e_deadband);

    foc_power_stage_armed = 1;
    foc.control_mode = CALIBRATION_MODE;
    HAL_Delay(50);

    foc_cal_encoder_misalignment(&foc);
    foc.check_mul_round = 0;

    HAL_Delay(100);

    foc.control_mode = POSITION_CONTROL_MODE;

  #else
    /* ===== NORMAL MODE: full protection + homing ===== */

    foc_power_stage_armed = 1;
    foc.control_mode = CALIBRATION_MODE;
    HAL_Delay(50);

    foc_cal_encoder_misalignment(&foc);
    foc.check_mul_round = 0;

    foc.control_mode = POWER_UP_MODE;

    HAL_Delay(500);

    if (!foc_protection_wait_as5600_init_ready()) {
        foc_protection_init_fail(init_fault_flags);
    }
    if (!foc_protection_wait_mt6816_init_ready()) {
        foc_protection_init_fail(init_fault_flags);
    }

    foc.control_mode = POSITION_CONTROL_MODE;

    uint32_t homing_start_ms = HAL_GetTick();
    /* Home tới sp_input chứ không cố định về 0 (sp_input lúc init có thể khác 0).
     * CAN chưa bật trong giai đoạn này nên snapshot ổn định. */
    const float homing_target = sp_input;
    foc_protection_set_homing_target(homing_target);
    foc_protection_init_zero_trend_reset();
    while (fabsf(foc.actual_angle_as5600 - homing_target) >= WAIT_ANG) {
        if ((HAL_GetTick() - homing_start_ms) > HOMING_TIMEOUT_MS) {
            foc_protection_init_fail(INIT_FAULT_HOMING_TIMEOUT);
        }
        uint32_t cause = 0;
        if (!foc_protection_angle_is_valid(foc.actual_angle_as5600))   cause |= BAD_ANGLE_AS5600_INVALID;
        if (!foc_protection_angle_is_valid(foc.actual_angle))          cause |= BAD_ANGLE_MT6816_INVALID;
//        if ((HAL_GetTick() - as5600.lastUpdateMs) > ENCODER_FRESH_MS)  cause |= BAD_ANGLE_AS5600_STALE;
//        if ((HAL_GetTick() - encoder.lastUpdateMs) > ENCODER_FRESH_MS) cause |= BAD_ANGLE_MT6816_STALE;
        if (cause != 0) {
            bad_angle_cause = cause;
            foc_protection_init_fail(INIT_FAULT_BAD_ANGLE);
        }
    }

    float as5600_homing_ref = 0.0f;
    if (!foc_protection_wait_as5600_stable_reference(&as5600_homing_ref) ||
        fabsf(as5600_homing_ref - homing_target) >= WAIT_ANG) {
        foc_protection_init_fail(INIT_FAULT_AS5600_UNSTABLE);
    }

    foc.actual_angle_offset = foc.actual_angle - as5600_homing_ref;
     if (!foc_protection_angle_is_valid(foc.actual_angle_offset) ||
         fabsf(foc.actual_angle_offset) > HOMING_OFFSET_LIMIT_DEG ||
         fabsf((foc.actual_angle - foc.actual_angle_offset) - as5600_homing_ref) > HOMING_VERIFY_TOL_DEG) {
         foc_protection_init_fail(INIT_FAULT_BAD_OFFSET);
     }
    foc.actual_angle_general = &foc.actual_angle;

    init_done = 1;
    HAL_TIM_Base_Stop_IT(&htim5);
    m_config.pos_kp = 1.5f;
    m_config.pos_ki = 0.0f;
    m_config.pos_kd = 0.07f;
    pid_init(&foc.pos_ctrl, m_config.pos_kp, m_config.pos_ki, m_config.pos_kd,
             FOC_TS * SPEED_CONTROL_CYCLE, m_config.pos_out_max, m_config.pos_e_deadband);

    /* Enable FDCAN */
    if (HAL_FDCAN_ActivateNotification(&hfdcan1,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
            FDCAN_IT_RX_FIFO0_MESSAGE_LOST, 0) != HAL_OK)
      Error_Handler();
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_COMPLETE,
            FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2) != HAL_OK)
      Error_Handler();
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
      Error_Handler();
    can_err = HAL_FDCAN_GetError(&hfdcan1);

    /* Báo cho Jetson biết joint đã init xong (data rác, chỉ cần ID là đủ) */
    uint8_t init_done_msg[1] = {0};
    FDCAN_Send(CAN_ID_INIT_DONE, init_done_msg, 1);

  #endif /* TEST */

#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if (sp_input != 0 && !deg_test) {
          HAL_TIM_Base_Start_IT(&htim3);
          deg_test = 1;
	  } else if (sp_input == 0) {
          HAL_TIM_Base_Stop_IT(&htim3);
          deg_test = 0;
          __HAL_TIM_SET_COUNTER(&htim3, 0);
          test_sim.index = 0;  // reset index về 0 cho segment tiếp theo
	  }

    /* ===== Giám sát HOMING LẠI sau khi ngã (CAN_ID_FALLING) =====
     * Nhánh CAN đã đặt init_done=0 + sp_input=home + rehome_request=1; vòng
     * position-control (ADC ISR) đang kéo khớp về home (tốc độ giới hạn bởi
     * pos_out_max + homing speed limit khi !init_done).
     * Dùng foc.actual_angle (MT6816, cập nhật liên tục) — KHÔNG dùng actual_angle_as5600
     * vì AS5600 không poll lúc runtime. offset cũ vẫn đúng (encoder tuyệt đối).
     * Về tới home -> init_done=1 + gửi CAN_ID_INIT_DONE (kết thúc vòng bảo vệ homing).
     * Chưa tới: cứ chờ (KHÔNG báo ready) -> STM không cho chạy lại -> an toàn. */
    if (rehome_request &&
        fabsf(foc.actual_angle - SP_INPUT_HOMING) < WAIT_ANG)
    {
        init_done = 1;
        uint8_t rehome_done_msg[1] = {0};
        FDCAN_Send(CAN_ID_INIT_DONE, rehome_done_msg, 1);
        rehome_request = 0;
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_DUALMODE_INJECSIMULT;
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_DISABLED;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_3CYCLES;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_6;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_CC4;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_FALLING;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_7;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = ENABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.GainCompensation = 0;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T4_CC3;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc3, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 10;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 13;
  hfdcan1.Init.NominalTimeSeg2 = 3;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 6;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x80A14A67;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 8499;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 200;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 169;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 169;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 99;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1699;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 169;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 199;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  EXTI1 callback — PA1 button pressed (rising edge).
  *
  * Debounce  : software 200 ms via HAL_GetTick(), rejects mechanical bounce.
  * On press  : clamp action to [SP_LIMIT_MIN, SP_LIMIT_MAX], pack as float into
  *             4-byte CAN frame and send to CAN_OBS_ID_BASE(1) = 0x034.
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != GPIO_PIN_1) return;

    exti_count ++;

    /* Software debounce */
    static uint32_t last_tick = 0;
    uint32_t now = HAL_GetTick();
    if ((now - last_tick) < 200U) return;
    last_tick = now;

    /* Clamp to valid range */
//    if (debug_action >= 100.0f) debug_action = 0.0f;
//    else if (debug_action == 0.0f) debug_action = 100.0f;
    float action = sp_input;
    if      (action > SP_LIMIT_MAX) action = SP_LIMIT_MAX;
    else if (action < SP_LIMIT_MIN) action = SP_LIMIT_MIN;

    /* Pack float → 4 bytes and send via FDCAN
     * Target ID: CAN_OBS_ID_BASE(1) = 0x033 + 1 = 0x034
     */
    uint8_t buf[4];
    memcpy(buf, &action, sizeof(float));
    FDCAN_Send(CAN_ACTION_ID_BASE(11), buf, 4);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1) {}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
