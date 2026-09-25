/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "utils.h"
#include "NRF24.h"
#include "NRF24_reg_addresses.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan2;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_rx;
DMA_HandleTypeDef hdma_spi3_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */

volatile uint8_t init_ok;

/*  =========================== BQ76952 =========================== */
BQ76952_Data_t    bq76952       = {0};
BQ76952_Debug_t   bq_dbg        = {0};

/*  =========================== CAN / BQ error tracking =========================== */
uint32_t             err       = 0;
uint8_t              debug_i2c = 5;   /* set in debugger: 0-5, see BQ76952_DiagMode_t */
BQ76952_DiagResult_t bq_diag   = {0};

/*  =========================== NRF24L01 (RX / slave) =========================== */
static uint8_t nrf_addr[5] = {0x12, 0x34, 0x56, 0x78, 0x9A};  /* phai trung master */
static uint8_t nrf_buf[4];
float    nrf_vx = 0.0f, nrf_vy = 0.0f, nrf_vz = 0.0f;   /* gia tri nhan, [-0.5, 0.5] */
uint8_t  nrf_action = 0;                                 /* 0=block, 1=bat dau kiem tra arm */
uint32_t nrf_rx_count = 0;                               /* tong so goi nhan duoc */
uint32_t nrf_bad_width = 0;                              /* goi RX sai do dai */
uint32_t nrf_last_rx  = 0;                               /* tick cua goi cuoi */
uint8_t  nrf_link_ok  = 0;                               /* 1 = dang nhan, 0 = mat ket noi */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI3_Init(void);
static void MX_CAN2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Nạp ACK payload cho NRF: khi remote (master) gửi gói tới, NRF tự trả ACK kèm
 * 5 byte trạng thái này. Payload:
 *   [0] = init_done_mask bit 0..7
 *   [1] = init_done_mask bit 8..11
 *   [2] = rx_joints_received bit 0..7
 *   [3] = rx_joints_received bit 8..11
 *   [4] = flags: bit0 robot_falling (1=falling, 0=standing), bit1 all_nodes_ready
 * Phải nạp TRƯỚC khi gói tới (ack tự gửi bằng payload đang có trong FIFO).
 * LƯU Ý: remote (master) PHẢI bật Dynamic Payload + Ack Payload mới đọc được. */
static void nrf_load_status_ack(void)
{
    uint16_t init_mask = (uint16_t)(init_done_mask & 0x0FFFu);
    uint16_t rx_mask   = (uint16_t)(rx_joints_received & 0x0FFFu);
    uint8_t  buf[5];
    buf[0] = (uint8_t)(init_mask & 0xFF);
    buf[1] = (uint8_t)((init_mask >> 8) & 0x0F);
    buf[2] = (uint8_t)(rx_mask & 0xFF);
    buf[3] = (uint8_t)((rx_mask >> 8) & 0x0F);
    buf[4] = (uint8_t)((robot_falling ? 0x01u : 0x00u) |
                       (all_nodes_ready ? 0x02u : 0x00u));
    nrf24_flush_tx();                    /* ACK payload FIFO chi giu trang thai moi nhat */
    nrf24_transmit_rx_ack_pld(0, buf, 5);
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
  MX_I2C2_Init();
  MX_SPI3_Init();
  MX_CAN2_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_SPI1_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  /*  =========================== MPU6050 Init =========================== */
  HAL_Delay(200);
  MPU6050_Init(&mpu, &hi2c1);
  MPU6050_Calibrate(&mpu);
  MPU6050_Update(&mpu);       /* Prime IMU outputs for debugger/first obs frame. */
  init_ok = 1;
  HAL_Delay(500);

  /*  =========================== BQ76952 Init =========================== */
  HAL_GPIO_WritePin(BQ_RST_SHUNT_PORT, BQ_RST_SHUNT_PIN, GPIO_PIN_RESET);
  HAL_Delay(100);

  bq76952.hi2c = &hi2c2;
  BQ76952_Diagnose(&bq76952, (BQ76952_DiagMode_t)debug_i2c,
                   GPIOB, GPIO_PIN_10, &bq_diag);

  if (bq_diag.i2c_status == HAL_OK)
  {
      BQ76952_SendSubcommand(&bq76952, BQ_SUBCMD_RESET);
      HAL_Delay(1000);

      BQ76952_Status_t init_ret = BQ76952_Init(&hi2c2, &bq76952, &bq_dbg);
      bq_dbg.cfg_ok = (init_ret == BQ_OK) ? 1 : 0;

      if (init_ret == BQ_OK)
      {
          BQ76952_DisableAllProtections(&bq76952);
          BQ76952_AllFetsOn(&bq76952);
          BQ76952_VerifyConfig(&bq76952, &bq_dbg);
          BQ76952_ReadAllData(&bq76952);
          BQ76952_UpdateDebug(&bq76952, &bq_dbg);
          HAL_GPIO_WritePin(LED_PORT, LED_L2, GPIO_PIN_SET);   /* L2: BMS OK        */
      }
      else
          HAL_GPIO_WritePin(LED_PORT, LED_L4, GPIO_PIN_SET);   /* L4: config error  */
  }
  else
      HAL_GPIO_WritePin(LED_PORT, LED_L4, GPIO_PIN_SET);       /* L4: not found     */

  HAL_Delay(50);

  /*  =========================== Jetson Output Pin Init =========================== */
  HAL_GPIO_WritePin(JETSON_OUT_PORT, JETSON_OUT_PIN, GPIO_PIN_RESET);
  jetson_out = 0;

  /*  =========================== Protection Init =========================== */
  /* Khởi tạo trạng thái có nhớ TRƯỚC khi start SPI/CAN (callback có thể chạy
   * ngay sau đó). Lớp NaN/range không cần init; slew/watchdog/stats cần.      */
  Protection_StatsReset(&prot_stats);
  Protection_SlewInit(&action_slew, PROT_ACTION_SLEW_MAX);
  Protection_WatchdogInit(&jetson_action_wd, PROT_ACTION_TIMEOUT_MS);
  jetson_action_link_ok = 0;

  /* Obs thiếu-khớp policy */
  obs_locked      = 0;
  obs_bad_streak  = 0;
  obs_good_streak = 0;

  /* Falling detection */
  robot_falling   = 0;

  /* Arming: chưa cho xuất action tới khi obs+action ổn định 5s (init/đứng dậy) */
  action_armed    = 0;

  /*  =========================== SPI3 DMA Start =========================== */
  HAL_SPI_TransmitReceive_DMA(&hspi3,
      (uint8_t *)spi_tx_buffer,
      (uint8_t *)spi_rx_buffer,
      SPI_PAYLOAD_FLOATS * sizeof(float));

  /*  =========================== CAN2 Filter & Start =========================== */
  CAN_FilterTypeDef filter = {0};
  filter.FilterActivation      = ENABLE;
  filter.FilterBank            = 14;
  filter.SlaveStartFilterBank  = 14;
  filter.FilterFIFOAssignment  = CAN_FILTER_FIFO0;
  filter.FilterMode            = CAN_FILTERMODE_IDMASK;
  filter.FilterScale           = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh          = 0x0000;
  filter.FilterIdLow           = 0x0000;
  filter.FilterMaskIdHigh      = 0x0000;
  filter.FilterMaskIdLow       = 0x0000;

  if (HAL_CAN_ConfigFilter(&hcan2, &filter) != HAL_OK) Error_Handler();
  if (HAL_CAN_ActivateNotification(&hcan2,
        CAN_IT_RX_FIFO0_MSG_PENDING |
        CAN_IT_TX_MAILBOX_EMPTY     |
        CAN_IT_ERROR                |
        CAN_IT_BUSOFF               |
        CAN_IT_LAST_ERROR_CODE      |
        CAN_IT_ERROR_WARNING        |
        CAN_IT_ERROR_PASSIVE) != HAL_OK) Error_Handler();
  if (HAL_CAN_Start(&hcan2) != HAL_OK) Error_Handler();

  err = HAL_CAN_GetError(&hcan2);

  /*  =========================== Init-done handshake ===========================
   * Hỏi 12 node đã homing/init xong chưa. TIM5 broadcast CHECK_INIT_DONE mỗi 5s
   * tới khi gom đủ 12 bit (init_done_mask == INIT_DONE_ALL_MASK). Chỉ khi đủ
   * (all_nodes_ready = 1) thì EXTI/SPI mới cho phép chu kỳ điều khiển với Jetson.
   * Node báo bằng CAN_ID_INIT_DONE = 101..112; CAN RX callback set bit + stop TIM5. */
  init_done_mask  = 0;
  all_nodes_ready = 0;
  CAN_SendCheckInitDone();              /* hỏi ngay lần đầu, không chờ 5s        */
  HAL_TIM_Base_Start_IT(&htim5);

  /*  =========================== NRF24L01 setup as RX =========================== */
  /* CSN=PA4, CE=PC5 (da cau hinh GPIO output trong MX_GPIO_Init). SPI1 master, /16. */
  nrf24_defaults();
  HAL_Delay(5);
  nrf24_init();

  nrf24_open_tx_pipe(nrf_addr);            /* auto-ack: TX addr = pipe0 addr */
  nrf24_open_rx_pipe(0, nrf_addr);
  nrf24_pipe_pld_size(0, 4);               /* 4 byte: vx, vy, vz, action */
  nrf24_set_channel(110);     /* phai trung master */
  nrf24_data_rate(_1mbps);    /* phai trung master (TEST 1Mbps) */
  nrf24_tx_pwr(_0dbm);
  nrf24_set_crc(en_crc, _1byte);
  nrf24_auto_ack(0, enable);
  /* Dynamic payload + ACK payload: cho phép NRF trả trạng thái về remote qua ACK.
   * (Remote/master PHẢI bật DPL + ack-payload tương ứng mới nhận được.)        */
  nrf24_activate_features();
  nrf24_dpl(enable);
  nrf24_set_rx_dpl(0, enable);
  nrf24_en_ack_pld(enable);
  nrf24_listen();                           /* vao che do RX, CE len HIGH */
  nrf_load_status_ack();                    /* nạp ACK payload lần đầu          */

  /* TIM2/3/4 are started on-demand from ISR callbacks — not at boot */

#if TEST
  HAL_TIM_Base_Start_IT(&htim1);
#endif

  /* USER CODE END 2 */

  spi_last_ms = HAL_GetTick();   /* mốc khởi điểm cho watchdog SPI */
  uint8_t spi_link_lost = 0;     /* cạnh: đã kích xử lý mất-SPI chưa */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    for (uint8_t nrf_drain = 0; nrf_drain < 3 && nrf24_data_available(); nrf_drain++)
    {
      uint8_t pld_width = nrf24_r_pld_wid();
      if (pld_width != 4)
      {
        nrf24_flush_rx();
        nrf24_clear_rx_dr();
        nrf_bad_width++;
        nrf_load_status_ack();
        continue;
      }

      nrf24_receive(nrf_buf, 4);

      /* moi byte = int8 co dau = gia tri*100 -> chia 100 lai.
       * Ghi THẲNG vào vel_cmd[0..2] (vx,vy,vyaw) -> obs SPI_IDX_CMD lên Jetson. */
      vel_cmd[0] = (float)((int8_t)nrf_buf[0]) / 100.0f;   /* vx   -0.5..+0.5 */
      vel_cmd[1] = (float)((int8_t)nrf_buf[1]) / 100.0f;   /* vy             */
      vel_cmd[2] = (float)((int8_t)nrf_buf[2]) / 100.0f;   /* vyaw           */
      nrf_vx = vel_cmd[0]; nrf_vy = vel_cmd[1]; nrf_vz = vel_cmd[2]; /* mirror debug */
      nrf_action = nrf_buf[3] ? 1 : 0;

      nrf_rx_count++;
      nrf_last_rx = HAL_GetTick();

      /* Nạp lại ACK payload mới nhất cho lần ACK kế tiếp. */
      nrf_load_status_ack();
    }

    /* mat ket noi neu qua 500ms khong co goi moi */
    nrf_link_ok = ((HAL_GetTick() - nrf_last_rx) < 500) ? 1 : 0;

    /* ===== Watchdog SPI: Jetson không clock SPI quá lâu -> mất link =====
     * -> về "chưa init" (STATUS=WAIT cho Jetson) + broadcast CAN 150 cho node homing.
     * Cạnh-kích: chỉ xử lý 1 lần mỗi lần mất; SPI sống lại thì re-arm. */
    if ((uint32_t)(HAL_GetTick() - spi_last_ms) > SPI_TIMEOUT_MS)
    {
      if (!spi_link_lost)
      {
        spi_link_lost   = 1;
        all_nodes_ready = 0;          /* chưa sẵn sàng -> STATUS=WAIT, chặn action */
        init_done_mask  = 0;
        action_armed    = 0;
        CAN_SendFalling();            /* gửi 150 cho node homing (3 lần cho chắc) */
        CAN_SendFalling();
        CAN_SendFalling();
        HAL_TIM_Base_Start_IT(&htim5);  /* poll lại CHECK_INIT_DONE */
      }
    }
    else
    {
      spi_link_lost = 0;
    }

    /* Protection: giam sat link action tu Jetson (watchdog, Lop 5).
     * CHI danh gia khi dang RUN — luc HOLD ta co tinh khong forward action nen
     * watchdog khong duoc feed, khong tinh la "mat link" (tranh dem nham).     */
    if (Robot_SafeLevel() == SAFE_RUN)
    {
      uint8_t link_ok = !Protection_WatchdogExpired(&jetson_action_wd, HAL_GetTick());
      if (!link_ok && jetson_action_link_ok)
      {
          prot_stats.stale_events++;     /* canh xuong: vua mat lenh           */
          dbg_action_timeout_ms = HAL_GetTick();
      }
      jetson_action_link_ok = link_ok;
    }
    else
    {
      jetson_action_link_ok = 0;         /* dang HOLD: link coi nhu chua active */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 3;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = ENABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = ENABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
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
  hspi3.Init.Mode = SPI_MODE_SLAVE;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 8399;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 399;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = 8399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9;
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
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
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
  htim3.Init.Prescaler = 8399;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9;
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
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8399;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 9;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
  HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
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
  htim5.Init.Prescaler = 16799;          /* 84MHz/16800 = 5000 Hz tick          */
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 24999;             /* 25000 tick / 5000 Hz = 5.0 s        */
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
  HAL_NVIC_SetPriority(TIM5_IRQn, 5, 0);   /* thấp hơn các ISR điều khiển (0) */
  HAL_NVIC_EnableIRQ(TIM5_IRQn);
  /* USER CODE END TIM5_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_5|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE5 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC1 PC2 PC5 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_5|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
