/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : XL330 chạy vị trí theo hình sin, lệnh và dữ liệu qua USB CDC.
  ******************************************************************************
  * Board: STM32F103C8T6, HSE 8 MHz -> PLL x9 = 72 MHz, USB = 72/1.5 = 48 MHz.
  * USART1 PA9 half-duplex -> bus Dynamixel, baud tự dò lúc boot (dxl_discover).
  * USB FS PA11/PA12 -> PC, giao thức xem docs/protocol.md.
  * PC13 (LED) = chân trace: THẤP trong lúc vòng tick đang nói chuyện với bus.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "User_code/dxl/xl330_hal.h"
#include "User_code/pc_link.h"
#include "User_code/motion.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*
 * Chu kỳ tick (ms): một lượt ghi goal + một lượt sync read cả dàn.
 * Mỗi motor tốn ~1 gói ghi + phần của sync read ở 1 Mbps. Đo thời gian PC13
 * THẤP trên scope trước khi hạ số này — vượt chu kỳ thì tick trễ dần, pha
 * sin vẫn đúng vì Motion_Step tính theo thời gian thật.
 */
#define MOTION_TICK_MS          10u
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Motor nhận lại từ người khác: không biết trước baud/ID. Thử lần lượt các baud
 * XL330 hỗ trợ (dxl_xl330.h XL330_BAUD_*), 1 Mbps trước vì là cấu hình cũ của dàn. */
static const uint32_t dxl_bauds[] = { 1000000u, 57600u, 115200u, 2000000u, 3000000u, 4000000u, 9600u };
#define DXL_BAUD_N ((uint8_t)(sizeof(dxl_bauds) / sizeof(dxl_bauds[0])))
#define DXL_LAST_ID 252u    /* 253 = reserved, 254 = broadcast */

XL330_HandleTypeDef dxl;
uint32_t dxl_baud;       /* baud bus đang dùng, 0 = không thấy motor nào. Gửi kèm PONG */

/* Xem bằng Live Expressions. */
uint32_t bus_err;        /* số lượt ghi/đọc bus hỏng          */
uint32_t tick_overrun;   /* số tick mà bus làm quá MOTION_TICK_MS */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void dxl_discover(void);
static void motion_init(void);
static void motion_tick(uint32_t dt_ms);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
 * Tìm baud và ID của dàn. Mỗi baud: một broadcast ping (motor nào cũng trả lời,
 * ~20 ms nếu im). Baud đầu tiên có trả lời thì quét ID 0..252 ở baud đó và
 * AddMotor mọi con trả lời. Chỉ ping đúng ID có thật: từ 2 motor trở lên
 * Update() dùng sync read, một ID ma làm hỏng mọi lượt đọc của cả dàn.
 * ponytail: quét ID tốn ~253 x 20 ms = 5 s lúc boot; rút ngắn khi dàn đã cố định.
 * Không đụng EEPROM motor: firmware theo baud của motor, không ép motor theo firmware.
 * Không thấy ở baud nào: có thể motor đang Protocol 1.0 (addr 13), cần Dynamixel Wizard.
 */
static void dxl_discover(void)
{
    uint8_t b;
    uint8_t i;
    uint8_t n = 0u;
    DXL2_DeviceInfo info;
    XL330_DeviceReport rep[XL330_MAX_MOTORS];

    dxl_baud = 0u;
    for (b = 0u; b < DXL_BAUD_N; ++b) {
        huart1.Init.BaudRate = dxl_bauds[b];
        if ((HAL_HalfDuplex_Init(&huart1) != HAL_OK) || (XL330_InitDefault(&dxl, &huart1) != XL330_OK)) {
            continue;
        }
        if (XL330_Ping(&dxl, DXL_BROADCAST_ID, &info) == XL330_OK) {
            dxl_baud = dxl_bauds[b];
            break;
        }
    }
    if (dxl_baud == 0u) {
        return;
    }

    (void)XL330_Rescan(&dxl, 0u, DXL_LAST_ID, rep, XL330_MAX_MOTORS, &n);
    for (i = 0u; (i < n) && (i < XL330_MAX_MOTORS); ++i) {
        (void)XL330_AddMotor(&dxl, rep[i].id);
    }
    if (dxl.motor_count == 0u) {
        (void)XL330_AddMotor(&dxl, info.id);   /* rescan hụt nhưng broadcast đã thấy */
    }
}

/*
 * Đưa mọi motor đã AddMotor về Current-based Position (mode 5), torque TẮT.
 * Motor chỉ có lực sau lệnh MOVE enable=1 đầu tiên từ PC.
 *
 * Mode và Return Delay Time nằm ở EEPROM: đọc trước, khác mới ghi, để mỗi
 * lần cấp nguồn không tốn một lượt ghi EEPROM.
 */
static void motion_init(void)
{
    uint8_t  i;
    uint8_t  id;
    uint8_t  v8;
    uint16_t v16;
    uint32_t v32;
    MotionCh *c;

    for (i = 0u; i < dxl.motor_count; ++i) {
        id = dxl.storage.ids[i];
        c  = &motion[i];

        /* EEPROM chỉ ghi được khi torque tắt. */
        (void)XL330_WriteU8(&dxl, id, XL330_ADDR_TORQUE_ENABLE, XL330_TORQUE_DISABLE);

        /* Mặc định 250 = 500us mỗi gói trả lời, ăn gần hết tick 10ms. */
        if ((XL330_ReadU8(&dxl, id, XL330_ADDR_RETURN_DELAY_TIME, &v8) == XL330_OK) && (v8 != 0u)) {
            (void)XL330_WriteU8(&dxl, id, XL330_ADDR_RETURN_DELAY_TIME, 0u);
        }
        if ((XL330_ReadU8(&dxl, id, XL330_ADDR_OPERATING_MODE, &v8) == XL330_OK) &&
            (v8 != XL330_MODE_CURRENT_BASED_POSITION)) {
            (void)XL330_WriteU8(&dxl, id, XL330_ADDR_OPERATING_MODE,
                                XL330_MODE_CURRENT_BASED_POSITION);
        }

        /* Trần thật của motor, để pc_link kẹp lệnh MOVE. Đọc hỏng = 0 = không kẹp. */
        c->cur_lim = (XL330_ReadU16(&dxl, id, XL330_ADDR_CURRENT_LIMIT, &v16) == XL330_OK) ? v16 : 0u;
        c->vel_lim = (XL330_ReadU32(&dxl, id, XL330_ADDR_VELOCITY_LIMIT, &v32) == XL330_OK) ? v32 : 0u;

        /* Đứng yên tại chỗ: center = vị trí hiện tại, biên độ 0. */
        c->center = (XL330_ReadU32(&dxl, id, XL330_ADDR_PRESENT_POSITION, &v32) == XL330_OK)
                    ? (int32_t)v32 : 0;
        c->goal     = c->center;
        c->amp      = 0;
        c->freq_mhz = 0u;
        c->prof_vel = c->vel_lim;
        c->goal_cur = c->cur_lim;
        c->enable   = 0u;
        c->dirty    = 0u;
        c->phase    = 0.0f;
    }
}

/*
 * Một tick: áp lệnh MOVE mới (nếu có), gửi goal sin, đọc trạng thái cả dàn.
 * Profile Velocity / Goal Current / Torque chỉ ghi khi PC đổi, không ghi mỗi tick.
 */
static void motion_tick(uint32_t dt_ms)
{
    uint8_t  i;
    uint8_t  id;
    MotionCh *c;
    XL330_StatusTypeDef s;

    for (i = 0u; i < dxl.motor_count; ++i) {
        id = dxl.storage.ids[i];
        c  = &motion[i];

        if (c->dirty) {
            /* Trần trước, torque sau: lúc có lực thì trần đã nằm sẵn. */
            s = XL330_SetProfileVelocity(&dxl, id, c->prof_vel);
            if (s == XL330_OK) {
                s = XL330_WriteU16(&dxl, id, XL330_ADDR_GOAL_CURRENT, c->goal_cur);
            }
            if (s == XL330_OK) {
                s = XL330_WriteU8(&dxl, id, XL330_ADDR_TORQUE_ENABLE,
                                  c->enable ? XL330_TORQUE_ENABLE : XL330_TORQUE_DISABLE);
            }
            if (s == XL330_OK) {
                c->dirty = 0u;     /* hỏng thì để nguyên, tick sau thử lại */
            } else {
                bus_err++;
            }
        }

        if (c->enable) {
            (void)XL330_SetGoalPos(&dxl, id, Motion_Step(c, dt_ms));
        }
    }

    s = XL330_Commit(&dxl);
    if ((s != XL330_OK) && (s != XL330_NO_PENDING_CHANGES)) {
        bus_err++;
    }
    s = XL330_Update(&dxl, XL330_LAYOUT_DEFAULT, NULL, 0u, NULL);
    if ((s != XL330_OK) && (s != XL330_DEVICE_ERROR)) {
        bus_err++;
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* Ép host enumerate lại sau MỖI lần reset (lấy từ uwb_node).
   * Blue Pill nối cứng điện trở kéo lên D+, reset CPU không tạo sự kiện rút
   * cáp: host giữ địa chỉ cũ còn chip đã quên -> link chết câm.
   * Kéo D+ (PA12) xuống 20 ms để giả lập rút cáp trước khi USB init.
   * ponytail: chỉ cần khi board thiếu transistor disconnect. */
  {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef dp = { .Pin = GPIO_PIN_12, .Mode = GPIO_MODE_OUTPUT_PP,
                            .Pull = GPIO_NOPULL, .Speed = GPIO_SPEED_FREQ_LOW };
    HAL_GPIO_Init(GPIOA, &dp);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_Delay(20);
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_12);      /* trả lại cho ngoại vi USB */
  }
  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
  dxl_discover();
  motion_init();
  PcLink_Init(&dxl);
  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  {
    uint32_t last = HAL_GetTick();
    uint32_t now;

    while (1)
    {
      PcLink_Task();

      now = HAL_GetTick();
      if ((dxl.motor_count > 0u) && ((now - last) >= MOTION_TICK_MS)) {
        if ((now - last) >= 2u * MOTION_TICK_MS) {
          tick_overrun++;
        }
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        motion_tick(now - last);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        last = now;
      }
    }
  }
  /* USER CODE END WHILE */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  /* USB phải đúng 48 MHz: 72 / 1.5. */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 1000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */
