/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "User_code/dxl/xl330_hal.h"
#include "User_code/delay.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* ==========================================================================
 * CURRENT CONTROL MODE (operating mode 0).
 *
 * Ra lệnh bằng DÒNG ĐIỆN (mA), không phải tốc độ. Dòng tỉ lệ thẳng với
 * mô-men, nên cái bạn đặt ở đây là LỰC ĐẨY, còn motor quay nhanh chậm bao
 * nhiêu là do tải quyết định. Không tải thì nó quay vọt lên hết tốc.
 * --------------------------------------------------------------------------
 * Bản test lực bằng tạ treo nằm ở Core/Src/main_ftest.c.bak.
 * ========================================================================== */

/*
 * Toàn bộ trạng thái gom vào một struct.
 * STM32CubeIDE: Debug > Live Expressions > gõ "motor_run" rồi bung cây ra.
 * Sửa goal_ma / target_idx ngay lúc đang chạy, không cần nạp lại.
 */
typedef struct {
    /* ---- SỬA ĐƯỢC lúc đang chạy ---- */
    int16_t  goal_ma;         /* dòng mong muốn, mA. ĐÂY LÀ CÔNG TẮC:
                               * 0 = dừng, khác 0 = chạy, âm = chiều ngược */
    uint8_t  all_motors;      /* 1 = chạy cả dàn, 0 = chỉ con target_idx   */
    uint8_t  target_idx;      /* chỉ số trong motor_ids[], KHÔNG phải ID   */

    /* ---- firmware tự điền, chỉ để xem ---- */
    uint8_t  target_id;       /* ID thật của motor đang chạy               */
    int16_t  applied_ma;      /* dòng thực sự gửi xuống motor, sau khi kẹp */
    uint16_t cur_limit_ma;    /* Current Limit đọc từ thanh ghi 38         */
    uint8_t  clamped;         /* 1 = lệnh đã bị kẹp xuống trần             */
    int32_t  bus_status;      /* XL330_StatusTypeDef của lần giao tiếp cuối*/

    /* ---- an toàn ---- */
    uint8_t  fault;           /* != 0 = firmware ĐÃ TỰ DỪNG, xem RUN_FAULT_*
                               * Xoá tay về 0 mới cho chạy lại.            */

    /* ---- đọc sống cả dàn, cập nhật mỗi RUN_SAMPLE_MS ---- */
    uint8_t  motor_n;
    uint8_t  all_ok[XL330_MAX_MOTORS];       /* 1 = lượt đọc GẦN NHẤT tốt  */
    int32_t  all_pos[XL330_MAX_MOTORS];
    int32_t  all_vel[XL330_MAX_MOTORS];      /* raw, 1 = 0.229 rev/min     */
    int16_t  all_cur[XL330_MAX_MOTORS];      /* dòng ĐO ĐƯỢC, mA           */
    uint8_t  hw_error[XL330_MAX_MOTORS];     /* thanh ghi 70, chỉ đọc khi
                                              * motor bật cờ alert         */
} MotorRun_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ===== CHỈNH THEO GIÀN CƠ KHÍ CỦA BẠN ==================================== */

/*
 * Trần dòng do FIRMWARE áp, mA. ĐÂY LÀ NÚM AN TOÀN CHÍNH của current mode:
 * dòng tỉ lệ thẳng với mô-men, nên con số này chính là trần lực kẹp/lực kéo.
 *
 * Cố ý tách khỏi Current Limit (thanh ghi 38) của motor: thanh ghi đó nằm ở
 * EEPROM, sửa là đổi vĩnh viễn cấu hình con motor và có giới hạn số lần ghi.
 * Số này sửa thoải mái. Firmware kẹp theo cái NHỎ HƠN trong hai cái.
 *
 * 300mA là mức dè dặt để dò giàn. Đo lực thật rồi hãy nâng.
 */
#define RUN_MAX_MA              300

/*
 * Dòng đặt sẵn lúc khởi động, mA.
 *
 * ĐỂ 0 trừ khi bạn thật sự cần motor đẩy ngay lúc cấp nguồn — lúc đó chưa ai
 * kịp cầm giàn cơ khí. Để 0 còn tránh được một cái bẫy: mọi giá trị bạn gõ
 * vào motor_run.goal_ma lúc debug đều khác 0, nên chắc chắn được ghi nhận là
 * một thay đổi và gửi xuống motor ngay.
 */
#define RUN_STARTUP_MA          0

/* ========================================================================= */

/* ===== AN TOÀN ============================================================ */

/*
 * Bus Watchdog của chính motor (thanh ghi 98), đơn vị 20ms, 0 = tắt.
 *
 * Đây là lớp bảo vệ QUAN TRỌNG NHẤT: không nhận được gói nào trong khoảng
 * này thì motor TỰ đặt goal về 0, không cần STM32 làm gì. Nó chặn đúng cái
 * kịch bản tệ nhất — MCU treo / reset / tuột dây UART mà motor vẫn đẩy mãi.
 *
 * Ở current mode nó còn đáng giá hơn ở velocity mode: dòng đặt cố định mà
 * mất tải là motor quay vọt lên hết tốc, và Velocity Limit KHÔNG có tác dụng
 * ở mode này. Không có gì chặn tốc độ ngoài chính cái tải.
 *
 * 10 x 20ms = 200ms. Đừng đặt quá sát: một lượt đọc hỏng đã tốn tới ~40-60ms
 * vì DXL_DEFAULT_TIMEOUT_MS = 20ms và sync read hỏng thì thư viện thử lại.
 */
#define RUN_WATCHDOG_20MS       10u

/* Số lượt đọc hỏng LIÊN TIẾP trước khi tự dừng. 10 x 20ms = 200ms. */
#define RUN_FAULT_LIMIT         10u

/* Giá trị của motor_run.fault. */
#define RUN_FAULT_NONE          0u
#define RUN_FAULT_BUS           1u   /* mất liên lạc với motor            */
#define RUN_FAULT_DEVICE        2u   /* motor báo lỗi phần cứng (alert)   */

/* ========================================================================= */

/*
 * Không có Profile Acceleration ở đây.
 * Profile Velocity/Acceleration chỉ có tác dụng ở các mode điều khiển vị trí
 * và velocity; current mode bỏ qua hoàn toàn, nên lệnh dòng là một bậc thang,
 * đổi phát ăn ngay. Cần vuốt mềm thì phải tự làm bằng phần mềm.
 */

/* Chu kỳ đọc trạng thái cả dàn (ms). 20ms = 50Hz. */
#define RUN_SAMPLE_MS           20u

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CRC_HandleTypeDef hcrc;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
#define MOTOR_1_ID 1u
#define MOTOR_2_ID 2u
#define MOTOR_3_ID 3u
#define MOTOR_5_ID 5u

/*
 * Toàn bộ motor trên bus, khai báo ở ĐÚNG MỘT CHỖ NÀY.
 * Thêm/bớt ID là đủ, phần còn lại tự theo. Tối đa XL330_MAX_MOTORS (8) con.
 */
/* Quét bus 2026-08-10 thấy 4 con: ID 1, 2, 3, 5. ID 0/4/6 không đáp. */
static const uint8_t motor_ids[] = { MOTOR_1_ID, MOTOR_2_ID, MOTOR_3_ID, MOTOR_5_ID };
#define MOTOR_N ((uint8_t)(sizeof(motor_ids) / sizeof(motor_ids[0])))

/* Motor chọn sẵn lúc khởi động: chỉ số trong motor_ids[], KHÔNG phải ID. */
#define RUN_DEFAULT_IDX         2u

XL330_HandleTypeDef dxl;
XL330_StatusTypeDef motor_status;

/* Kết quả ping lúc khởi động: 1 = ID đó có thật trên bus. */
static uint8_t motor_present[XL330_MAX_MOTORS];

volatile MotorRun_t motor_run;

/* Ảnh của lệnh đã áp dụng, để biết lúc nào cần gửi lại xuống bus. */
static int16_t run_last_ma;
static uint8_t run_last_idx;
static uint8_t run_last_all;

/* Số lượt đọc hỏng liên tiếp. Đọc tốt một lượt là về 0. */
static uint8_t run_fault_streak;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */
static void    run_stop(uint8_t fault_code);
static void    run_apply(void);
static void    run_sample(void);
static void    run_init(void);
static void    run_poll(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * Dừng an toàn. Chỉ ghi cờ; lượt run_poll() ngay sau đó thấy run đổi và gửi
 * goal 0mA xuống bus.
 *
 * COI CHỪNG: ở current mode, 0mA là KHÔNG SINH MÔ-MEN, tức là thả lỏng trục.
 * Đây không phải "dừng có hãm" như velocity mode. Tải đang treo trên trục sẽ
 * rơi theo trọng lực khi dừng. Giàn nào có tải treo thì phải có hãm cơ khí
 * hoặc phải đổi sang chế độ giữ vị trí, firmware không giữ hộ được.
 *
 * Muốn chạy lại phải xoá tay motor_run.fault về 0 — cố ý bắt người vận hành
 * nhìn thấy đã có sự cố, thay vì tự hồi phục rồi lại quay tiếp.
 */
static void run_stop(uint8_t fault_code)
{
    motor_run.fault   = fault_code;
    motor_run.goal_ma = 0;
}

/*
 * Gửi dòng hiện tại xuống bus. Chỉ gọi khi lệnh thay đổi, không gọi mỗi vòng
 * lặp - mỗi lần là một lượt sync write trên bus.
 *
 * Con nào không được chọn thì nhận goal 0mA. Ở current mode, 0mA là THẢ LỎNG
 * hoàn toàn (không sinh mô-men), khác hẳn velocity mode - ở đó goal 0 là đứng
 * yên có hãm. Tải treo trên con không được chọn sẽ tự rơi theo trọng lực.
 */
static void run_apply(void)
{
    uint8_t i;
    int32_t ma = 0;
    int32_t lim;

    if (motor_run.target_idx >= MOTOR_N) {
        motor_run.target_idx = 0u;
    }
    motor_run.target_id = motor_ids[motor_run.target_idx];

    /*
     * Đang có fault thì ép chính goal_ma về 0, không chỉ ép cái gửi đi.
     * Ép cả biến lệnh mới đúng: nếu để goal_ma treo giá trị cũ, lúc người ta
     * xoá fault và gõ lại đúng con số đó thì goal_ma không đổi giá trị,
     * run_poll() không thấy "có thay đổi" và motor nằm im.
     */
    if (motor_run.fault != RUN_FAULT_NONE) {
        motor_run.goal_ma = 0;
    }
    ma = motor_run.goal_ma;

    /*
     * Kẹp theo cái nhỏ hơn giữa trần firmware và Current Limit thật của motor.
     * Vượt trần motor thì nó trả lỗi Data Range và BỎ NGUYÊN lệnh - lúc đó
     * dòng giữ nguyên giá trị cũ chứ không phải về 0, nên phải tự kẹp ở đây.
     */
    lim = RUN_MAX_MA;
    if ((motor_run.cur_limit_ma > 0u) && ((int32_t)motor_run.cur_limit_ma < lim)) {
        lim = (int32_t)motor_run.cur_limit_ma;
    }

    motor_run.clamped = 0u;
    if (ma > lim) {
        ma = lim;
        motor_run.clamped = 1u;
    } else if (ma < -lim) {
        ma = -lim;
        motor_run.clamped = 1u;
    }
    motor_run.applied_ma = (int16_t)ma;

    for (i = 0u; i < MOTOR_N; i++) {
        if (!motor_present[i]) {
            continue;
        }
        /*
         * Lên dây Bus Watchdog. Ghi 0 trước là bắt buộc: watchdog đã sập một
         * lần thì motor giữ nguyên trạng thái lỗi và BỎ QUA mọi goal mới cho
         * tới khi thanh ghi 98 được ghi 0. Ghi ở đây, ngay trước khi giao
         * goal, nên không có cửa sổ nào motor quay mà không có watchdog.
         */
        XL330_WriteU8(&dxl, motor_ids[i], XL330_ADDR_BUS_WATCHDOG, 0u);
        XL330_WriteU8(&dxl, motor_ids[i], XL330_ADDR_BUS_WATCHDOG, RUN_WATCHDOG_20MS);

        XL330_SetGoalCur(&dxl, motor_ids[i],
                         (motor_run.all_motors || (i == motor_run.target_idx))
                             ? (int16_t)ma : (int16_t)0);
    }
    motor_run.bus_status = (int32_t)XL330_Commit(&dxl);

    run_last_ma  = motor_run.goal_ma;
    run_last_idx = motor_run.target_idx;
    run_last_all = motor_run.all_motors;
}

/*
 * Đọc trạng thái cả dàn.
 * XL330_GetState() chỉ trả về cache, nên phải XL330_Update() trước để nạp
 * cache từ bus. LAYOUT_DEFAULT = {CUR, VEL, POS}, đúng thứ cần.
 *
 * all_ok[] phản ánh lượt đọc GẦN NHẤT, không phải kết quả ping lúc khởi động.
 * Bắt buộc phải vậy: các mảng all_* giữ nguyên giá trị cũ khi đọc hỏng, nên
 * nếu không có cờ này thì một motor rớt giữa chừng vẫn trông như đang sống.
 * Đọc số liệu mà thấy all_ok[i] == 0 thì all_pos/all_cur[i] là số CŨ.
 */
static void run_sample(void)
{
    const XL330_StatusReport *rep;
    XL330_State st;
    uint8_t i;
    uint8_t hw;
    uint8_t alert = 0u;
    uint8_t ok;

    motor_run.bus_status =
        (int32_t)XL330_Update(&dxl, XL330_LAYOUT_DEFAULT, NULL, 0, NULL);

    /* DEVICE_ERROR = gói về đủ nhưng có con kêu lỗi. Số liệu vẫn dùng được,
     * phần lỗi để cờ alert bên dưới lo, nên KHÔNG tính là hỏng bus. */
    ok = (motor_run.bus_status == (int32_t)XL330_OK) ||
         (motor_run.bus_status == (int32_t)XL330_DEVICE_ERROR);

    for (i = 0u; i < MOTOR_N; i++) {
        if (ok && motor_present[i] &&
            (XL330_GetState(&dxl, motor_ids[i], &st) == XL330_OK)) {
            motor_run.all_ok[i]  = 1u;
            motor_run.all_pos[i] = st.present_position;
            motor_run.all_vel[i] = st.present_velocity;
            motor_run.all_cur[i] = st.present_current;

            /* alert đã nằm sẵn trong gói status của lượt đọc vừa rồi, xem nó
             * không tốn thêm gói nào. Chỉ khi nó bật mới đọc thanh ghi 70 để
             * biết lỗi gì — quá nhiệt, quá tải, quá áp... */
            rep = XL330_GetStatusReport(&dxl, motor_ids[i]);
            if ((rep != NULL) && rep->alert) {
                alert = 1u;
                hw = 0u;
                (void)XL330_ReadU8(&dxl, motor_ids[i],
                                   XL330_ADDR_HARDWARE_ERROR_STATUS, &hw);
                motor_run.hw_error[i] = hw;
            }
        } else {
            motor_run.all_ok[i] = 0u;
        }
    }

    if (!ok) {
        /* Mất liên lạc. Đủ lâu thì bỏ cuộc — dù bên motor Bus Watchdog cũng
         * đã tự dừng nó rồi, đây chỉ là lớp thứ hai để cờ fault hiện ra. */
        if (run_fault_streak < 0xFFu) {
            run_fault_streak++;
        }
        if (run_fault_streak >= RUN_FAULT_LIMIT) {
            run_stop(RUN_FAULT_BUS);
        }
    } else {
        run_fault_streak = 0u;
        if (alert) {
            run_stop(RUN_FAULT_DEVICE);
        }
    }
}

/*
 * Vào velocity mode. Gọi ĐÚNG MỘT LẦN sau khi đã AddMotor xong.
 *
 * Không được gọi XL330_SetMode() trong vòng lặp: nó stage vô điều kiện, và
 * XL330_Commit() mỗi lần thấy pending mode sẽ tắt torque rồi ghi EEPROM địa
 * chỉ 11. Chạy trong loop là mòn EEPROM và motor giật liên tục.
 */
static void run_init(void)
{
    uint8_t  i;
    uint16_t clim;
    uint16_t clim_min = 0u;

    motor_run.goal_ma    = 0;
    motor_run.all_motors = 0u;
    motor_run.target_idx = RUN_DEFAULT_IDX;
    motor_run.motor_n    = MOTOR_N;
    motor_run.target_id  = motor_ids[RUN_DEFAULT_IDX];
    motor_run.fault      = RUN_FAULT_NONE;
    run_fault_streak     = 0u;

    /* Giá trị ban đầu từ ping. Từ lượt run_sample() đầu tiên trở đi, all_ok[]
     * do chính lượt đọc đó quyết định. */
    for (i = 0u; i < MOTOR_N; i++) {
        motor_run.all_ok[i] = motor_present[i];
    }

    /* Current Limit thật của motor, đọc chứ không tin số mặc định trong
     * datasheet. Cả dàn dùng chung một trần nên lấy con nhỏ nhất. */
    for (i = 0u; i < MOTOR_N; i++) {
        if (!motor_present[i]) {
            continue;
        }
        if (XL330_ReadU16(&dxl, motor_ids[i], XL330_ADDR_CURRENT_LIMIT, &clim) == XL330_OK) {
            if ((clim_min == 0u) || (clim < clim_min)) {
                clim_min = clim;
            }
        }
    }
    motor_run.cur_limit_ma = clim_min;

    for (i = 0u; i < MOTOR_N; i++) {
        if (!motor_present[i]) {
            continue;
        }
        XL330_SetMode(&dxl, motor_ids[i], XL330_MODE_CURRENT);
        XL330_SetTorque(&dxl, motor_ids[i], XL330_ENABLE);
        XL330_SetGoalCur(&dxl, motor_ids[i], 0);
    }
    motor_run.bus_status = (int32_t)XL330_Commit(&dxl);

    /* Chốt ảnh ở 0mA. Nhờ vậy MỌI giá trị khác 0 bạn gõ vào goal_ma sau này
     * đều được tính là thay đổi, kể cả lần đầu tiên. */
    run_last_ma  = 0;
    run_last_idx = motor_run.target_idx;
    run_last_all = motor_run.all_motors;

    /* Watchdog do run_apply() lên dây, không phải ở đây: lúc này goal đang là
     * 0 nên chưa có gì để bảo vệ, và nếu vừa reset giữa lúc motor đang quay
     * thì watchdog của phiên trước vẫn còn hiệu lực (thanh ghi 98 ở RAM của
     * motor, còn nguyên chừng nào motor còn điện). */

    motor_run.goal_ma = RUN_STARTUP_MA;
}

/* Gọi mỗi vòng lặp. Gửi lệnh khi có thay đổi, đọc trạng thái theo chu kỳ. */
static void run_poll(void)
{
    static uint32_t next_sample_ms;
    uint32_t now = HAL_GetTick();

    if ((motor_run.goal_ma    != run_last_ma)  ||
        (motor_run.target_idx != run_last_idx) ||
        (motor_run.all_motors != run_last_all)) {
        run_apply();
    }

    if ((int32_t)(now - next_sample_ms) >= 0) {
        next_sample_ms = now + RUN_SAMPLE_MS;
        run_sample();
    }
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
  MX_USART1_UART_Init();
  // Motor-only test: peripherals used only by disabled libraries are not started.
  // MX_TIM2_Init();
  // MX_ADC1_Init();
  // MX_USART2_UART_Init();
  // MX_CRC_Init();
  /* USER CODE BEGIN 2 */

    // HAL-style XL330 manager. Add motors once, then stage commands and commit.
    motor_status = XL330_InitDefault(&dxl, &huart1);

    // Dò bus TRƯỚC khi nạp, rồi chỉ nạp con nào thật sự trả lời.
    //
    // Bắt buộc phải làm vậy: XL330_AddMotor() chỉ ghi sổ, không hề chạm bus,
    // nên ID không tồn tại vẫn "nạp thành công". Mà từ 2 motor trở lên thư
    // viện chuyển XL330_Update() sang sync read — đòi MỌI ID trong danh sách
    // phải đáp. Nạp nhầm một ID ma là mọi lượt đọc đều lỗi, kể cả của những
    // con đang sống. Ping trước cũng cho luôn all_ok[] đáng tin.
    if (motor_status == XL330_OK) {
        uint8_t i;
        uint8_t added = 0u;
        DXL2_DeviceInfo info;

        for (i = 0u; i < MOTOR_N; i++) {
            if (XL330_Ping(&dxl, motor_ids[i], &info) == XL330_OK) {
                motor_present[i] = 1u;
                if (XL330_AddMotor(&dxl, motor_ids[i]) == XL330_OK) {
                    added++;
                }
            } else {
                motor_present[i] = 0u;
            }
        }
        motor_status = added ? XL330_OK : XL330_ERROR;
    }
    if (motor_status == XL330_OK) {
        run_init();
    }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // Motor chạy ở current mode: ra lệnh DÒNG (mô-men), không phải tốc độ.
      //   chạy / dừng   : motor_run.goal_ma khác 0 / bằng 0 (âm = chiều ngược)
      //   kiểm chứng    : motor_run.applied_ma là số THỰC SỰ đã gửi xuống bus
      //   dòng đo được  : motor_run.all_cur[] , mA
      //   đã tự dừng?   : motor_run.fault (xem RUN_FAULT_*), xoá về 0 để chạy lại
      //
      // run_poll() phải được gọi đều tay: nó chính là nhịp nuôi Bus Watchdog
      // của motor. Vòng lặp này đứng quá RUN_WATCHDOG_20MS x 20ms là motor
      // tự dừng — đó là tính năng, không phải lỗi. Đừng chèn việc gì chặn lâu
      // vào đây, và nhất là đừng chèn dbh_DelayMS().
      run_poll();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
  /* USER CODE END ADC1_Init 2 */

}
/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
  htim2.Init.Prescaler = 799;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
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
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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

#ifdef  USE_FULL_ASSERT
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
