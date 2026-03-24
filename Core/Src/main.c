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
#include "adc.h"
#include "iwdg.h"
#include "lwip.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pulse_control.h"
#include "relay_control.h"
#include "encoder_reader.h"
#include "position_control.h"
#include "ethernet_communication.h"
#include "homing.h"
#include "adc_potentiometer.h"
#include "constants.h"
#include "latency_profiler.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AUTO_FIXED_PULSE_TEST  0
#define AUTO_FIXED_PULSE_HZ    500000
#define KEYBOARD_TEST_MODE     1
#define KEYBOARD_STEP_DEG      1.0f
#define PERIODIC_CSV_LOG_ENABLE 1
#define PERIODIC_CSV_LOG_PERIOD_MS 100U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern volatile uint8_t interrupt_flag;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Latency_TryAutoReport(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint32_t g_latency_report_seq = 0U;
static float g_keyboard_target_steer_deg = 0.0f;
static char g_keyboard_line_buf[32] = {0};
static uint8_t g_keyboard_line_len = 0U;
#if PERIODIC_CSV_LOG_ENABLE
static uint8_t g_periodic_csv_enabled = 1U;
#endif

static float TargetSteeringDegToMotorDeg(float steering_deg)
{
    return SteeringDegToMotorDeg(steering_deg);
}

static float TargetMotorDegToSteeringDeg(float motor_deg)
{
    return MotorDegToSteeringDeg(motor_deg);
}

static const char* Main_CommandStateName(CommandState_t state)
{
    switch (state) {
    case CMD_ACTIVE:
        return "ACTIVE";
    case CMD_REACHED:
        return "REACHED";
    case CMD_TIMEOUT:
        return "TIMEOUT";
    case CMD_ABORTED:
        return "ABORTED";
    case CMD_FAULTED:
        return "FAULTED";
    case CMD_IDLE:
    default:
        return "IDLE";
    }
}

static const char* Main_CommandResultName(CommandResult_t result)
{
    switch (result) {
    case CMD_RESULT_REACHED:
        return "REACHED";
    case CMD_RESULT_TIMEOUT:
        return "TIMEOUT";
    case CMD_RESULT_ESTOP:
        return "ESTOP";
    case CMD_RESULT_DISABLED:
        return "DISABLED";
    case CMD_RESULT_REPLACED:
        return "REPLACED";
    case CMD_RESULT_FAULT_LIMIT:
        return "FAULT_LIMIT";
    case CMD_RESULT_FAULT_TRACKING:
        return "FAULT_TRACKING";
    case CMD_RESULT_NONE:
    default:
        return "NONE";
    }
}

#if PERIODIC_CSV_LOG_ENABLE
static void PeriodicCsv_PrintHeader(void)
{
    printf("CSV_HEADER,ms,mode,target_deg,current_deg,error_deg,output,dir,enc,cmd_id,cmd_state,cmd_result\r\n");
}

static void PeriodicCsv_Task(void)
{
    static uint32_t last_ms = 0U;
    uint32_t now_ms = HAL_GetTick();
    PositionControl_State_t s = PositionControl_GetState();
    CommandLifecycle_t cmd = PositionControl_GetCommandLifecycle();
    GPIO_PinState dir_state = HAL_GPIO_ReadPin(DIR_PIN_GPIO_Port, DIR_PIN_Pin);
    uint16_t enc_raw = EncoderReader_GetRawCounter();

    if (g_periodic_csv_enabled == 0U) {
        return;
    }

    if ((uint32_t)(now_ms - last_ms) < PERIODIC_CSV_LOG_PERIOD_MS) {
        return;
    }
    last_ms = now_ms;

    printf("CSV,%lu,%d,%.3f,%.3f,%.3f,%.0f,%d,%u,%lu,%d,%d\r\n",
           (unsigned long)now_ms,
           (int)PositionControl_GetMode(),
           TargetMotorDegToSteeringDeg(s.target_angle),
           TargetMotorDegToSteeringDeg(s.current_angle),
           TargetMotorDegToSteeringDeg(s.error),
           s.output,
           (int)dir_state,
           (unsigned int)enc_raw,
           (unsigned long)cmd.command_id,
           (int)cmd.state,
           (int)cmd.result);
}
#else
#define PeriodicCsv_PrintHeader() ((void)0)
#define PeriodicCsv_Task() ((void)0)
#endif

static void Latency_TryAutoReport(void)
{
#if LATENCY_AUTO_REPORT_ENABLE
    static uint32_t check_div = 0U;
    uint32_t i = 0U;
    uint32_t miss_count = 0U;
    LatencyStageStats_t stats = {0};

    if (++check_div < 10U) {
        return;
    }
    check_div = 0U;

    for (i = 0U; i < (uint32_t)LAT_STAGE_COUNT; i++) {
        if (LatencyProfiler_GetStageSampleCount((LatencyStage_t)i) < LATENCY_AUTO_REPORT_SAMPLES) {
            return;
        }
    }

    miss_count = LatencyProfiler_GetDeadlineMissCount();
    printf("LATENCY_BATCH_BEGIN,seq=%lu,samples=%u,core_hz=%lu,deadline_miss=%lu\r\n",
           (unsigned long)g_latency_report_seq,
           (unsigned int)LATENCY_AUTO_REPORT_SAMPLES,
           (unsigned long)SystemCoreClock,
           (unsigned long)miss_count);

    for (i = 0U; i < (uint32_t)LAT_STAGE_COUNT; i++) {
        if (LatencyProfiler_GetStageStats((LatencyStage_t)i, &stats)) {
            printf("LATENCY_STAGE,seq=%lu,name=%s,count=%lu,avg_cycles=%lu,p99_cycles=%lu,max_cycles=%lu,avg_us=%.3f,p99_us=%.3f,max_us=%.3f\r\n",
                   (unsigned long)g_latency_report_seq,
                   LatencyProfiler_StageName((LatencyStage_t)i),
                   (unsigned long)stats.sample_count,
                   (unsigned long)stats.avg_cycles,
                   (unsigned long)stats.p99_cycles,
                   (unsigned long)stats.max_cycles,
                   stats.avg_us,
                   stats.p99_us,
                   stats.max_us);
        }
    }

    printf("LATENCY_BATCH_END,seq=%lu\r\n", (unsigned long)g_latency_report_seq);

    g_latency_report_seq++;
    LatencyProfiler_Reset();
#endif
}

#if KEYBOARD_TEST_MODE
static float KeyboardTest_ClampSteeringDeg(float steering_deg)
{
    if (steering_deg > MAX_STEERING_ANGLE) {
        return MAX_STEERING_ANGLE;
    }
    if (steering_deg < MIN_STEERING_ANGLE) {
        return MIN_STEERING_ANGLE;
    }
    return steering_deg;
}

static void KeyboardTest_ClearLine(void)
{
    g_keyboard_line_len = 0U;
    g_keyboard_line_buf[0] = '\0';
}

static void KeyboardTest_PrintControlSnapshot(const char *reason)
{
    PositionControl_State_t s = PositionControl_GetState();
    CommandLifecycle_t cmd = PositionControl_GetCommandLifecycle();
    GPIO_PinState dir_state = HAL_GPIO_ReadPin(DIR_PIN_GPIO_Port, DIR_PIN_Pin);
    uint16_t enc_raw = EncoderReader_GetRawCounter();

    printf("[KB][%s] T=%.2fdeg C=%.2fdeg E=%.2fdeg O=%.0f DIR=%d ENC=%u CMD=%lu/%s/%s\r\n",
           reason,
           TargetMotorDegToSteeringDeg(s.target_angle),
           TargetMotorDegToSteeringDeg(s.current_angle),
           TargetMotorDegToSteeringDeg(s.error),
           s.output,
           (int)dir_state,
           (unsigned int)enc_raw,
           (unsigned long)cmd.command_id,
           Main_CommandStateName(cmd.state),
           Main_CommandResultName(cmd.result));
}

static void KeyboardTest_ApplyTarget(void)
{
    float motor_target_deg = SteeringDegToMotorDeg(g_keyboard_target_steer_deg);
    int ret = PositionControl_SetTargetWithSource(motor_target_deg, CMD_SRC_KEYBOARD);

    printf("[KB] target steer=%.1f deg motor=%.1f deg ret=%d\r\n",
           g_keyboard_target_steer_deg,
           motor_target_deg,
           ret);
    KeyboardTest_PrintControlSnapshot("target");
}

static void KeyboardTest_PrintHelp(void)
{
    printf("[KB] A:left D:right S:center E:enable Q:disable X:estop P:print L:csv H:help step=%.1f deg\r\n",
           KEYBOARD_STEP_DEG);
    printf("[KB] numeric target: type steering deg then Enter. ex) 5, -3.5, 0\r\n");
}

static void KeyboardTest_ApplyTypedTarget(void)
{
    char *end_ptr = NULL;
    float typed_target_deg = 0.0f;

    g_keyboard_line_buf[g_keyboard_line_len] = '\0';
    typed_target_deg = strtof(g_keyboard_line_buf, &end_ptr);

    if (end_ptr == g_keyboard_line_buf || *end_ptr != '\0') {
        printf("[KB] invalid target \"%s\"\r\n", g_keyboard_line_buf);
        KeyboardTest_ClearLine();
        return;
    }

    g_keyboard_target_steer_deg = KeyboardTest_ClampSteeringDeg(typed_target_deg);
    KeyboardTest_ApplyTarget();
    KeyboardTest_ClearLine();
}

static void KeyboardTest_ProcessInput(void)
{
    uint8_t ch = 0U;

    if (HAL_UART_Receive(&huart3, &ch, 1, 0U) != HAL_OK) {
        return;
    }

    switch (ch) {
    case '\r':
    case '\n':
        if (g_keyboard_line_len > 0U) {
            KeyboardTest_ApplyTypedTarget();
        }
        break;

    case '\b':
    case 0x7FU:
        if (g_keyboard_line_len > 0U) {
            g_keyboard_line_len--;
            g_keyboard_line_buf[g_keyboard_line_len] = '\0';
        }
        break;

    case 'a':
    case 'A':
        KeyboardTest_ClearLine();
        g_keyboard_target_steer_deg = KeyboardTest_ClampSteeringDeg(
            g_keyboard_target_steer_deg - KEYBOARD_STEP_DEG);
        KeyboardTest_ApplyTarget();
        break;

    case 'd':
    case 'D':
        KeyboardTest_ClearLine();
        g_keyboard_target_steer_deg = KeyboardTest_ClampSteeringDeg(
            g_keyboard_target_steer_deg + KEYBOARD_STEP_DEG);
        KeyboardTest_ApplyTarget();
        break;

    case 's':
    case 'S':
        KeyboardTest_ClearLine();
        g_keyboard_target_steer_deg = 0.0f;
        KeyboardTest_ApplyTarget();
        break;

    case 'e':
    case 'E':
        KeyboardTest_ClearLine();
        PositionControl_Enable();
        printf("[KB] control enabled\r\n");
        break;

    case 'q':
    case 'Q':
        KeyboardTest_ClearLine();
        PositionControl_Disable();
        printf("[KB] control disabled\r\n");
        break;

    case 'x':
    case 'X':
        KeyboardTest_ClearLine();
        PositionControl_EmergencyStop();
        printf("[KB] emergency stop\r\n");
        break;

    case 'p':
    case 'P':
        KeyboardTest_ClearLine();
        KeyboardTest_PrintControlSnapshot("snapshot");
        break;

    case 'l':
    case 'L':
        KeyboardTest_ClearLine();
#if PERIODIC_CSV_LOG_ENABLE
        g_periodic_csv_enabled = (uint8_t)(g_periodic_csv_enabled == 0U ? 1U : 0U);
        printf("[KB] csv log %s\r\n", (g_periodic_csv_enabled != 0U) ? "enabled" : "disabled");
        if (g_periodic_csv_enabled != 0U) {
            PeriodicCsv_PrintHeader();
        }
#else
        printf("[KB] csv log feature disabled at build time\r\n");
#endif
        break;

    case 'h':
    case 'H':
        KeyboardTest_ClearLine();
        KeyboardTest_PrintHelp();
        break;

    default:
        if ((ch >= '0' && ch <= '9') || ch == '-' || ch == '+' || ch == '.') {
            if (g_keyboard_line_len < (uint8_t)(sizeof(g_keyboard_line_buf) - 1U)) {
                g_keyboard_line_buf[g_keyboard_line_len++] = (char)ch;
                g_keyboard_line_buf[g_keyboard_line_len] = '\0';
            } else {
                printf("[KB] input too long\r\n");
                KeyboardTest_ClearLine();
            }
        } else {
            printf("[KB] unknown key '%c' (0x%02X)\r\n", (char)ch, (unsigned int)ch);
            KeyboardTest_ClearLine();
        }
        break;
    }
}
#endif

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
  LatencyProfiler_Init(SystemCoreClock);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_IWDG_Init();
  MX_TIM4_Init();
  MX_LWIP_Init();
  /* USER CODE BEGIN 2 */

  // PE10을 direction line driver 입력용 GPIO Output으로 명시 재설정
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

  Relay_Init();        // EMG 핀 HIGH 설정 (릴레이 24V 없어도 GPIO 상태만 설정됨)
  PulseControl_Init(); // DIR 핀 초기화 (PWM은 여기서 시작하지 않음)
  EncoderReader_Init();
  PositionControl_Init();

  // [BUG FIX] HAL_TIM_PWM_Start 제거:
  // 이전에 이 줄이 ARR=9(83kHz), DIR=CCW 상태로 PWM을 조기 시작하여
  // ServoOn 직후 모터가 무제어 역방향 고속 회전하는 문제가 있었음.
  // PWM은 PulseControl_SetFrequency() 내부에서 방향/속도 설정 후 시작됨.

  // 초기 구동 시 서보를 ON 상태로 올린다.
  // EMG 동작은 런타임 fail-safe / EmergencyStop 경로에서 처리된다.
  Relay_ServoOn();
  HAL_Delay(500); // 서보 ON 안정화 대기

  char msg[] = "Servo Start!\r\n";
  HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);

  //Homing_Init(); // 초기 위치로 이동 (예: 0도)
  //ADC_Pot_Init(NULL); // ADC 포텐셔미터 초기화 (필요 시)
  //Homing_FindZero();  
  //if (!Homing_IsComplete()) {
  //    printf("[Main] Homing failed!\n");
  //    while (1);
  //}
  //HAL_Delay(500);
  EncoderReader_Reset(); // 엔코더 카운터 리셋 (0점 기준)
  PositionControl_SetTargetWithSource(TargetSteeringDegToMotorDeg(0.0f), CMD_SRC_LOCALTEST); // 외부 조향각 0° -> 내부 motor_deg
  g_keyboard_target_steer_deg = 0.0f;
  PositionControl_Enable();

#if KEYBOARD_TEST_MODE
  KeyboardTest_PrintHelp();
  PeriodicCsv_PrintHeader();
#else
  EthComm_UDP_Init(); // UDP 수신 소켓 열기 (MX_LWIP_Init 이후 호출 필수)
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t debug_cnt = 0;
#if !KEYBOARD_TEST_MODE
  SteerMode_t prev_mode = STEER_MODE_NONE;
#endif
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if KEYBOARD_TEST_MODE
    LAT_BEGIN(LAT_STAGE_COMMS);
    KeyboardTest_ProcessInput();
    LAT_END(LAT_STAGE_COMMS);
#else
    /* ── LwIP 폴링: 이더넷 패킷 수신 처리 ── */
    LAT_BEGIN(LAT_STAGE_COMMS);
    MX_LWIP_Process();

    SteerMode_t mode = EthComm_GetCurrentMode();
    uint32_t now_ms = HAL_GetTick();
    uint32_t last_rx_ms = EthComm_GetLastRxTick();

    /* ── 통신 타임아웃 fail-safe: AUTO/MANUAL에서 RX 끊기면 ESTOP ── */
    if ((mode == STEER_MODE_AUTO || mode == STEER_MODE_MANUAL) &&
        ((now_ms - last_rx_ms) > ETHCOMM_RX_TIMEOUT_MS)) {
        EthComm_ForceMode(STEER_MODE_ESTOP);
        mode = STEER_MODE_ESTOP;
    }

    /* ── 모드 전이 처리 ── */
    if (mode == STEER_MODE_ESTOP) {
        if (prev_mode != STEER_MODE_ESTOP) {
            PositionControl_EmergencyStop();
        }
    } else if (mode == STEER_MODE_NONE) {
        if (prev_mode != STEER_MODE_NONE) {
            PositionControl_Disable();
        }
    } else if ((mode == STEER_MODE_AUTO || mode == STEER_MODE_MANUAL) &&
               (prev_mode == STEER_MODE_NONE || prev_mode == STEER_MODE_ESTOP)) {
        Relay_EmergencyRelease();
        PositionControl_Enable();
    }

    /* ── PC misc brake 등 one-shot ESTOP 요청 처리 ── */
    if (EthComm_ConsumeEmergencyRequest()) {
        PositionControl_EmergencyStop();
        mode = STEER_MODE_ESTOP;
    }

    /* ── UDP 수신 데이터 → 모드별 목표 갱신 ── */
    if (EthComm_HasNewData()) {
        AutoDrive_Packet_t pkt = EthComm_GetLatestData();
        if (mode == STEER_MODE_AUTO || mode == STEER_MODE_MANUAL) {
            PositionControl_SetTargetWithSource(TargetSteeringDegToMotorDeg(pkt.steering_angle), CMD_SRC_UDP);
        }
    }
    LAT_END(LAT_STAGE_COMMS);
    prev_mode = mode;
#endif

    /* ── 1ms 제어 루프 ── */
    if (interrupt_flag) {
        interrupt_flag = 0;
#if AUTO_FIXED_PULSE_TEST
        if (mode == STEER_MODE_AUTO) {
            PulseControl_SetFrequency(AUTO_FIXED_PULSE_HZ);
        } else {
            PulseControl_Stop();
        }
#else
        PositionControl_Update();
#endif

        if (++debug_cnt >= 100) {
            uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
            uint32_t ccr = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1);
            uint32_t enc_raw = __HAL_TIM_GET_COUNTER(&htim4);
            GPIO_PinState dir_state = HAL_GPIO_ReadPin(DIR_PIN_GPIO_Port, DIR_PIN_Pin);
            PositionControl_State_t s = PositionControl_GetState();
            CommandLifecycle_t cmd = PositionControl_GetCommandLifecycle();
            float target_steer_deg = TargetMotorDegToSteeringDeg(s.target_angle);
            float current_steer_deg = TargetMotorDegToSteeringDeg(s.current_angle);
            float error_steer_deg = TargetMotorDegToSteeringDeg(s.error);
            debug_cnt = 0;
#if LATENCY_LOG_ENABLE
            printf("[DIAG] MODE:%d CMD:%lu/%s/%s Tst:%.2f Cst:%.2f Est:%.2f O:%.0f ARR:%lu CCR:%lu DIR:%d ENC:%lu\r\n",
                   (int)PositionControl_GetMode(),
                   (unsigned long)cmd.command_id,
                   Main_CommandStateName(cmd.state),
                   Main_CommandResultName(cmd.result),
                   target_steer_deg,
                   current_steer_deg,
                   error_steer_deg,
                   s.output,
                   (unsigned long)arr,
                   (unsigned long)ccr,
                   (int)dir_state,
                   (unsigned long)enc_raw);
#else
            (void)arr;
            (void)ccr;
            (void)enc_raw;
            (void)dir_state;
            (void)s;
            (void)cmd;
            (void)target_steer_deg;
            (void)current_steer_deg;
            (void)error_steer_deg;
#endif
        }
    }

    PeriodicCsv_Task();
    Latency_TryAutoReport();
    HAL_IWDG_Refresh(&hiwdg);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

/* USER CODE BEGIN 4 */

// printf UART 리다이렉션
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

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
