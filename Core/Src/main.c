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
#include "latency_profiler.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AUTO_FIXED_PULSE_TEST  0
#define AUTO_FIXED_PULSE_HZ    500000

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
  LatencyProfiler_Init(SystemCoreClock);

  /* USER CODE BEGIN SysInit */

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

  // PE10(DIR)을 GPIO Output으로 명시 재설정 (방향 신호용)
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
  PositionControl_SetTarget(0.0f); // 초기 목표 0° (UDP 수신값으로 갱신됨)
  PositionControl_Enable();

  EthComm_UDP_Init(); // UDP 수신 소켓 열기 (MX_LWIP_Init 이후 호출 필수)
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t debug_cnt = 0;
  SteerMode_t prev_mode = STEER_MODE_NONE;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
            PositionControl_SetTarget(pkt.steering_angle);
        }
    }
    LAT_END(LAT_STAGE_COMMS);
    prev_mode = mode;

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
            debug_cnt = 0;
#if LATENCY_LOG_ENABLE
            printf("[DIAG] MODE:%d T:%.2f C:%.2f E:%.2f O:%.0f ARR:%lu CCR:%lu DIR:%d ENC:%lu\r\n",
                   (int)PositionControl_GetMode(),
                   s.target_angle,
                   s.current_angle,
                   s.error,
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
#endif
        }
    }

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
