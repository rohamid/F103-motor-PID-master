/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include "PID_simple.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;


#define TIMER_MOTOR_ENC		htim1
#define TIMER_ROTARY_ENC	htim3
#define TIMER_PWM_ENC		htim2
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define	MOTOR_PPR				232	// Pulse per revolution

#define ENCODER_MAX_BUFFER		10

// Program time rate
#define PROGRAM_SAMPLE_TIME		20
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// Keep track last millis
uint32_t previousMillis = 0;
uint32_t nowMillis = 0;
uint32_t elapsedTime = 0;


uint16_t setPointBuf[10] = {10, 20, 30, 50, 100, 110, 200, 500, 400, 500};

typedef struct lpf_config_t {
	float value;
	float valueNew;
	float valueOld;
	float factor;
}lpf_config_t;

lpf_config_t rpmFilter;


pid_config_t motorPID;


/* RPM config structure */
typedef struct rpm_config_t {
	TIM_HandleTypeDef *timEncoder;
	uint32_t lastMillis;
	uint32_t nowMillis;
	uint16_t sampleTime;
	float rpm;
	uint16_t encoderVal;
}rpm_config_t;

rpm_config_t motorEncoder;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

void lpf_init(lpf_config_t *lpf, float filter_factor) {
	lpf->value = lpf->valueNew = lpf->valueOld = 0.0;
	lpf->factor = filter_factor;
}

float lpf_get_filter(lpf_config_t *lpf, float input) {
	lpf->value = input;
	lpf->valueNew = ((1.0 - lpf->factor) * lpf->valueOld) + (lpf->factor * lpf->value);
	lpf->valueOld = lpf->valueNew;

	return (lpf->valueNew);
}

void motor_rpm_init(rpm_config_t *rpm, TIM_HandleTypeDef *timEnc, uint16_t sample_time) {
	rpm->timEncoder = timEnc;
	rpm->lastMillis = 0;
	rpm->nowMillis = 0;
	rpm->rpm = 0;
	rpm->encoderVal = 0;
	rpm->sampleTime = sample_time;
}

void motor_rpm_reset(rpm_config_t *rpm) {
	rpm->lastMillis = 0;
	rpm->nowMillis = 0;
	rpm->rpm = 0;
	rpm->encoderVal = 0;
	rpm->sampleTime = 0;
}

uint16_t motor_get_rpm(rpm_config_t *rpm) {

	// Stop timer encoder in order to get a correct encoder value
	HAL_TIM_Encoder_Stop(rpm->timEncoder, TIM_CHANNEL_ALL);
	// Extract encoder value from timer counter
	rpm->encoderVal = __HAL_TIM_GET_COUNTER(rpm->timEncoder);
	// Calculate rpm
	rpm->rpm = ((rpm->encoderVal * (1000 / rpm->sampleTime)) * 60) / MOTOR_PPR;
	// Reset encoder value
	__HAL_TIM_SET_COUNTER(rpm->timEncoder, 0);
	// Restart timer encoder
	HAL_TIM_Encoder_Start(rpm->timEncoder, TIM_CHANNEL_ALL);

	return rpm->rpm;
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */


  /* Start PWM Timer */
  HAL_TIM_PWM_Start(&TIMER_PWM_ENC, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&TIMER_PWM_ENC, TIM_CHANNEL_2);

  /* Start Motor Encoder Timer */
  HAL_TIM_Encoder_Start(&TIMER_MOTOR_ENC, TIM_CHANNEL_ALL);
  // Motor encoder rpm init
  motor_rpm_init(&motorEncoder, &TIMER_MOTOR_ENC, PROGRAM_SAMPLE_TIME);
  // RPM low-pass-filter init
  lpf_init(&rpmFilter, .02);
  // PID init
  pid_init(&motorPID, 255, 5.0, 2.5, 0.2, (PROGRAM_SAMPLE_TIME / 1000.0), 0, 100);

  /* Start Rotary Encoder Timer */
  //  HAL_TIM_Encoder_Start(&TIMER_ROTARY_ENC, TIM_CHANNEL_ALL);

  float pwmMapped = 0.0;
  uint16_t rpmNew = 0;

  TIM2->CCR1 = 0;
  TIM2->CCR2 = 20;

  HAL_GPIO_WritePin(LED_ONBOARD_GPIO_Port, LED_ONBOARD_Pin, GPIO_PIN_SET);

  nowMillis = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  nowMillis = HAL_GetTick();
	  elapsedTime = nowMillis - previousMillis;
	  if(elapsedTime >= PROGRAM_SAMPLE_TIME) {

		  uint16_t rpmVal = motor_get_rpm(&motorEncoder);
		  rpmNew = lpf_get_filter(&rpmFilter, rpmVal);

		  float outpwm = pid_compute_all(&motorPID, rpmNew);
		  pwmMapped = fmap(outpwm, 0, 500, 0, 100);

//		  __HAL_TIM_SET_COMPARE(motorEncoder.timEncoder, TIM_CHANNEL_2, ((uint32_t)pwmMapped));

		  TIM2->CCR2 = (uint32_t)pwmMapped;
//		  printf("%.2f,%.2f,%.2f\r\n", setPoint, (float)rpmNew, pwmMapped);
		  printf("SP:%.2f,RPM:%.2f,PWM:%.2f\r\n", motorPID.setPoint, (float)rpmNew, pwmMapped);

		  previousMillis = nowMillis;
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
