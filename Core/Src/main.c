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
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DMA_BUFFER_SIZE 32
// #define SAMPLE_RATE 44100
#define CLOCK_SPEED 84000000
#define PERIOD 1905 // 44.1 kHz sample rate, 84 MHz clock speed, 84e6 / 44100 = 1904.76
const float SAMPLE_RATE = (float)CLOCK_SPEED / (float)PERIOD;
#define OUTPUT_MID 2048 // 12-bit DAC, mid value is 2048
#define DEBOUNCE_MS 200
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
const float two_pi = 2 * M_PI;

uint32_t cb_counter = 0; // callback counter, incremented in timer interrupt
uint32_t cb_full = 0;
uint32_t cb_half = 0;

uint32_t last_button_press_time = 0; // last time the button was pressed, for debouncing
// volatile because it is modified in an interrupt and used in the main loop, so the compiler should not optimize it away
volatile bool is_button_pressed = 0;
bool is_timer_running = 0;

uint16_t dma_buffer[2 * DMA_BUFFER_SIZE]; // buffer for DMA transfer
// twice the dma buffer cuz we use circular mode

float angle = 0;
float angle_increment = two_pi * 440 / SAMPLE_RATE; // 440 Hz sine wave

// float phase = 0;
// float phase_increment

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_DAC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Send printf to uart2
int _write(int fd, char* ptr, int len) {
  // overwrites the built-in _write function, which is called by printf and other stdio functions
  HAL_StatusTypeDef hstatus;

  if (fd == 1 || fd == 2) {
    hstatus = HAL_UART_Transmit(&huart2, (uint8_t *) ptr, len, HAL_MAX_DELAY);
    if (hstatus == HAL_OK)
      return len;
    else
      return -1;
  }
  return -1;
}

// optimize get tick function to be inline, so it doesn't have the overhead of a function call
// only affects release
inline uint32_t HAL_GetTick(void)
{
  return uwTick;
}

// callbacks
inline void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) // timer cb
{
  if (htim->Instance == TIM6) {
    cb_counter++; // increment callback counter
  }
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) // button cb
{
    // if (GPIO_Pin == BUTTON_USER)   // or the appropriate pin macro
    if (GPIO_Pin == GPIO_PIN_13)   // or the appropriate pin macro
    {
      uint32_t now = HAL_GetTick();
      if ( now - last_button_press_time >= DEBOUNCE_MS) {
        last_button_press_time = now;
        is_button_pressed = 1;
      }
    }
}

// const float sine_table[]

static inline void do_dac(uint16_t *buffer){
  for (int i = 0; i < DMA_BUFFER_SIZE; i++) {
    // fill buffer with a sine wave
    // buffer[i] = OUTPUT_MID + (OUTPUT_MID - 1) * sinf(2 * M_PI * i / DMA_BUFFER_SIZE);

    // float amplitude = OUTPUT_MID / 30.0f; // 90% of the DAC range, to avoid clipping
    float amplitude = OUTPUT_MID * 0.9f; // 90% of the DAC range, to avoid clipping
    float volume = 0.03f;
    amplitude *= volume; // apply volume to amplitude
    /*
    don't just scale the whole thing. keep the mid value at 2048,
    otherwise DC offset will be introduced,
    which can cause the output to be clipped at the top and bottom of the DAC range.
    volume isn't just a multiplier, it's an offset too. the sine wave should oscillate around the mid value, not around 0.
    */

    float wave = sinf(angle);
    // float wave = 2.0f * (angle/two_pi) - 1.0f;
    buffer[i] = OUTPUT_MID + amplitude * wave;

    // increment angle
    angle += angle_increment;
    // wrap angle
    if (angle > two_pi) {
      angle -= two_pi; // wrap angle to 0-2pi
    }
  }
}

inline void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
  cb_full++;
  do_dac(&dma_buffer[DMA_BUFFER_SIZE]); // fill second half of buffer
}
inline void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
  cb_half++;
  do_dac(&dma_buffer[0]); // fill first half of buffer
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
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_DAC_Init();
  /* USER CODE BEGIN 2 */
  printf("Starting main loop...\n");

  // HAL_TIM_Base_Start_IT(&htim6); // start timer 6 in interrupt mode
  // HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)dma_buffer, 2 * DMA_BUFFER_SIZE, DAC_ALIGN_12B_R); // start DAC in DMA mode

  uint32_t loop_counter = 0;
  uint32_t now = HAL_GetTick();
  #define LOOP_COUNTER_LOG_INTERVAL 1000
  uint32_t next_loop_counter_log = now + LOOP_COUNTER_LOG_INTERVAL;

  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED2);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    now = HAL_GetTick();

    // if (BSP_PB_GetState(BUTTON_USER) == GPIO_PIN_SET) {
    // if (BSP_PB_GetState(BUTTON_USER)) {
    //   if(last_button_press_time)
    //   // button is pressed
    //   printf("Button pressed!\n");
    // }

    if (is_button_pressed) {
      is_button_pressed = 0;
      printf("Button pressed!\n");

      // toggle timer6
      if (is_timer_running)
      {
          // no need to start TIM in interrupt mode, because we are using DAC with DMA, which will trigger TIM6 automatically
          HAL_TIM_Base_Stop(&htim6);
          HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
      }
      else
      {
          HAL_TIM_Base_Start(&htim6);
          HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)dma_buffer, 2 * DMA_BUFFER_SIZE, DAC_ALIGN_12B_R);
      }

      is_timer_running = !is_timer_running;
    }

    // log loop counter
    if (now >= next_loop_counter_log) {
      printf("loop_counter: %lu\n", loop_counter);
      // printf("callback counter: %lu Hz\n", cb_counter);
      // printf("callback half: %lu Hz\n", cb_half);
      // printf("callback full: %lu Hz\n", cb_full);
      loop_counter = 0;
      cb_counter = 0;
      cb_half = 0;
      cb_full = 0;
      next_loop_counter_log = now + LOOP_COUNTER_LOG_INTERVAL;
      // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // toggle led using HAL function
      // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // turn on led using HAL function
      // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // turn off led using HAL function

      BSP_LED_Toggle(LED2); // toggle led using BSP function
      // BSP_LED_On(LED2); // turn on led using BSP function
      // BSP_LED_Off(LED2); // turn off led using BSP function
    }

    loop_counter++;
    

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = PERIOD - 1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
