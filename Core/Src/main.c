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

#include "synth.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DAC_DMA_BUFFER_SIZE 32
#define OUTPUT_MID 2048 // 12-bit DAC, mid value is 2048
#define DEBOUNCE_MS 200

#define ADC_RESOLUTION pow(2,12)
#define ADC_DMA_SAMPLES 10
#define NUM_ADC_CHANNELS 8 // just need to update this when increasing ADC channels
// #define ADC_DMA_BUFFER_SIZE 1 * ADC_DMA_SAMPLES

#define ADC_DMA_BUFFER_SIZE (NUM_ADC_CHANNELS * ADC_DMA_SAMPLES)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint32_t cb_counter = 0; // callback counter, incremented in timer interrupt

uint32_t last_button_press_time = 0; // last time the button was pressed, for debouncing
uint32_t last_OSC2_btn_press_time = 0; // last time the button was pressed, for debouncing
uint32_t last_WAVE_FORM_btn_press_time = 0; // last time the button was pressed, for debouncing
// volatile because it is modified in an interrupt and used in the main loop, so the compiler should not optimize it away
volatile bool is_blue_button_pressed = 0;
volatile bool is_OSC2_btn_pressed = 0;
volatile bool is_WAVE_FORM_btn_pressed = 0;
bool is_timer_running = 0;

uint16_t dac_dma_buffer[2 * DAC_DMA_BUFFER_SIZE]; // buffer for DMA transfer
uint16_t adc_dma_buffer[2 * ADC_DMA_BUFFER_SIZE];

volatile float adc_values[NUM_ADC_CHANNELS];

// synth effects parameters
volatile float volume = 0.05f;
volatile float pitch = 1.0f;

// twice the dma buffer cuz we use circular mode

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_DAC_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM8_Init(void);
static void MX_I2C1_Init(void);
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
  // blue button from nucleo board
  if (GPIO_Pin == GPIO_PIN_13)   // or the appropriate pin macro
  {
    uint32_t now = HAL_GetTick();
    if ( now - last_button_press_time >= DEBOUNCE_MS) {
      last_button_press_time = now;
      is_blue_button_pressed = 1;
    }
  }

  // additional buttons
  if(GPIO_Pin == WAVE_FORM_Pin)
  {
    uint32_t now = HAL_GetTick();
    if ( now - last_WAVE_FORM_btn_press_time >= DEBOUNCE_MS) {
      last_WAVE_FORM_btn_press_time = now;
      is_WAVE_FORM_btn_pressed = 1;
    }
  }
  if (GPIO_Pin == OSC2_Pin)
  {
    uint32_t now = HAL_GetTick();
    if ( now - last_OSC2_btn_press_time >= DEBOUNCE_MS) {
      last_OSC2_btn_press_time = now;
      is_OSC2_btn_pressed = 1;
    }
  }

}


void apply_effects(uint16_t *sample) {
  // apply effects to the sample

  // volume
  *sample = (uint16_t)((float)(*sample) * volume);

  // low-pass filter

  // pitch
  // *sample = (uint16_t)((float)(*sample) * pitch);
}

static inline void do_dac(uint16_t *buffer){
  float amplitude = OUTPUT_MID * 0.9f; // 90% of the DAC range, to avoid clipping

  // update volume
  volume = (float) adc_values[0] / (ADC_RESOLUTION); // scale to 0.0 - 1.0
  pitch = (float) adc_values[1] / (ADC_RESOLUTION); // scale to 0.0 - 1.0
  set_osc1_freq(440.0f * pitch); // update frequency of osc1 based on adc value

  // update pitch

  for (int i = 0; i < DAC_DMA_BUFFER_SIZE; i++) {
    //! sine wave
    // uint32_t index = phase >> 22;
    // float wave = sine_table[index];

    //! sawtooth wave
    // float wave = 2.0f * (phase / 4294967296.0f) - 1.0f; // phase is a uint32_t, so it wraps around at 2^32, which is 4294967296

    // float sample = wave * amplitude;
    float sample = synth_process() * amplitude;

    // buffer[i] = OUTPUT_MID + amplitude * wave;
    buffer[i] = OUTPUT_MID + sample;
    apply_effects(&buffer[i]);

    // increment phase
    // phase += phase_increment;
    // phase will auto wrap due to uint32_t overflow
  }
}

void process_adc_dma_buffer(uint16_t *buffer) {
  // process the adc dma buffer, which is filled by the adc dma interrupt

  // sum the samples for each channel
  uint32_t sums[NUM_ADC_CHANNELS] = {0};
  for (int sample = 0; sample < ADC_DMA_SAMPLES; sample++)
  {
      for (int ch = 0; ch < NUM_ADC_CHANNELS; ch++)
      {
          sums[ch] += buffer[sample * NUM_ADC_CHANNELS + ch];
      }
  }

  // average the samples and store in adc_value
  for (int ch = 0; ch < NUM_ADC_CHANNELS; ch++)
  {
      adc_values[ch] = sums[ch] / ADC_DMA_SAMPLES;
  }
}

// OLED TEST
#define OLED1_ADDR (0x3D << 1)

void OLED_Command(uint8_t cmd, uint16_t address)
{
    uint8_t data[2] = {0x00, cmd};
    // 0x00 is the control byte used by SH1106, followed by the command byte

    HAL_I2C_Master_Transmit(
        &hi2c1,
        address,
        data,
        2,
        HAL_MAX_DELAY
    );
}

void OLED_Init(uint16_t address)
{
  HAL_Delay(100);

  OLED_Command(0xAE, address); // Display OFF

  OLED_Command(0xD5, address);
  OLED_Command(0x80, address);

  OLED_Command(0xA8, address);
  OLED_Command(0x3F, address);

  OLED_Command(0xD3, address);
  OLED_Command(0x00, address);

  OLED_Command(0x40, address);

  OLED_Command(0x8D, address);
  OLED_Command(0x14, address); // Charge pump

  OLED_Command(0x20, address);
  OLED_Command(0x00, address); // Horizontal addressing mode

  OLED_Command(0xA1, address); // Segment remap
  OLED_Command(0xC8, address); // COM scan direction

  OLED_Command(0xDA, address);
  OLED_Command(0x12, address);

  OLED_Command(0x81, address);
  OLED_Command(0x7F, address);

  OLED_Command(0xD9, address);
  OLED_Command(0xF1, address);

  OLED_Command(0xDB, address);
  OLED_Command(0x40, address);

  OLED_Command(0xA4, address);
  OLED_Command(0xA6, address);

  OLED_Command(0xAF, address); // Display ON
}

uint16_t load_dac_buffer[4*DAC_DMA_BUFFER_SIZE] = {0}; // 4*32=128 fits the oled resolution nicely
// uint16_t load_dac_buffer_index = 0;
uint8_t oled_data[8][129] = {0b00000000};
uint32_t render_oled_count = 0;
typedef enum{
    FILL_FIRST_QUARTER_OF_BUFFER,
    FILL_SECOND_QUARTER_OF_BUFFER,
    FILL_THIRD_QUARTER_OF_BUFFER,
    FILL_FOURTH_QUARTER_OF_BUFFER,
    START_RENDER,
} RenderStep; // keeps track which step of the render is the system currently doing
volatile RenderStep curr_render_step = FILL_FIRST_QUARTER_OF_BUFFER;
void render_oled(void){
  render_oled_count++;

  // clear oled data first
  for (int i = 0; i < 8*129; i++) {
      ((uint8_t*)oled_data)[i] = 0;
  }

  // fill with new data
  // for (int i = 0; i < (2*DAC_DMA_BUFFER_SIZE); i++) {
  //   // map to oled_data
  //   uint16_t value = load_dac_buffer[i];
  //   uint16_t page = 7 - (value>>9);
  //   // uint16_t page = 7 - (value>>9);
  //   uint8_t command = 1 << (7-((value+1)%8)); // 2**(8-value mod 8)
  //   oled_data[page][i+1] = command;
  // }

      for (int i = 0; i < (4 * DAC_DMA_BUFFER_SIZE); i++) {
      uint16_t value = load_dac_buffer[i];
      // Map 12-bit DAC value to 0-63 OLED Y coordinate
      uint8_t y = value * 63 / 4095;
      uint8_t page = y / 8;
      uint8_t bit  = y % 8;
      oled_data[page][i + 1] |= (1 << bit);
  }

  // send i2c command to oled
  for (int page = 0; page < 8; page++){
    // for (int i = 1; i < 129; i++)
    //     oled_data[page][i] = 0x00;

    oled_data[page][0] = 0x40; // 0x40 = following bytes are display data
    OLED_Command(0xB0 + page, OLED1_ADDR);
    OLED_Command(0x00, OLED1_ADDR);
    OLED_Command(0x10, OLED1_ADDR);
    
    HAL_I2C_Master_Transmit(
      &hi2c1,
      OLED1_ADDR,
      oled_data[page],
      129,
      HAL_MAX_DELAY
    );
  }

  curr_render_step = FILL_FIRST_QUARTER_OF_BUFFER; // done rendering, ready to update the buffer
}

// OLED TEST

inline void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
  do_dac(&dac_dma_buffer[0]); // fill first half of buffer
  // copy_dac_buffer();
  switch (curr_render_step) {
    case FILL_FIRST_QUARTER_OF_BUFFER:
      for (int i = 0; i < DAC_DMA_BUFFER_SIZE; i++) {
        load_dac_buffer[i] = dac_dma_buffer[i];
      }
      break;
    case FILL_THIRD_QUARTER_OF_BUFFER:
      for (int i = 0; i < DAC_DMA_BUFFER_SIZE; i++) {
        load_dac_buffer[i+2*DAC_DMA_BUFFER_SIZE] = dac_dma_buffer[i];
      }
      break;
    default:
      return;
  }
  curr_render_step++; // start render
}
inline void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
  do_dac(&dac_dma_buffer[DAC_DMA_BUFFER_SIZE]); // fill second half of buffer
  // render_oled(); // call once buffer is full
  switch (curr_render_step) {
    case FILL_SECOND_QUARTER_OF_BUFFER:
      for (int i = DAC_DMA_BUFFER_SIZE; i < 2*DAC_DMA_BUFFER_SIZE; i++) {
        load_dac_buffer[i] = dac_dma_buffer[i];
      }
      break;
    case FILL_FOURTH_QUARTER_OF_BUFFER:
      for (int i = DAC_DMA_BUFFER_SIZE; i < 2*DAC_DMA_BUFFER_SIZE; i++) {
        load_dac_buffer[i+2*DAC_DMA_BUFFER_SIZE] = dac_dma_buffer[i];
      }
      break;
    default:
      return;
  }
  curr_render_step++; // start render
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
  process_adc_dma_buffer(&adc_dma_buffer[ADC_DMA_BUFFER_SIZE]);
}
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc){
  process_adc_dma_buffer(&adc_dma_buffer[0]);
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
  MX_ADC1_Init();
  MX_TIM8_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  printf("Starting main loop...\n");

  printf("filling sine lookup table...\n");
  synth_init(); // initialize the synth

  HAL_TIM_Base_Start(&htim8);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &adc_dma_buffer, 2 * ADC_DMA_BUFFER_SIZE);
  // HAL_TIM_Base_Start_IT(&htim6); // start timer 6 in interrupt mode
  // HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)dma_buffer, 2 * DAC_DMA_BUFFER_SIZE, DAC_ALIGN_12B_R); // start DAC in DMA mode


  OLED_Init(OLED1_ADDR);
  uint8_t data[129];

  // Using SH1106 which has 132x64 memory compared to 128x64 in SSD1306
  data[0] = 0x40; // 0x40 = following bytes are display data

  for (int i = 1; i < 129; i++)
      data[i] = 0x00;

  for (int page = 0; page < 8; page++)
  {
      OLED_Command(0xB0 + page, OLED1_ADDR);

      // Start from column address 2
      OLED_Command(0x00, OLED1_ADDR);
      OLED_Command(0x10, OLED1_ADDR);


      // Only page 0 gets the pixel
      if (page == 0){
          // data[40] = 0x11;
          // data[40] = 0b01010101;
          // data[40] = 0b00000001;
          data[40] = 0b10000000;
          // data[41] = 0b10101010;
      }
      else if (page == 1){
          data[40] = 0x00;
          data[41] = 0x00;
          // data[41] = 0x01;
      }
      else{
          data[40] = 0x00;
          data[41] = 0x00;
      }

      HAL_I2C_Master_Transmit(
          &hi2c1,
          OLED1_ADDR,
          data,
          129,
          HAL_MAX_DELAY
      );
  }

  // data[0] = 0x40; // 0x40 = following bytes are display data
  // OLED_Command(0xB0 + 1);
  // OLED_Command(0x02);
  // OLED_Command(0x10);

  // // data[40] = 0x00;
  // data[42] = 0x01;

  // HAL_I2C_Master_Transmit(
  //     &hi2c1,
  //     OLED_ADDR,
  //     data,
  //     129,
  //     HAL_MAX_DELAY
  // );


  uint32_t loop_counter = 0;
  uint32_t now = HAL_GetTick();
  // #define LOOP_COUNTER_LOG_INTERVAL 1000
  #define LOOP_COUNTER_LOG_INTERVAL 100
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

    if(is_WAVE_FORM_btn_pressed) {
      is_WAVE_FORM_btn_pressed = 0;
      BSP_LED_Toggle(LED2); // toggle led using BSP function
      iter_OSC1_waveform();
    }
    if(is_OSC2_btn_pressed) {
      is_OSC2_btn_pressed = 0;
      BSP_LED_Toggle(LED2); // toggle led using BSP function
      // toggle_OSC2();
      iter_chord();
    }

    if (is_blue_button_pressed) {
      is_blue_button_pressed = 0;
      BSP_LED_Toggle(LED2); // toggle led using BSP function
      // printf("Button pressed!\n");

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
          HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)dac_dma_buffer, 2 * DAC_DMA_BUFFER_SIZE, DAC_ALIGN_12B_R);
      }

      is_timer_running = !is_timer_running;
    }

    // log loop counter
    if (now >= next_loop_counter_log) {
      // printf("loop_counter: %lu\n", loop_counter);
      // printf("render_oled_count: %lu\n", render_oled_count);
      // loop_counter = 0;
      // render_oled_count = 0;
      next_loop_counter_log = now + LOOP_COUNTER_LOG_INTERVAL;

      // for (int page = 0; page < 8; page++){
      //   // for (int i = 0; i < (2*DAC_DMA_BUFFER_SIZE)*2; i++) {

      // oled_data[page][0] = 0x40; // 0x40 = following bytes are display data
      // // for (int i = 1; i < 129; i++)
      // //     oled_data[page][i] = 0x00;
      //   OLED_Command(0xB0 + page, OLED1_ADDR);
      //   OLED_Command(0x00, OLED1_ADDR);
      //   OLED_Command(0x10, OLED1_ADDR);
        
      //   // }
      //   HAL_I2C_Master_Transmit(
      //     &hi2c1,
      //     OLED1_ADDR,
      //     oled_data[page],
      //     129,
      //     HAL_MAX_DELAY
      //   );
      // }

      // if (is_oled_rendering) render_oled();
      // is_oled_rendering = false;
      if (curr_render_step == START_RENDER) render_oled();
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 8;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  for (uint8_t address = 1; address < 127; address++)
{
    if (HAL_I2C_IsDeviceReady(&hi2c1, address << 1, 3, 100) == HAL_OK)
    {
        printf("I2C device found at 0x%02X\n", address);
    }
}

  /* USER CODE END I2C1_Init 2 */

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
  htim6.Init.Period = 1905-1;
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
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 84-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 2000-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

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
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : WAVE_FORM_Pin OSC2_Pin */
  GPIO_InitStruct.Pin = WAVE_FORM_Pin|OSC2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

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
