/* USER CODE BEGIN Header */
/**
  ******************************************************************************
    @file           : main.c
    @brief          : Main program body
  ******************************************************************************
    @attention

    Copyright (c) 2025 STMicroelectronics.
    All rights reserved.

    This software is licensed under terms that can be found in the LICENSE file
    in the root directory of this software component.
    If no LICENSE file comes with this software, it is provided AS-IS.

  ******************************************************************************
*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <letter.h>
#include <morse.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Struct for Letter and its Code
typedef struct {
  char ascii;                     // used to get index of this struct with its ascii value
  uint8_t *letter;                // pointer to letter bitmap at letter.h
  const uint32_t (*morse)[8];      // pointer to sequence of frames
  const char *morse_str;          // string to be transmitted to the monitor
} Pair;

Pair alphabet[27] = {
  {'A', letterA, morseA, ".- "},
  {'B', letterB, morseB, "-... "},
  {'C', letterC, morseC, "-.-. "},
  {'D', letterD, morseD, "-.. "},
  {'E', letterE, morseE, ". " },
  {'F', letterF, morseF, "..-. "},
  {'G', letterG, morseG, "--. "},
  {'H', letterH, morseH, ".... "},
  {'I', letterI, morseI, ".. "},
  {'J', letterJ, morseJ, ".--- "},
  {'K', letterK, morseK, "-.-"},
  {'L', letterL, morseL, ".-.. "},
  {'M', letterM, morseM, "-- "},
  {'N', letterN, morseN, "-. "},
  {'O', letterO, morseO, "--- "},
  {'P', letterP, morseP, ".--. "},
  {'Q', letterQ, morseQ, "--.- "},
  {'R', letterR, morseR, ".-. "},
  {'S', letterS, morseS, "... "},
  {'T', letterT, morseT, "- "},
  {'U', letterU, morseU, "..- "},
  {'V', letterV, morseV, "...- "},
  {'W', letterW, morseW, ".-- "},
  {'X', letterX, morseX, "-..- "},
  {'Y', letterY, morseY, "-.-- "},
  {'Z', letterZ, morseZ, "--.. "},
  {' ', space, blank_frame, "  "}
};

// Struct for LCD, it's will be highly useful when using more than 1 LCD
typedef struct {
  GPIO_TypeDef* RS_Port;     // RS = register select
  uint16_t RS_Pin;           // RS = 0 (command), RS = 1 (data)
  // don't use RW because "write only" mode, lcd doesn't read, it always 0 / GND
  // GPIO_TypeDef* RW_Port;  // RW = Read / Write
  // uint16_t RW_Pin;        // RW = 0 (write). RW = 1 (read)
  GPIO_TypeDef* EN_Port;     // E = enable
  uint16_t EN_Pin;           // for clock edge to update data
  GPIO_TypeDef* D4_Port;     // D4 - D7 = 4 bit data I/O
  uint16_t D4_Pin;
  GPIO_TypeDef* D5_Port;
  uint16_t D5_Pin;
  GPIO_TypeDef* D6_Port;
  uint16_t D6_Pin;
  GPIO_TypeDef* D7_Port;
  uint16_t D7_Pin;
} LCD_HandleTypeDef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//For 7 Segments
#define PB GPIOB               // We only use GPIOB for the 7 Segments
#define SEG_a GPIO_PIN_1
#define SEG_b GPIO_PIN_2
#define SEG_c GPIO_PIN_3
#define SEG_d GPIO_PIN_4
#define SEG_e GPIO_PIN_5
#define SEG_f GPIO_PIN_6
#define SEG_g GPIO_PIN_7

// For LED Dot Matrix
// 1
#define CS_PORT1   GPIOD       // CS = chip select
#define CS_PIN1    GPIO_PIN_1
#define CLK_PORT1  GPIOD       // clock
#define CLK_PIN1   GPIO_PIN_0
#define DIN_PORT1  GPIOD       // DIN = Data In 
#define DIN_PIN1   GPIO_PIN_2

// CS1 low/high
#define CS1_LOW()  HAL_GPIO_WritePin(CS_PORT1, CS_PIN1, GPIO_PIN_RESET)
#define CS1_HIGH() HAL_GPIO_WritePin(CS_PORT1, CS_PIN1, GPIO_PIN_SET)

#define MATRIX1_CASCADE 4   // chain of matrix 1

// 2
#define CS_PORT2   GPIOB
#define CS_PIN2    GPIO_PIN_9
#define CLK_PORT2  GPIOB
#define CLK_PIN2   GPIO_PIN_8
#define DIN_PORT2  GPIOB
#define DIN_PIN2   GPIO_PIN_10

// CS2 low/high
#define CS2_LOW()  HAL_GPIO_WritePin(CS_PORT2, CS_PIN2, GPIO_PIN_RESET)
#define CS2_HIGH() HAL_GPIO_WritePin(CS_PORT2, CS_PIN2, GPIO_PIN_SET)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// For LCD
LCD_HandleTypeDef lcd1 = {
  .RS_Port = GPIOA, .RS_Pin = GPIO_PIN_4,
  .EN_Port = GPIOA, .EN_Pin = GPIO_PIN_6,
  .D4_Port = GPIOA, .D4_Pin = GPIO_PIN_7,
  .D5_Port = GPIOA, .D5_Pin = GPIO_PIN_8,
  .D6_Port = GPIOA, .D6_Pin = GPIO_PIN_9,
  .D7_Port = GPIOA, .D7_Pin = GPIO_PIN_10
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

// For 7 Segments
void display(int num);
void clear_segments();

// For LED Dot Matrix
void MAX7219_Send1(uint8_t reg, uint8_t data);
void MAX7219_SendRaw(GPIO_TypeDef *DIN_PORT, uint16_t DIN_PIN,
                     GPIO_TypeDef *CLK_PORT, uint16_t CLK_PIN,
                     uint8_t reg, uint8_t data);
void MAX7219_Init1(void);
void MAX7219_Display1(const uint32_t frame[8]);
void MAX7219_Clear1(void);

void MAX7219_Send2(uint8_t reg, uint8_t data);
void MAX7219_Init2(void);
void MAX7219_Display2(uint8_t rows[8]);
void MAX7219_Clear2(void);

// UART Input
void Play (Pair data, int idx);
Pair GetMorse (char input);
void runSequence (char *input);

// LCD
void LCD_EnablePulse(LCD_HandleTypeDef *lcd);
void LCD_Send4Bits(LCD_HandleTypeDef *lcd, uint8_t data);
void LCD_Send(LCD_HandleTypeDef *lcd, uint8_t data, uint8_t rs);
void LCD_Init(LCD_HandleTypeDef *lcd);
void LCD_Clear(LCD_HandleTypeDef *lcd);
void LCD_SetCursor(LCD_HandleTypeDef *lcd, uint8_t row, uint8_t col);
void LCD_Print(LCD_HandleTypeDef *lcd, char *str);
void LCD_update(Pair data);

// ADC -> LED Dot Matrix
volatile uint16_t duration;
long mapp(long x, long in_min, long in_max, long out_min, long out_max);
void MAX7219_SetIntensity(uint8_t intensity);

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void ch0()
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}
void ch1()
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}
// Function to reverse bitmap
uint8_t reverseByte(uint8_t b) {
  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
  b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
  b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
  return b;
}
void mirror(uint8_t bitmap[8]) {
  for (int i = 0; i < 8; i++) {
    bitmap[i] = reverseByte(bitmap[i]);
  }
}
// void flip(uint8_t bitmap[8]) {
//   for (int i = 0; i < 4; i++) {
//     uint8_t temp = bitmap[i];
//     bitmap[i] = bitmap[7 - i];
//     bitmap[7 - i] = temp;
//   }
// }


// void mirrorAll(Pair *table, int count) {
//     for (int i = 0; i < count; i++) {
//         mirror(table[i].letter);   // mirror bitmap huruf
//     }
// }
// void flipAll(Pair *table, int count) {
//     for (int i = 0; i < count; i++) {
//         flip(table[i].letter);   // mirror bitmap huruf
//     }
// }


// Function dot & dash
void dot(int time) {
  for (int i = 0; i < time; i++) {
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_3);
    HAL_Delay(1.1);
  }
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET); // pastikan OFF
}
void dash(int time) {
  for (int i = 0; i < (3 * time); i++) {
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_3);
    HAL_Delay(1.1);
  }
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET); // pastikan OFF
}

/* USER CODE END 0 */

/**
    @brief  The application entry point.
    @retval int
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
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  LCD_Init(&lcd1);

  /* USER CODE BEGIN 2 */
  uint8_t RXdata;   // store 1 char from UART
  char buffer1[100]; // buffer to store UART input (max 100 char)
  int index = 0;    // index for buffer
  char header[] = "Type something here, then press ENTER \n";

  HAL_UART_Transmit (&huart2, (uint8_t*)header, strlen(header), 100);

  // function to test the alignment of the LED Dot Matrix
  // test_order();

  // Mirrored letters
  mirror(letterA);
  mirror(letterB);
  mirror(letterC);
  mirror(letterD);
  mirror(letterF);
  mirror(letterE);
  mirror(letterG);
  mirror(letterH);
  mirror(letterI);
  mirror(letterJ);
  mirror(letterK);
  mirror(letterL);
  mirror(letterM);
  mirror(letterN);
  mirror(letterO);
  mirror(letterP);
  mirror(letterQ);
  mirror(letterR);
  mirror(letterS);
  mirror(letterT);
  mirror(letterU);
  mirror(letterV);
  mirror(letterW);
  mirror(letterX);
  mirror(letterY);
  mirror(letterZ);
  // mirrorAll(alphabet,26);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // ADC
    ch0(); //switch to channel 0
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 1);
    uint16_t adcResult0 = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    // set brightness of LED Dot Matrix
    uint8_t brightness = mapp(adcResult0, 0, 4095, 0, 15);
    MAX7219_SetIntensity(brightness);

    // define brightness as number of 7 Segments
    int digit = mapp(brightness, 0, 15, 0, 9);
    display(digit);

    ch1(); //switch to channel 1
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 1); //100ms adalah nilai timeout atau batas waktu
    uint16_t adcResult1 = HAL_ADC_GetValue (&hadc1);
    HAL_ADC_Stop (&hadc1);

    duration = mapp(adcResult1, 0, 4095, 50, 150);

    // UART
    if (HAL_UART_Receive(&huart2, &RXdata, 1, 0) == HAL_OK)
    {
        // A byte was received
        if (RXdata == '\n')
        {
            buffer1[index] = '\0';
            runSequence(buffer1);
            index = 0;
        }
        else 
        {
            buffer1[index++] = RXdata;
        }
    }

  }
  /* USER CODE END 3 */
}

/**
    @brief System Clock Configuration
    @retval None
*/
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_1);

  /** Initializes the RCC Oscillators according to the specified parameters
    in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
    @brief ADC1 Initialization Function
    @param None
    @retval None
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  // sConfig.Channel = ADC_CHANNEL_0;
  // sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  // if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  // {
  //   Error_Handler();
  // }

  // /** Configure Regular Channel
  // */
  // sConfig.Channel = ADC_CHANNEL_1;
  // if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  // {
  //   Error_Handler();
  // }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
    @brief USART2 Initialization Function
    @param None
    @retval None
*/
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
    @brief GPIO Initialization Function
    @param None
    @retval None
*/
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7
                    | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_10 | GPIO_PIN_3
                    | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7
                    | GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;   // alternate function allowing it to connect to internal peripherals
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7
                           PA8 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7
                        | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 PB10 PB3
                           PB4 PB5 PB6 PB7
                           PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_10 | GPIO_PIN_3
                        | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7
                        | GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 PD2 PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// ADC -> LED Dot Matrix
long mapp(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void MAX7219_SetIntensity(uint8_t intensity) {
  if (intensity > 15) intensity = 15;
  MAX7219_Send1(0x0A, intensity);
  MAX7219_Send2(0x0A, intensity);
}

// For 7 Segments
void display(int num)
{
  clear_segments();

  switch (num)
  {
    case 0:
      HAL_GPIO_WritePin(PB, SEG_a | SEG_b | SEG_c | SEG_d | SEG_e | SEG_f, 0);
      break;

    case 1:
      HAL_GPIO_WritePin(PB, SEG_b | SEG_c, 0);
      break;

    case 2:
      HAL_GPIO_WritePin(PB, SEG_a | SEG_b | SEG_d | SEG_e | SEG_g, 0);
      break;

    case 3:
      HAL_GPIO_WritePin(PB, SEG_a | SEG_b | SEG_c | SEG_d | SEG_g, 0);;
      break;

    case 4:
      HAL_GPIO_WritePin(PB, SEG_b | SEG_c | SEG_f | SEG_g, 0);
      break;

    case 5:
      HAL_GPIO_WritePin(PB, SEG_a | SEG_c | SEG_d | SEG_f | SEG_g, 0);
      break;

    case 6:
      HAL_GPIO_WritePin(PB, SEG_a | SEG_c | SEG_d | SEG_e | SEG_f | SEG_g, 0);
      break;

    case 7:
      HAL_GPIO_WritePin(PB, SEG_a | SEG_b | SEG_c, 0);
      break;

    case 8:
      HAL_GPIO_WritePin(PB, SEG_a | SEG_b | SEG_c | SEG_d | SEG_e | SEG_f | SEG_g, 0);
      break;

    case 9:
      HAL_GPIO_WritePin(PB, SEG_a | SEG_b | SEG_c | SEG_d | SEG_f | SEG_g, 0);
      break;

    default:
      clear_segments();
      break;
  }
}
void clear_segments()
{
  HAL_GPIO_WritePin(PB, SEG_a | SEG_b | SEG_c | SEG_d | SEG_e | SEG_f | SEG_g, 1);
  // Default HIGH karena display common-anode (CA disambung ke VCC, CC disambung ke GND)
}

// For LED Dot Matrix
//1
void MAX7219_Send1(uint8_t reg, uint8_t data)
{
  CS1_LOW();

  // Send to all cascaded panels (LEFTMOST FIRST)
  for (int chip = MATRIX1_CASCADE - 1; chip >= 0; chip--)
  {
    for (int i = 0; i < 8; i++)
    {
      HAL_GPIO_WritePin(CLK_PORT1, CLK_PIN1, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DIN_PORT1, DIN_PIN1,
                        (reg & (1 << (7 - i))) ? GPIO_PIN_SET : GPIO_PIN_RESET);
      HAL_GPIO_WritePin(CLK_PORT1, CLK_PIN1, GPIO_PIN_SET);
    }

    for (int i = 0; i < 8; i++)
    {
      HAL_GPIO_WritePin(CLK_PORT1, CLK_PIN1, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DIN_PORT1, DIN_PIN1,
                        (data & (1 << (7 - i))) ? GPIO_PIN_SET : GPIO_PIN_RESET);
      HAL_GPIO_WritePin(CLK_PORT1, CLK_PIN1, GPIO_PIN_SET);
    }
  }

  CS1_HIGH();
}
void MAX7219_SendRaw(GPIO_TypeDef *DIN_PORT, uint16_t DIN_PIN,
                     GPIO_TypeDef *CLK_PORT, uint16_t CLK_PIN,
                     uint8_t reg, uint8_t data)
{
  // Send register byte (8 bits)
  for (int i = 0; i < 8; i++)
  {
    HAL_GPIO_WritePin(CLK_PORT, CLK_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DIN_PORT, DIN_PIN,
                      (reg & (1 << (7 - i))) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CLK_PORT, CLK_PIN, GPIO_PIN_SET);
  }

  // Send data byte (8 bits)
  for (int i = 0; i < 8; i++)
  {
    HAL_GPIO_WritePin(CLK_PORT, CLK_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DIN_PORT, DIN_PIN,
                      (data & (1 << (7 - i))) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CLK_PORT, CLK_PIN, GPIO_PIN_SET);
  }
}
void MAX7219_Init1(void)
{
  MAX7219_Send1(0x09, 0x00); // Decode mode: none
  MAX7219_Send1(0x0A, 0x0F); // Intensity: max 0–15
  MAX7219_Send1(0x0B, 0x07); // Scan limit: 0–7 (all 8 digits)
  MAX7219_Send1(0x0C, 0x01); // Shutdown register: normal operation
  MAX7219_Send1(0x0F, 0x00); // Display test: off
}
void MAX7219_Display1(const uint32_t frame[8])
{
  for (uint8_t row = 0; row < 8; row++)
  {
    CS1_LOW();

    // SEND LEFTMOST PANEL FIRST
    for (int chip = MATRIX1_CASCADE - 1; chip >= 0; chip--)
    {
      // ALSO REVERSE THE BYTE EXTRACTION
      uint8_t colbyte = (frame[row] >> (8 * (MATRIX1_CASCADE - 1 - chip))) & 0xFF;

      MAX7219_SendRaw(
        DIN_PORT1, DIN_PIN1,
        CLK_PORT1, CLK_PIN1,
        row + 1,
        colbyte
      );
    }

    CS1_HIGH();
  }
}
void MAX7219_Clear1(void)
{
  for (uint8_t row = 0; row < 8; row++)
  {
    CS1_LOW();
    for (int chip = 0; chip < MATRIX1_CASCADE; chip++)
    {
      MAX7219_SendRaw(DIN_PORT1, DIN_PIN1,
                      CLK_PORT1, CLK_PIN1,
                      row + 1, 0x00);
    }
    CS1_HIGH();
  }
}

//2
void MAX7219_Send2(uint8_t reg, uint8_t data)
{
  CS2_LOW();
  for (int i = 0; i < 8; i++) {   // Kirim register byte
    HAL_GPIO_WritePin(CLK_PORT2, CLK_PIN2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DIN_PORT2, DIN_PIN2, (reg & (1 << (7 - i))) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CLK_PORT2, CLK_PIN2, GPIO_PIN_SET);
  }

  for (int i = 0; i < 8; i++) {   // Kirim data byte
    HAL_GPIO_WritePin(CLK_PORT2, CLK_PIN2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DIN_PORT2, DIN_PIN2, (data & (1 << (7 - i))) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CLK_PORT2, CLK_PIN2, GPIO_PIN_SET);
  }
  CS2_HIGH();
}
void MAX7219_Init2(void)
{
  MAX7219_Send2(0x09, 0x00); // Decode mode: none
  MAX7219_Send2(0x0A, 0x0F); // Intensity: max 0–15
  MAX7219_Send2(0x0B, 0x07); // Scan limit: 0–7 (all 8 digits)
  MAX7219_Send2(0x0C, 0x01); // Shutdown register: normal operation
  MAX7219_Send2(0x0F, 0x00); // Display test: off

}
void MAX7219_Display2(uint8_t letter[8])
{
  for (uint8_t row = 0; row < 8; row++)
    MAX7219_Send2(row + 1, letter[row]);
}
void MAX7219_Clear2(void)
{
  for (uint8_t i = 0; i < 8; i++)
    MAX7219_Send2(i + 1, 0x00);
}

// UART I/O
// Execution
void play(Pair data, int idx)
{
  uint32_t (*temp1)[8];
  temp1 = data.morse;
  uint16_t time = duration;
  MAX7219_Display1(blank_frame);
  MAX7219_Display2(data.letter);
  LCD_update(data);
  HAL_Delay (200);

  if ( idx != 26 ) {
    uint8_t temp2 = data.ascii;
    HAL_UART_Transmit(&huart2, &temp2, 1, 10);
    HAL_UART_Transmit (&huart2, (uint8_t*)" : ", 3, 10);
    HAL_UART_Transmit (&huart2, (uint8_t*)data.morse_str, strlen(data.morse_str), 10);
    HAL_UART_Transmit (&huart2, (uint8_t*)"\n", 1, 10);
  }

  switch (idx)
  {
    // A .-
    case 0 : {
        MAX7219_Display1(blank_frame); HAL_Delay(time);
        dot(time);
        MAX7219_Display1(temp1[0]); HAL_Delay(time);
        dash(time);
        MAX7219_Display1(temp1[1]); HAL_Delay(3*time);
      }
      break;

    // B -...
    case 1 : {
        MAX7219_Display1(blank_frame); HAL_Delay(time);
        dash(time);
        MAX7219_Display1(temp1[0]);  HAL_Delay(time);
        dot(time);
        MAX7219_Display1(temp1[1]); HAL_Delay(time);
        dot(time);
        MAX7219_Display1(temp1[2]); HAL_Delay(time);
        dot(time);
        MAX7219_Display1(temp1[3]); HAL_Delay(3*time);
      } break;

    // C -.-.
    case 2 : {
        MAX7219_Display1(blank_frame); HAL_Delay(time);
        dot(time);
        MAX7219_Display1(temp1[0]);  HAL_Delay(time);
        dash(time);
        MAX7219_Display1(temp1[1]); HAL_Delay(time);
        dot(time);
        MAX7219_Display1(temp1[2]); HAL_Delay(3*time);
      } break;

    // D -..
    case 3 : {
        MAX7219_Display1(blank_frame); HAL_Delay(time);
        dash(time);
        MAX7219_Display1(temp1[0]);  HAL_Delay(time);
        dot(time);
        MAX7219_Display1(temp1[1]); HAL_Delay(time);
        dot(time);
        MAX7219_Display1(temp1[2]); HAL_Delay(3*time);
      } break;

    // E .
    case 4 : {
        MAX7219_Display1(blank_frame); HAL_Delay(time);
        dot(time);
        MAX7219_Display1(temp1[0]); HAL_Delay(3*time);
      } break;

    // F ..-.
    case 5 :  {
        MAX7219_Display1(blank_frame); HAL_Delay(time);
        dot(time);
        MAX7219_Display1(temp1[0]);  HAL_Delay(time);
        dot(time); \
        MAX7219_Display1(temp1[1]); HAL_Delay(time);
        dash(time);
        MAX7219_Display1(temp1[2]); HAL_Delay(time);
        dot(time);
        MAX7219_Display1(temp1[3]); HAL_Delay(3*time);
      } break;

    // G --.
    case 6 :  {
        MAX7219_Display1(blank_frame); HAL_Delay(time);
        dash(time);
        MAX7219_Display1(temp1[0]);  HAL_Delay(time);
        dash(time);
        MAX7219_Display1(temp1[1]); HAL_Delay(time);
        dot(time);
        MAX7219_Display1(temp1[2]); HAL_Delay(3*time);
      } break;

    // H ....
    case 7 :  {
        MAX7219_Display1(blank_frame); HAL_Delay(time);
        dot(time);
        MAX7219_Display1(temp1[0]);  HAL_Delay(time);
        dot(time);
        MAX7219_Display1(temp1[1]); HAL_Delay(time);
        dot(time); \
        MAX7219_Display1(temp1[2]); HAL_Delay(time);
        dot(time);
        MAX7219_Display1(temp1[3]); HAL_Delay(3*time);
      } break;

    // I ..
    case 8 :  {
        MAX7219_Display1(blank_frame); HAL_Delay(time);
        dot(time);
        MAX7219_Display1(temp1[0]);  HAL_Delay(time);
        dot(time);
        MAX7219_Display1(temp1[1]); HAL_Delay(3*time);
      } break;

    // J .---
    case 9 :  {
        MAX7219_Display1(blank_frame); HAL_Delay(time);
        dot(time);
        MAX7219_Display1(temp1[0]);  HAL_Delay(time);
        dash(time);
        MAX7219_Display1(temp1[1]); HAL_Delay(time);
        dash(time);
        MAX7219_Display1(temp1[2]); HAL_Delay(time);
        dash(time);
        MAX7219_Display1(temp1[3]); HAL_Delay(3*time);
      } break;

    // K -.-
    case 10 :  {
        MAX7219_Display1(blank_frame); HAL_Delay(time);
        dash(time);
        MAX7219_Display1(temp1[0]);  HAL_Delay(time);
        dot(time);
        MAX7219_Display1(temp1[1]); HAL_Delay(time);
        dash(time);
        MAX7219_Display1(temp1[2]); HAL_Delay(3*time);
      } break;

    // L .-..
    case 11 :  {
        MAX7219_Display1(blank_frame); HAL_Delay(time);
        dot(time);
        MAX7219_Display1(temp1[0]);  HAL_Delay(time);
        dash(time);
        MAX7219_Display1(temp1[1]); HAL_Delay(time);
        dot(time);
        MAX7219_Display1(temp1[2]); HAL_Delay(time);
        dot(time);
        MAX7219_Display1(temp1[3]); HAL_Delay(3*time);
      } break;

    // M --
    case 12 :  {
        MAX7219_Display1(blank_frame); HAL_Delay(time);
        dash(time);
        MAX7219_Display1(temp1[0]);  HAL_Delay(time);
        dash(time);
        MAX7219_Display1(temp1[1]); HAL_Delay(3*time);
      } break;

    // N -.
    case 13 :  {
        dash(time);
        MAX7219_Display1(temp1[0]);  HAL_Delay(time);
        dot(time);
        MAX7219_Display1(temp1[1]); HAL_Delay(3*time);
      } break;

    // O ---
    case 14 :  {
        MAX7219_Display1(blank_frame); HAL_Delay(time);
        dash(time);
        MAX7219_Display1(temp1[0]);  HAL_Delay(time);
        dash(time);
        MAX7219_Display1(temp1[1]); HAL_Delay(time);
        dash(time);
        MAX7219_Display1(temp1[2]); HAL_Delay(3*time);
      } break;

    // P .--.
    case 15 : {
        MAX7219_Display1(blank_frame); HAL_Delay(time);
        dot(time);
        MAX7219_Display1(temp1[0]);  HAL_Delay(time);
        dash(time);
        MAX7219_Display1(temp1[1]); HAL_Delay(time);
        dash(time);
        MAX7219_Display1(temp1[2]); HAL_Delay(time);
        dot(time);
        MAX7219_Display1(temp1[3]); HAL_Delay(3*time);
      } break;

    // Q --.-
    case 16 :  {
        MAX7219_Display1(blank_frame); HAL_Delay(time);
        dash(time);
        MAX7219_Display1(temp1[0]);  HAL_Delay(time);
        dash(time);
        MAX7219_Display1(temp1[1]); HAL_Delay(time);
        dot(time);
        MAX7219_Display1(temp1[2]); HAL_Delay(time);
        dash(time);
        MAX7219_Display1(temp1[3]); HAL_Delay(3*time);
      } break;

    // R .-.
    case 17 :  {
        MAX7219_Display1(blank_frame); HAL_Delay(time);
        dot(time);
        MAX7219_Display1(temp1[0]);  HAL_Delay(time);
        dash(time);
        MAX7219_Display1(temp1[1]); HAL_Delay(time);
        dot(time);
        MAX7219_Display1(temp1[2]); HAL_Delay(3*time);
      } break;

    // S ...
    case 18 :  {
        MAX7219_Display1(blank_frame); HAL_Delay(time);
        dot(time);
        MAX7219_Display1(temp1[0]);  HAL_Delay(time);
        dot(time);
        MAX7219_Display1(temp1[1]); HAL_Delay(time);
        dot(time);
        MAX7219_Display1(temp1[2]); HAL_Delay(3*time);
      } break;

    // T -
    case 19 :  {
        MAX7219_Display1(blank_frame); HAL_Delay(time);
        dash(time);
        MAX7219_Display1(temp1[0]);  HAL_Delay(3*time);
      } break;

    // U ..-
    case 20 :  {
        MAX7219_Display1(blank_frame); HAL_Delay(time);
        dot(time);
        MAX7219_Display1(temp1[0]);  HAL_Delay(time);
        dot(time);
        MAX7219_Display1(temp1[1]); HAL_Delay(time);
        dash(time);
        MAX7219_Display1(temp1[2]); HAL_Delay(3*time);
      } break;

    // V ...-
    case 21 :  {
        MAX7219_Display1(blank_frame); HAL_Delay(time);
        dot(time);
        MAX7219_Display1(temp1[0]);  HAL_Delay(time);
        dot(time);
        MAX7219_Display1(temp1[1]); HAL_Delay(time);
        dot(time);
        MAX7219_Display1(temp1[2]); HAL_Delay(time);
        dash(time);
        MAX7219_Display1(temp1[3]); HAL_Delay(3*time);
      } break;

    // W .--
    case 22 :  {
        MAX7219_Display1(blank_frame); HAL_Delay(time);
        dot(time);
        MAX7219_Display1(temp1[0]);  HAL_Delay(time);
        dash(time);
        MAX7219_Display1(temp1[1]); HAL_Delay(time);
        dash(time);
        MAX7219_Display1(temp1[2]); HAL_Delay(3*time);
      } break;

    // X -..-
    case 23 :  {
        MAX7219_Display1(blank_frame); HAL_Delay(time);
        dash(time);
        MAX7219_Display1(temp1[0]);  HAL_Delay(time);
        dot(time);
        MAX7219_Display1(temp1[1]); HAL_Delay(time);
        dot(time);
        MAX7219_Display1(temp1[2]); HAL_Delay(time);
        dash(time);
        MAX7219_Display1(temp1[3]); HAL_Delay(3*time);
      } break;

    // Y -.--
    case 24 :  {
        MAX7219_Display1(blank_frame); HAL_Delay(time);
        dash(time);
        MAX7219_Display1(temp1[0]);  HAL_Delay(time);
        dot(time);
        MAX7219_Display1(temp1[1]); HAL_Delay(time);
        dash(time);
        MAX7219_Display1(temp1[2]); HAL_Delay(time);
        dash(time);
        MAX7219_Display1(temp1[3]); HAL_Delay(3*time);
      } break;

    // Z --..
    case 25 :  {
        MAX7219_Display1(blank_frame); HAL_Delay(time);
        dash(time);
        MAX7219_Display1(temp1[0]);  HAL_Delay(time);
        dash(time);
        MAX7219_Display1(temp1[1]); HAL_Delay(time);
        dot(time);
        MAX7219_Display1(temp1[2]); HAL_Delay(time);
        dot(time);
        MAX7219_Display1(temp1[3]); HAL_Delay(3*time);
      } break;

    // Space
    case 26 :  {
        HAL_UART_Transmit(&huart2, (uint8_t*)" space \n", 8, 100);
        MAX7219_Clear1();
        HAL_Delay(7 * time);
      } break;

    default: MAX7219_Clear1();
  }
}
// Get index
int GetMorseIndex(char c)
{
  if (c == ' ') return 26;
  if (c >= 'A' && c <= 'Z') return c - 'A';
  if (c >= 'a' && c <= 'z') return c - 'a';
  return -1;
}
// Sequential Char Running (read the input from serial monitor)
void runSequence(char *input)
{
  int i = 0;
  while (input[i] != '\0')
  {
    int idx = GetMorseIndex(input[i]);
    if (idx >= 0)
    {
      play(alphabet[idx], idx);
    }
    i++;
  }
  MAX7219_Clear1();
  MAX7219_Clear2();
  LCD_Clear(&lcd1);
}

// For LCD
void LCD_EnablePulse(LCD_HandleTypeDef *lcd) {
  HAL_GPIO_WritePin(lcd->EN_Port, lcd->EN_Pin, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(lcd->EN_Port, lcd->EN_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);
}
void LCD_Send4Bits(LCD_HandleTypeDef *lcd, uint8_t data) {
  HAL_GPIO_WritePin(lcd->D4_Port, lcd->D4_Pin, (data & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(lcd->D5_Port, lcd->D5_Pin, (data & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(lcd->D6_Port, lcd->D6_Pin, (data & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(lcd->D7_Port, lcd->D7_Pin, (data & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  LCD_EnablePulse(lcd);
}
void LCD_Send(LCD_HandleTypeDef *lcd, uint8_t data, uint8_t rs) {
  HAL_GPIO_WritePin(lcd->RS_Port, lcd->RS_Pin, rs ? GPIO_PIN_SET : GPIO_PIN_RESET);
  LCD_Send4Bits(lcd, (data >> 4) & 0x0F); // high nibble
  LCD_Send4Bits(lcd, data & 0x0F);        // low nibble
}
void LCD_Init(LCD_HandleTypeDef *lcd) {
  HAL_Delay(50);
  LCD_Send(lcd, 0x33, 0);
  LCD_Send(lcd, 0x32, 0);
  LCD_Send(lcd, 0x28, 0); // 4-bit, 2 line
  LCD_Send(lcd, 0x0C, 0); // Display ON, Cursor OFF
  LCD_Send(lcd, 0x06, 0); // Entry mode
  LCD_Send(lcd, 0x01, 0); // Clear
  HAL_Delay(2);
}
void LCD_Clear(LCD_HandleTypeDef *lcd) {
  LCD_Send(lcd, 0x01, 0);
  HAL_Delay(2);
}
void LCD_SetCursor(LCD_HandleTypeDef *lcd, uint8_t row, uint8_t col) {
  uint8_t addr = (row == 0) ? 0x00 : 0x40;
  addr += col;
  LCD_Send(lcd, 0x80 | addr, 0);
}
void LCD_Print(LCD_HandleTypeDef *lcd, char *str) {
  while (*str) {
    LCD_Send(lcd, *str++, 1);
  }
}
void LCD_update(Pair data)
{
  LCD_Clear(&lcd1);

  char buf[32];
  LCD_SetCursor(&lcd1, 0, 0);
  sprintf(buf, "Char  : %c", data.ascii);
  LCD_Print(&lcd1, buf);

  LCD_SetCursor(&lcd1, 1, 0);
  sprintf(buf, "Morse : %s", data.morse_str);
  LCD_Print(&lcd1, buf);

  HAL_Delay(500);
}

/* USER CODE END 4 */

/**
    @brief  This function is executed in case of error occurrence.
    @retval None
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
    @brief  Reports the name of the source file and the source line number
            where the assert_param error has occurred.
    @param  file: pointer to the source file name
    @param  line: assert_param error line source number
    @retval None
*/
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
