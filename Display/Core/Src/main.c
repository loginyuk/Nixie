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
#include "i2c.h"
#include "i2s.h"
#include "spi.h"
#include "tim.h"
#include "usb_host.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd5110.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

uint16_t pins_lamps[] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3}; //LAMP HOUR 2-nd, LAMP HOUR 1-st, LAMP MINUTE 2-nd, LAMP MINUTE 1-st
uint16_t pins_controls[] = {GPIO_PIN_7, GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_10}; //CONTROL for K155id {A, B, C, D}
int controls[4][4];  //bin code for lamps and its control
bool state_ex = true;


//STATE FOR setting and working of lamp
typedef enum state_time {
 SETTING,
 WORKING,
} state_t;
volatile state_t state = WORKING;

//variable for setting clock
int MINUTE = 0;
int HOUR = 0;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//LCD5110_display lcd1; //Define LCD_DISPLAY

//convert decimal to bin to control lamp
void decimal_to_binary(int number, int *binary_value)
{
	int temp = number;
	for(int i = 0; i < 4; i++)
	{
		binary_value[i] = temp % 2;
		temp = temp / 2;
	}
}
//***********************DS3231-RTC MODULE***********************
#define DS3231_ADDRESS 0xD0 //define DS3231

// Convert normal decimal numbers to binary coded decimal
uint8_t decToBcd(int val)
{
  return (uint8_t)( (val/10*16) + (val%10) );
}


// Convert binary coded decimal to normal decimal numbers
int bcdToDec(uint8_t val)
{
  return (int)( (val/16*10) + (val%16) );
}

//structure for time
typedef struct {
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hour;
	uint8_t dayofweek;
	uint8_t dayofmonth;
	uint8_t month;
	uint8_t year;

}	TIME;
TIME time;


void Set_Time (uint8_t sec, uint8_t min, uint8_t hour, uint8_t dow, uint8_t dom, uint8_t month, uint8_t year)
{
	uint8_t set_time[7];
	set_time[0] = decToBcd(sec);
	set_time[1] = decToBcd(min);
	set_time[2] = decToBcd(hour);
	set_time[3] = decToBcd(dow);
	set_time[4] = decToBcd(dom);
	set_time[5] = decToBcd(month);
	set_time[6] = decToBcd(year);

	HAL_I2C_Mem_Write(&hi2c3, DS3231_ADDRESS, 0x00, 1, set_time, 7, 1000);
}

void Get_Time (void)
{
	uint8_t get_time[7];
	HAL_I2C_Mem_Read(&hi2c3, DS3231_ADDRESS, 0x00, 1, get_time, 7, 1000);
	time.seconds = bcdToDec(get_time[0]);
	time.minutes = bcdToDec(get_time[1]);
	time.hour = bcdToDec(get_time[2]);
	time.dayofweek = bcdToDec(get_time[3]);
	time.dayofmonth = bcdToDec(get_time[4]);
	time.month = bcdToDec(get_time[5]);
	time.year = bcdToDec(get_time[6]);
}
//*************************************************************************




//***************** Display TM_1638 ***********************//

GPIO_TypeDef* port = GPIOB;
int CLK_PIN = GPIO_PIN_14;
int DIO_PIN = GPIO_PIN_15;
int STB_PIN = GPIO_PIN_13;

const uint8_t SegmCodes[18] =
{
  0x3F, // 0
  0x06, // 1
  0x5B, // 2
  0x4F, // 3
  0x66, // 4
  0x6D, // 5
  0x7D, // 6
  0x07, // 7
  0x7F, // 8
  0x6F, // 9
  0x77, // A
  0x7c, // b
  0x39, // C
  0x5E, // d
  0x79, // E
  0x71,  // F
  //!!!! not hex
  0x38,// L - 10
  0x76 // H - 11
};

void shift_out(GPIO_TypeDef* port, int CLK_PIN,
    int DIO_PIN, bool dir, uint8_t command){
    for (int i = 0; i < 8; i++)
    {
        bool output = false;
        if (dir)
        {
            output = command & 0b10000000;
            command = command << 1;
        }
        else
        {
            output = command & 0b00000001;
            command = command >> 1;
        }
        HAL_GPIO_WritePin(port, DIO_PIN, output);
        HAL_GPIO_WritePin(port, CLK_PIN, 1);
        HAL_GPIO_WritePin(port, CLK_PIN, 0);
    }
  }


  void send_command(uint8_t bt){
    HAL_GPIO_WritePin(port, STB_PIN, 0);
    shift_out(port, CLK_PIN,  DIO_PIN, false, bt);
    HAL_GPIO_WritePin(port, STB_PIN, 1);
  }

  void send_args(uint8_t bt){
    shift_out(port, CLK_PIN,  DIO_PIN, false, bt);
  }

  void reset_TM(){
    send_command(0x40);
    HAL_GPIO_WritePin(port, STB_PIN, 0);
    send_args(0xc0);
    for (uint8_t i = 0; i < 16; i++){
         send_args(0x00);
       }
    HAL_GPIO_WritePin(port, STB_PIN, 1);
  }

  void print_zero(){
	  send_command(0x40);
	  HAL_GPIO_WritePin(port, STB_PIN, 0);
	  for(int i = 0; i < 8; i++)
	  {
		  send_args(SegmCodes[0]);
	  }
	  HAL_GPIO_WritePin(port, STB_PIN, 1);
  }

  void print_ones(){
	  send_command(0x40);
	  HAL_GPIO_WritePin(port, STB_PIN, 0);
	  for(int i = 0; i < 8; i++)
	  {
		  send_args(SegmCodes[1]);
	  }
	  HAL_GPIO_WritePin(port, STB_PIN, 1);
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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2S2_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  MX_I2C3_Init();
  MX_TIM10_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
//  lcd1.hw_conf.spi_handle = &hspi1;
//  lcd1.hw_conf.spi_cs_pin =  GPIO_PIN_2;
//  lcd1.hw_conf.spi_cs_port = GPIOA;
//  lcd1.hw_conf.rst_pin =  GPIO_PIN_0;
//  lcd1.hw_conf.rst_port = GPIOB;
//  lcd1.hw_conf.dc_pin =  GPIO_PIN_3;
//  lcd1.hw_conf.dc_port = GPIOA;
//  lcd1.def_scr = lcd5110_def_scr;
//  LCD5110_init(&lcd1.hw_conf, LCD5110_NORMAL_MODE, 0x40, 2, 3);

  //START DISPLAY TM1638
  HAL_TIM_Base_Start_IT(&htim2);
  send_command(0x8a);
  reset_TM();

//  Set_Time(10, 11, 17, 4, 21, 12, 23);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  char buffer[20];


	  if (state == WORKING)
	  {
	//	  sprintf(buffer, "TIME: %02d:%02d:%02d\n", time.hour, time.minutes, time.seconds);
	//	  LCD5110_print(buffer, BLACK, &lcd1);
		  Get_Time();
		  int time_units[] = {time.hour / 10, time.hour % 10, time.minutes / 10, time.minutes % 10}; //take time from rtc module
		  for (int i = 0; i < 4; i++)
		  {
			  decimal_to_binary(time_units[i], controls[i]);
			  for(int j = 0; j < 4; j++)
			  {
				  HAL_GPIO_WritePin(GPIOE, pins_controls[j], controls[i][j]);
			  }
			  HAL_GPIO_WritePin(GPIOD, pins_lamps[i], 1);
			  HAL_Delay(1);
			  HAL_GPIO_WritePin(GPIOD, pins_lamps[i], 0);
			  HAL_Delay(0.1);
		  }

	  }
	  else
	  {
		  int time_units[] = {HOUR / 10, HOUR % 10, MINUTE / 10, MINUTE % 10}; //take time from setting
//		  LCD5110_print("SETTING\n", BLACK, &lcd1);
//		  sprintf(buffer, "HOUR: %02d\nMINUTE: %02d\n", HOUR, MINUTE);
//		  LCD5110_print(buffer, BLACK, &lcd1);
		  for (int i = 0; i < 4; i++)
		  	  {
			  decimal_to_binary(time_units[i], controls[i]);
			  for(int j = 0; j < 4; j++)
			  	  {
				  HAL_GPIO_WritePin(GPIOE, pins_controls[j], controls[i][j]);
			  	  }
			  HAL_GPIO_WritePin(GPIOD, pins_lamps[i], 1);
			  HAL_Delay(1);
			  HAL_GPIO_WritePin(GPIOD, pins_lamps[i], 0);
			  HAL_Delay(0.1);
		  	  }
	  }
//	  LCD5110_refresh(&lcd1.hw_conf);
//	  LCD5110_clear_scr(&lcd1.hw_conf);
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 200;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 5;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//*****************SETTING*************//
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if((GPIO_Pin == GPIO_PIN_9) && state_ex == true){
		HAL_TIM_Base_Start_IT(&htim10);
		state_ex = false;
		MINUTE = time.minutes;
		HOUR = time.hour;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//PRINT 1 or 0 on TM1638
	if (htim == &htim2){
		if (state == WORKING)
		{
			print_ones();
		}
		else
		{
			print_zero();
		}
	}
	//SET MINUTE AND HOUR
	if (htim == &htim10)
	{
		if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_9) == GPIO_PIN_SET)
			{
			 if (state == WORKING){
			 MINUTE = time.minutes;
			 HOUR = time.hour;
			 state = SETTING;
			 }
			 else{
				 Set_Time(00, MINUTE, HOUR, time.dayofweek, time.dayofmonth, time.month, time.year);
				 state = WORKING;
				 state_ex = true;
				 HAL_TIM_Base_Stop_IT(&htim10);

			 }


			}
		else if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10) == GPIO_PIN_SET)
			 {
			 if (state == SETTING)
				 {
				 MINUTE++;
				 MINUTE = MINUTE % 60;
				 }
			 }
		else if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_11) == GPIO_PIN_SET)
			{
			 if (state == SETTING)
				 {
				 HOUR++;
				 HOUR = HOUR % 24;
				 }
			}
	}

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
