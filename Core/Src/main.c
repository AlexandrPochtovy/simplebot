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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "I2C/MyI2C.h"
#include "Function/Function.h"
#include "Move.h"
#include "PID/MyPid.h"
#include "VL53L0x/VL53L0x.h"
#include "MCP23017/MCP23_17.h"
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
//TIME
volatile uint32_t mainTick = 0;
volatile uint32_t pool = 100;
uint32_t cycleTime;
uint32_t previousTime = 0;
uint32_t dt = 0;

volatile uint32_t PeripheralRequest = 0;


//EMERGENCY
volatile uint8_t stopEmrgFront = 0;  //emergency stop flag from switch
uint16_t wall = 0;

//I2C
uint8_t i2c1_buffer[32] = {0};  //data buffer for i2c bus
fifo_t ringI2C1 = { i2c1_buffer, 32, 0, 0, 0 };  //structure for circular i2c buffer
I2C_Connection_t I2C1_Bus = { I2C1, PORT_FREE, 0, 0, I2C_MODE_WRITE, 0, &ringI2C1 };

uint8_t i2c2_buffer[32] = {0};  //data buffer for i2c bus
fifo_t ringI2C2 = { i2c2_buffer, 32, 0, 0, 0 };  //structure for circular i2c buffer
I2C_Connection_t I2C2_Bus = { I2C2, PORT_FREE, 0, 0, I2C_MODE_WRITE, 0, &ringI2C2 };

//OBJECTS
VL53L0X LidarLeft;	//left laser sensor
VL53L0X LidarRight;	//right laser sensor

robot_t bot = {//robot main object
             	{0.0, 0.0},			//координаты робота
             	0.0,						//main speed
             	0.0,						//расстояние до цели
             	0.0,					//путь пройденный центром робота
             	0.0,		//интеграл пути
             	0.0,  		//пеленг (theta) и�?комый
             	0.0,  			//psi кур�? робота, угол в �?и�?теме координат
             	0.0,  			//курcовой угол
             	0.0,		//угловая скорость
              {0, 0, 0, 0.0, 0.0},		//wheel left
              {0, 0, 0, 0.0, 0.0},		//wheel right
              {TIM4, LL_TIM_CHANNEL_CH1, LL_TIM_CHANNEL_CH2, STOP },	//drive
              {	{0.0, 0.0},//координаты робота}
								0,
								0
								}
		};

point_t target = {1.0, 1.0};

//PID
pidData_t pidLeft;
pidData_t pidRight;

//distance
uint16_t lms_int;
uint16_t rms_int;
uint16_t distL_mm;  //ра�?�?то�?ние �?лева по�?ле фильтрации, мм
uint16_t distR_mm;  //ра�?�?то�?ние �?права по�?ле фильтрации, мм
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
	uint8_t laserStep = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /** NOJTAG: JTAG-DP Disabled and SW-DP Enabled
  */
  LL_GPIO_AF_Remap_SWJ_NOJTAG();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  //MY_I2C1_Init();
  //MY_I2C2_Init();
  /* USER CODE BEGIN 2 */
	SysTick_Config(SystemCoreClock / 1000);  //1ms tick
	LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);
	setupVL53L0X(&LidarLeft, 20);
	setupVL53L0X(&LidarRight, 20);
	previousTime = mainTick;

	pid_Init(14, 6, 2, &pidLeft);
	pid_Init(14, 6, 2, &pidRight);
	//DWT_Init();
	uint8_t stl = 0;
	uint8_t str = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  	//DWT_Meas(&cycleTime);
  	if (PeripheralRequest & LIDAR_REQUEST_MASK) {
			switch (laserStep) {
			case 0:
				while (str == 0) {
					str = VL_Init(&I2C1_Bus, &LidarRight);
					rms_int = distCalc(&LidarRight, LIMIT);
				}
				while (stl == 0) {
					stl = VL_Init(&I2C2_Bus, &LidarLeft);
					lms_int = distCalc(&LidarLeft, LIMIT);
				}
				if ((stl == 1) && (str == 1)) {
					requestOff(PeripheralRequest, LIDAR_REQUEST_MASK);
					laserStep = 1;
				}
				break;
			case 1:
				while (stl == 0) {
					stl = readRangeSingleMillimeters(&I2C2_Bus, &LidarLeft);
				}
				while (str == 0) {
					str = readRangeSingleMillimeters(&I2C1_Bus, &LidarRight);
				}
				if ((stl == 1) && (str == 1)) {
					requestOff(PeripheralRequest, LIDAR_REQUEST_MASK);
					LL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
					str = 0; stl = 0;
				}
				break;
			default:
				laserStep = 0;
				break;
			}
		}

		if (PeripheralRequest & PACH_CALC_MASK) {
			dt = mainTick - previousTime;
			previousTime = mainTick;
			distL_mm = alphabeta(distL_mm, lms_int, 16);
			distR_mm = alphabeta(distR_mm, rms_int, 16);
			uint8_t mvst= MoveDrive(&bot, dt, target, distL_mm, distR_mm, wall);
			if (mvst) {
				target.x = 1.0f * rand() / RAND_MAX + 0.5;
				target.y = 1.0f * rand() / RAND_MAX + 0.5;
			}
			requestOff(PeripheralRequest, PACH_CALC_MASK);
		}


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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_HSE_EnableCSS();
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(72000000);
  LL_SetSystemCoreClock(72000000);
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
