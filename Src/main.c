/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "bsp_DataTransmissionLayer.h"
#include "bsp_ProtocolLayer.h"
#include "bsp_motor.h"
#include "bsp_AirSensor.h"
#include "bsp_Common.h"
#include "bsp_GentleSensor.h"
#include "BSP_DAC5571.h"
#include "bsp_led.h"
#include "bsp_Log.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define CODEVERSION			0x0201

MOTORMACHINE gMotorMachine;
GPIOSTRUCT gHorGpio;
GPIOSTRUCT gVerGpio;
GPIOSTRUCT gOpenBoxGpio;	//开箱检测GPIO
GPIOSTRUCT gGentleSensorGpio;

uint8_t gRevOpenFlag;	//接收到开闸指令标记位
uint8_t gEnterTimeoutFlag;//出入场超时标记
uint8_t gCarEnteredFlag;  //车辆完成出入场标记位

uint16_t Tim4Cnt;
uint8_t  Tim4Flag;

uint16_t Tim5Cnt;
uint8_t  Tim5Flag;

uint16_t Tim5LedCnt;
uint8_t  Tim5LedFlag;

uint32_t Tim5LogCnt;
uint8_t  Tim5LogFlag;

uint32_t Tim5EnterTimeoutSum;//单位为ms
uint32_t Tim5EnterTimeoutCnt;

uint16_t Tim6Cnt;
uint8_t  Tim6Flag;
/* 电流及温度采集 */
uint32_t gADCBuffer[10];
uint32_t gIValue;
uint32_t gTempertureValue;

/* 电机调速 */
uint8_t  Tim6OpenSpeedCnt;
uint8_t  Tim6OpenSpeedFlag;
uint8_t  gOpenSpeedFlag;
uint16_t gSpeed;

uint8_t gCloseFlag;

extern uint8_t gOpenBControlBuf[300];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void RasterStateCheck(void);
static void CheckCarEnterFlag(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim4); 	//0.1ms
  HAL_TIM_Base_Start_IT(&htim5);	//1ms
  HAL_TIM_Base_Start_IT(&htim6);	//

  BSP_MotorInit();
  BSP_AirSensorInit(5);
  BSP_GentleSensorInit(5);
  BSP_DriverBoardProtocolInit();
  BSP_Log_Init(0x12345678);

  BSP_DAC5571_Init(NormalOperationMode);
  BSP_DAC5571_WriteValue(NormalOperationMode, 100);
  
  gHorGpio.FilterCntSum = 10;
  gVerGpio.FilterCntSum = 10;
  Tim5EnterTimeoutSum = 15000; //15s

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  BSP_HandingUartDataFromDriverBoard();
  BSP_HandingDriverBoardRequest();
  BSP_SendAckData();


  //检测光栅状态
  if(Tim4Flag)
  {
	RasterStateCheck();
	Tim4Flag = 0;
  }
  //检测空气压力传感器以及地感传感器
  if(Tim5Flag)
  {
  	BSP_AirSensorChecked();
	BSP_GentleSensorCheck();
	Tim5Flag = 0;
  }

  BSP_MotorCheckA();
  BSP_MotorActionA();
  
  //检测车辆是否进出场
  CheckCarEnterFlag();

  /*  开闸调速 */
  if(Tim6OpenSpeedFlag)
  {
	if(gOpenSpeedFlag)
	{
		BSP_DAC5571_WriteValue(NormalOperationMode,gOpenBControlBuf[gSpeed]);
		//调试使用
		BSP_SendByteToDriverBoard(gOpenBControlBuf[gSpeed], 0xFFFF);
		gSpeed++;
		if(gSpeed > 299)
		{
			gSpeed = 299;
		}
	}
	else
	{
		gSpeed = 0;
	}
	Tim6OpenSpeedFlag = 0;
  }
	//设置氛围灯
	if(Tim5LedFlag)
	{
		BSP_LEDCheck();
		Tim5LedFlag = 0;
	}
	if(Tim5LogFlag)
	{
		Tim5LogFlag = 0;
		BSP_Log_WriteFlag();
	}
	//检测上报日志信息
	BSP_Log_CheckReportInfo();
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* TIM5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM5_IRQn);
  /* TIM4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* ADC1_2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
  /* I2C2_EV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C2_EV_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
  /* I2C2_ER_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C2_ER_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
{
	UNUSED(htim);

	if(htim4.Instance == htim->Instance)
	{
		Tim4Flag = 1;
	}

	if(htim5.Instance == htim->Instance)
	{
		Tim5Flag= 1;
		if(gRevOpenFlag)
		{
			Tim5EnterTimeoutCnt ++;
			gCloseFlag = 1;
			if(Tim5EnterTimeoutCnt > Tim5EnterTimeoutSum)
			{
				gRevOpenFlag = 0;
				gEnterTimeoutFlag = 1;
				Tim5EnterTimeoutCnt = 0;
				/* 写关闸操作 */
				gCloseFlag = 0;
			}
		}
		Tim5LedCnt++;
		if(Tim5LedCnt > 200)
		{
			Tim5LedFlag = 1;
			Tim5LedCnt = 0;
		}

		Tim5LogCnt++;
		if(Tim5LogCnt > 10000)
		{
			Tim5LogFlag = 1;
			Tim5LogCnt = 0;
		}
	}

	if(htim6.Instance == htim->Instance)
	{
		Tim6OpenSpeedCnt++;
		if(Tim6OpenSpeedCnt > 1)
		{
			Tim6OpenSpeedCnt = 0;
			Tim6OpenSpeedFlag = 1;
		}
	}
}

static void RasterStateCheck(void)
{
	gHorGpio.CurrentReadVal = HAL_GPIO_ReadPin(HorRasterInput_GPIO_Port,HorRasterInput_Pin);
	gVerGpio.CurrentReadVal = HAL_GPIO_ReadPin(VerRasterInput_GPIO_Port,VerRasterInput_Pin);

	if(0 == gHorGpio.CurrentReadVal && 0 == gHorGpio.LastReadVal)
	{
		gHorGpio.FilterCnt ++;
		if(gHorGpio.FilterCnt > gHorGpio.FilterCntSum)
		{
			gMotorMachine.HorizontalRasterState = 1;
			gHorGpio.FilterCnt = 0;
			if(DOWNDIR == gMotorMachine.RunDir)
			{
				gMotorMachine.RunDir = UPDIR;
				gMotorMachine.RunningState = 0;
				BSP_MotorStop();
				if(1 == gEnterTimeoutFlag)
				{
					gEnterTimeoutFlag = 2;
				}
				if(1 == gCarEnteredFlag)
				{
					gRevOpenFlag = 0;
					gCarEnteredFlag = 2;
				}
			}
		}
	}
	else
	{
		gMotorMachine.HorizontalRasterState = 0;
		gHorGpio.FilterCnt = 0;
	}
	gHorGpio.LastReadVal = gHorGpio.CurrentReadVal;

	if(0 == gVerGpio.CurrentReadVal &&0 == gVerGpio.LastReadVal)
	{
		gVerGpio.FilterCnt ++;
		if(gVerGpio.FilterCnt > gVerGpio.FilterCntSum)
		{
			gMotorMachine.VerticalRasterState = 1;
			gVerGpio.FilterCnt = 0;
			if(UPDIR == gMotorMachine.RunDir)
			{
				gMotorMachine.RunDir = DOWNDIR;
				gMotorMachine.RunningState = 0;
				BSP_MotorStop();
				if(gMotorMachine.EncounteredFlag)
				{
					gMotorMachine.EncounteredFlag = 0;
					Tim5EnterTimeoutCnt = 0;
				}
			}	
		}	
	}
	else
	{
		gMotorMachine.VerticalRasterState = 0;
		gVerGpio.FilterCnt = 0;
	}
	gVerGpio.LastReadVal = gVerGpio.CurrentReadVal;
}

void CheckCarEnterFlag(void)
{
	uint8_t pData[7];
	pData[0] = 0x5B;
	pData[1] = 0xE3;
	pData[3] = 0x00;
	pData[4] = 0x00;
	pData[6] = 0x5D;

	if(2 == gCarEnteredFlag)
	{
		pData[2] = 0x00;
		pData[5] = 0xE3;
		gCarEnteredFlag = 0;

		/*数据发送*/
		BSP_SendDataToDriverBoard(pData,7,0xFFFF);
		return;
	}

	if(2 == gEnterTimeoutFlag)
	{
		pData[2] = 0x01;
		pData[5] = 0xE2;
		gEnterTimeoutFlag = 0;

		/*数据发送*/
		BSP_SendDataToDriverBoard(pData,7,0xFFFF);
		return;
	}
	
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
