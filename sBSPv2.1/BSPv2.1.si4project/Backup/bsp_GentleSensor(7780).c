#include "bsp_GentleSensor.h"
#include "bsp_Motor.h"

extern uint8_t gCarEnteredFlag;
extern GPIOSTRUCT gGentleSensorGpio;
extern MOTORMACHINE gMotorMachine;

BSP_StatusTypeDef BSP_GentleSensorInit(uint8_t nFilterCntSum)
{
	BSP_StatusTypeDef state = BSP_OK;
	
	gGentleSensorGpio.FilterCntSum = nFilterCntSum;

	return state;
}
BSP_StatusTypeDef BSP_GentleSensorCheck(void)
{
	BSP_StatusTypeDef state = BSP_OK;
	gGentleSensorGpio.CurrentReadVal = HAL_GPIO_ReadPin(GentleSensor_GPIO_Port,GentleSensor_Pin);

	if(0 == gGentleSensorGpio.CurrentReadVal && 0 == gGentleSensorGpio.LastReadVal)
	{
		if(0 == gMotorMachine.GentleSensorFlag)
		{
			gGentleSensorGpio.FilterCnt ++;
			if(gGentleSensorGpio.FilterCnt > gGentleSensorGpio.FilterCntSum)
			{
				gGentleSensorGpio.GpioState = 1;
				gGentleSensorGpio.FilterCnt = 0;
				gMotorMachine.GentleSensorFlag = 1;
			}
			
		}
	}
	else
	{
		if(gGentleSensorGpio.GpioState)
		{
			gCarEnteredFlag = 1;
		}
		gMotorMachine.GentleSensorFlag = 0;
		gGentleSensorGpio.GpioState = 0;
		gGentleSensorGpio.FilterCnt = 0;
	}
	gGentleSensorGpio.LastReadVal = gGentleSensorGpio.CurrentReadVal;
	return state;
}



