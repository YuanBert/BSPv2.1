#include "bsp_AirSensor.h"

AirSensor gAirSensor;


BSP_StatusTypeDef BSP_AirSensorInit(uint8_t nFilterSum)
{
	BSP_StatusTypeDef state = BSP_OK;
	gAirSensor.FilterCntSum = nFilterSum;
	return state;
}

BSP_StatusTypeDef BSP_AirSensorChecked(void)
{
	BSP_StatusTypeDef state = BSP_OK;
	gAirSensor.CurrentReadValue = HAL_GPIO_ReadPin(MCU_AIR_GPIO_Port, MCU_AIR_Pin);
	if(0 == gAirSensor.CurrentReadValue && 0 == gAirSensor.LastReadValue)
	{
		if(0 == gAirSensor.CheckedFlag)
		{
			gAirSensor.FilterCnt++;
			if(gAirSensor.FilterCnt > gAirSensor.FilterCntSum)
			{
				gAirSensor.CheckedFlag = 1;
				gAirSensor.FilterCnt = 0;
			}
		}
	}
	else
	{
		gAirSensor.CheckedFlag = 0;
		gAirSensor.FilterCnt = 0;
	}
	return state;
}



//

