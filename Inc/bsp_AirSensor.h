#ifndef __BSP_AIRSENSOR_H
#define __BSP_AIRSENSOR_H


#ifdef __cplusplus
extern "c"{
#endif

#include "gpio.h"
#include "main.h"
#include "bsp_common.h"

struct tAirSensor
{
	uint8_t FilterCnt;
	uint8_t FilterCntSum;
	uint8_t CheckedFlag;
	uint8_t CurrentReadValue;
	uint8_t LastReadValue;
};

typedef struct tAirSensor AirSensor, *pAirSensor;

BSP_StatusTypeDef BSP_AirSensorInit(uint8_t nFilterSum);
BSP_StatusTypeDef BSP_AirSensorChecked(void);



#ifdef __cplusplus
}
#endif 
#endif

