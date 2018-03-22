#ifndef __BSP_GENTLESENSOR_H
#define __BSP_GENTLESENSOR_H

#ifdef __cplusplus
extern "C"{
#endif

#include "bsp_Common.h"
#include "main.h"
#include "stm32f1xx_hal.h"


BSP_StatusTypeDef BSP_GentleSensorInit(uint8_t nFilterCntSum);
BSP_StatusTypeDef BSP_GentleSensorCheck(void);





#ifdef __cplusplus
}
#endif
#endif

