#ifndef  __bsp_wrieless_h
#define  __bsp_wrieless_h

#ifdef __cplusplus
extern "C"{
#endif

#include "main.h"
#include "bsp_common.h"
#include "gpio.h"
#include "stm32f1xx_hal.h"

BSP_StatusTypeDef  BSP_WirelessInit(uint8_t nFilterSum);
BSP_StatusTypeDef  BSP_WirelessCheck(void);
BSP_StatusTypeDef  BSP_WirelessAction(void);



#ifdef __cplusplus
}
#endif

#endif
