#include "bsp_wrieless.h"
#include "bsp_motor.h"
#include "bsp_Log.h"
#include "BSP_DAC5571.h"




extern uint8_t gWrielessActionFlag;
extern uint8_t gWrielessModeFlag;
extern MOTORMACHINE gMotorMachine;

static uint8_t wrielessCurrentReadValue;
static uint8_t wrielessLastReadValue;
static uint8_t wrielessFilterCnt;
static uint8_t wrielessFilterSum;
static uint8_t wrielessGpioState;
static uint8_t wrielessGpioRelease;


BSP_StatusTypeDef  BSP_WirelessInit(uint8_t nFilterSum)
{
	BSP_StatusTypeDef state = BSP_OK;
	wrielessFilterSum = nFilterSum;

	return state;
}

BSP_StatusTypeDef  BSP_WirelessCheck(void)
{
	BSP_StatusTypeDef state = BSP_OK;
	wrielessCurrentReadValue = HAL_GPIO_ReadPin(MCU_WIRLESSA_GPIO_Port, MCU_WIRLESSA_Pin);
	if(1 == wrielessCurrentReadValue && 1 == wrielessLastReadValue)
	{
		if(0 == wrielessGpioState && wrielessGpioRelease)
		{
			wrielessFilterCnt ++;
			if(wrielessFilterCnt > wrielessFilterSum)
			{
				wrielessGpioState = 1;
				wrielessFilterCnt = 0;
				wrielessGpioRelease = 0;
			}
		}
	}
	else
	{
		wrielessFilterCnt = 0;
		wrielessGpioState = 0;
		wrielessGpioRelease = 1;
	}
	wrielessLastReadValue = wrielessCurrentReadValue;

	if(0 == wrielessGpioState)
	{
		return state;
	}

	

	//在水平位置执行开闸操作
	if(1 == gMotorMachine.HorizontalRasterState && 0 == gMotorMachine.RunningState)
	{
		gWrielessActionFlag = 2;
		gWrielessModeFlag = 1;
		wrielessGpioState = 0;
		return state;
	}

	//在垂直位置执行关闸操作
	if(1 == gMotorMachine.VerticalRasterState && 0 == gMotorMachine.RunningState)
	{
		gWrielessActionFlag = 3;
		gWrielessModeFlag = 1;
		wrielessGpioState = 0;
		return state;
	}

	//处在暂停位置时进行开闸操作
	if(1 == gWrielessActionFlag && 0 == gMotorMachine.RunningState)
	{
		gWrielessActionFlag = 2;
		gWrielessModeFlag = 1;
		wrielessGpioState = 0;
		return state;
	}
	
	//处在运动状态时，按下遥控则暂停
	if(1 == gMotorMachine.RunningState)
	{
		gWrielessActionFlag = 1; //暂停
		gWrielessModeFlag = 1;	//处在遥控开闸模式
		wrielessGpioState = 0;
		return state;
	}
	return state;
}

BSP_StatusTypeDef  BSP_WirelessAction(void)
{
	BSP_StatusTypeDef state = BSP_OK;

	switch (gWrielessActionFlag)
		{
		case 1:
			BSP_MotorStop();
			gMotorMachine.RunningState = 0;
			BSP_Log_UpOpenMode(1 + (gWrielessModeFlag << 4));
			break;
			
		case 2:
			gMotorMachine.RunningState = 1;
			gMotorMachine.RunDir = UPDIR;
			BSP_MotorRun(UPDIR);
			BSP_DAC5571_WriteValue(NormalOperationMode, 100);
			BSP_Log_UpOpenMode(2 + (gWrielessModeFlag << 4));
			break;

	    case 3:
			gMotorMachine.RunningState = 1;
			gMotorMachine.RunDir = DOWNDIR;
			BSP_MotorRun(DOWNDIR);
			BSP_DAC5571_WriteValue(NormalOperationMode, 80);
			BSP_Log_UpOpenMode(3 + (gWrielessModeFlag << 4));
			break;
			
	default:
		BSP_Log_UpOpenMode(0 + (gWrielessModeFlag << 4));
		break;
		
		}
	return state;
}

//END OF FILE 

