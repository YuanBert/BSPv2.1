#include "bsp_LockGpio.h"
#include "bsp_Can.h"

extern uint8_t CanId;

static uint8_t CtrlBuffer[22]= {0};
static uint8_t NeedHandBufferFlag;


static uint8_t LockGoodsStateChangedBuffer[22][2] = {0};
static uint8_t LockPinsStateChangedBuffer[22][2] = {0};

//LOCKERRORCODE BSP_LOCKInit(void);

LOCKERRORCODE BSP_LOCKUpdateOfGoodsState()
{
	LOCKERRORCODE state = LOCK_OK;
        
	if(LockGoodsStateChangedBuffer[0][0] != GetLockGoods1State)
	{
		LockGoodsStateChangedBuffer[0][1] = 1;
	}
	
	if(LockGoodsStateChangedBuffer[1][0] != GetLockGoods1State)
	{
		LockGoodsStateChangedBuffer[1][1] = 1;
	}

	if(LockGoodsStateChangedBuffer[2][0] != GetLockGoods1State)
	{
		LockGoodsStateChangedBuffer[2][1] = 1;
	}

	if(LockGoodsStateChangedBuffer[3][0] != GetLockGoods1State)
	{
		LockGoodsStateChangedBuffer[3][1] = 1;
	}

	if(LockGoodsStateChangedBuffer[4][0] != GetLockGoods1State)
	{
		LockGoodsStateChangedBuffer[4][1] = 1;
	}
	if(LockGoodsStateChangedBuffer[5][0] != GetLockGoods1State)
	{
		LockGoodsStateChangedBuffer[5][1] = 1;
	}
	if(LockGoodsStateChangedBuffer[6][0] != GetLockGoods1State)
	{
		LockGoodsStateChangedBuffer[6][1] = 1;
	}
	if(LockGoodsStateChangedBuffer[7][0] != GetLockGoods1State)
	{
		LockGoodsStateChangedBuffer[7][1] = 1;
	}
	if(LockGoodsStateChangedBuffer[8][0] != GetLockGoods1State)
	{
		LockGoodsStateChangedBuffer[8][1] = 1;
	}
	if(LockGoodsStateChangedBuffer[9][0] != GetLockGoods1State)
	{
		LockGoodsStateChangedBuffer[9][1] = 1;
	}
	if(LockGoodsStateChangedBuffer[10][0] != GetLockGoods1State)
	{
		LockGoodsStateChangedBuffer[10][1] = 1;
	}
	
	if(LockGoodsStateChangedBuffer[11][0] != GetLockGoods1State)
	{
		LockGoodsStateChangedBuffer[11][1] = 1;
	}

	if(LockGoodsStateChangedBuffer[12][0] != GetLockGoods1State)
	{
		LockGoodsStateChangedBuffer[12][1] = 1;
	}

	if(LockGoodsStateChangedBuffer[13][0] != GetLockGoods1State)
	{
		LockGoodsStateChangedBuffer[13][1] = 1;
	}

	if(LockGoodsStateChangedBuffer[14][0] != GetLockGoods1State)
	{
		LockGoodsStateChangedBuffer[14][1] = 1;
	}
	if(LockGoodsStateChangedBuffer[15][0] != GetLockGoods1State)
	{
		LockGoodsStateChangedBuffer[15][1] = 1;
	}
	if(LockGoodsStateChangedBuffer[16][0] != GetLockGoods1State)
	{
		LockGoodsStateChangedBuffer[16][1] = 1;
	}
	if(LockGoodsStateChangedBuffer[17][0] != GetLockGoods1State)
	{
		LockGoodsStateChangedBuffer[17][1] = 1;
	}
	if(LockGoodsStateChangedBuffer[18][0] != GetLockGoods1State)
	{
		LockGoodsStateChangedBuffer[18][1] = 1;
	}
	if(LockGoodsStateChangedBuffer[19][0] != GetLockGoods1State)
	{
		LockGoodsStateChangedBuffer[19][1] = 1;
	}
	if(LockGoodsStateChangedBuffer[20][0] != GetLockGoods1State)
	{
		LockGoodsStateChangedBuffer[20][1] = 1;
	}
	if(LockGoodsStateChangedBuffer[21][0] != GetLockGoods1State)
	{
		LockGoodsStateChangedBuffer[21][1] = 1;
	}


	
	LockGoodsStateChangedBuffer[0][0] = GetLockGoods1State;
	LockGoodsStateChangedBuffer[1][0] = GetLockGoods2State;
	LockGoodsStateChangedBuffer[2][0] = GetLockGoods3State;
	LockGoodsStateChangedBuffer[3][0] = GetLockGoods4State;
	LockGoodsStateChangedBuffer[4][0] = GetLockGoods5State;
	LockGoodsStateChangedBuffer[5][0] = GetLockGoods6State;
	LockGoodsStateChangedBuffer[6][0] = GetLockGoods7State;
	LockGoodsStateChangedBuffer[7][0] = GetLockGoods8State;
	LockGoodsStateChangedBuffer[8][0] = GetLockGoods9State;
	LockGoodsStateChangedBuffer[9][0] = GetLockGoods10State;
	LockGoodsStateChangedBuffer[10][0] = GetLockGoods11State;
	LockGoodsStateChangedBuffer[11][0] = GetLockGoods12State;
	LockGoodsStateChangedBuffer[12][0] = GetLockGoods13State;
	LockGoodsStateChangedBuffer[13][0] = GetLockGoods14State;
	LockGoodsStateChangedBuffer[14][0] = GetLockGoods15State;
	LockGoodsStateChangedBuffer[15][0] = GetLockGoods16State;
	LockGoodsStateChangedBuffer[16][0] = GetLockGoods17State;
	LockGoodsStateChangedBuffer[17][0] = GetLockGoods18State;
	LockGoodsStateChangedBuffer[18][0] = GetLockGoods19State;
	LockGoodsStateChangedBuffer[19][0] = GetLockGoods20State;
	LockGoodsStateChangedBuffer[20][0] = GetLockGoods21State;
	LockGoodsStateChangedBuffer[21][0] = GetLockGoods22State;

	return state;
}

LOCKERRORCODE BSP_LOCKSendGoodsChangedMessage(void)
{
	LOCKERRORCODE state = LOCK_OK;
	uint8_t i = 0;
	for(i = 0; i < 22; i++)
	{
		if(1 == LockGoodsStateChangedBuffer[i][1])
		{
			state = BSP_CanSendDatas(CanId, i, LockPinsStateChangedBuffer[i][0], LockGoodsStateChangedBuffer[i][0]);
			LockGoodsStateChangedBuffer[i][1] = 0;
		}
		
	}

	return state;
}
LOCKERRORCODE BSP_LOCKUpdateOfLockPinsState(void)
{
	LOCKERRORCODE state = LOCK_OK;
	if(LockPinsStateChangedBuffer[0][0] != GetLockPins1State)
	{
		LockPinsStateChangedBuffer[0][1] = 1;
	}
	if(LockPinsStateChangedBuffer[1][0] != GetLockPins2State)
	{
		LockPinsStateChangedBuffer[1][1] = 1;
	}
	if(LockPinsStateChangedBuffer[2][0] != GetLockPins3State)
	{
		LockPinsStateChangedBuffer[2][1] = 1;
	}
	if(LockPinsStateChangedBuffer[3][0] != GetLockPins4State)
	{
		LockPinsStateChangedBuffer[3][1] = 1;
	}
	if(LockPinsStateChangedBuffer[4][0] != GetLockPins5State)
	{
		LockPinsStateChangedBuffer[4][1] = 1;
	}
	if(LockPinsStateChangedBuffer[5][0] != GetLockPins6State)
	{
		LockPinsStateChangedBuffer[5][1] = 1;
	}
	if(LockPinsStateChangedBuffer[6][0] != GetLockPins7State)
	{
		LockPinsStateChangedBuffer[6][1] = 1;
	}
	if(LockPinsStateChangedBuffer[7][0] != GetLockPins8State)
	{
		LockPinsStateChangedBuffer[7][1] = 1;
	}
	if(LockPinsStateChangedBuffer[8][0] != GetLockPins9State)
	{
		LockPinsStateChangedBuffer[8][1] = 1;
	}
	if(LockPinsStateChangedBuffer[9][0] != GetLockPins10State)
	{
		LockPinsStateChangedBuffer[9][1] = 1;
	}
	
	if(LockPinsStateChangedBuffer[10][0] != GetLockPins11State)
	{
		LockPinsStateChangedBuffer[10][1] = 1;
	}
	if(LockPinsStateChangedBuffer[11][0] != GetLockPins12State)
	{
		LockPinsStateChangedBuffer[11][1] = 1;
	}
	if(LockPinsStateChangedBuffer[12][0] != GetLockPins13State)
	{
		LockPinsStateChangedBuffer[12][1] = 1;
	}
	if(LockPinsStateChangedBuffer[13][0] != GetLockPins14State)
	{
		LockPinsStateChangedBuffer[13][1] = 1;
	}
	if(LockPinsStateChangedBuffer[14][0] != GetLockPins15State)
	{
		LockPinsStateChangedBuffer[14][1] = 1;
	}
	if(LockPinsStateChangedBuffer[15][0] != GetLockPins16State)
	{
		LockPinsStateChangedBuffer[15][1] = 1;
	}
	if(LockPinsStateChangedBuffer[16][0] != GetLockPins17State)
	{
		LockPinsStateChangedBuffer[16][1] = 1;
	}
	if(LockPinsStateChangedBuffer[17][0] != GetLockPins18State)
	{
		LockPinsStateChangedBuffer[17][1] = 1;
	}
	if(LockPinsStateChangedBuffer[18][0] != GetLockPins19State)
	{
		LockPinsStateChangedBuffer[18][1] = 1;
	}
	if(LockPinsStateChangedBuffer[19][0] != GetLockPins20State)
	{
		LockPinsStateChangedBuffer[19][1] = 1;
	}
	if(LockPinsStateChangedBuffer[20][0] != GetLockPins21State)
	{
		LockPinsStateChangedBuffer[20][1] = 1;
	}
	if(LockPinsStateChangedBuffer[21][0] != GetLockPins22State)
	{
		LockPinsStateChangedBuffer[21][1] = 1;
	}

	LockPinsStateChangedBuffer[0][0] = GetLockPins1State;
	LockPinsStateChangedBuffer[1][0] = GetLockPins2State;
	LockPinsStateChangedBuffer[2][0] = GetLockPins3State;
	LockPinsStateChangedBuffer[3][0] = GetLockPins4State;
	LockPinsStateChangedBuffer[4][0] = GetLockPins5State;
	LockPinsStateChangedBuffer[5][0] = GetLockPins6State;
	LockPinsStateChangedBuffer[6][0] = GetLockPins7State;
	LockPinsStateChangedBuffer[7][0] = GetLockPins8State;
	LockPinsStateChangedBuffer[8][0] = GetLockPins9State;
	LockPinsStateChangedBuffer[9][0] = GetLockPins10State;
	LockPinsStateChangedBuffer[10][0] = GetLockPins11State;
	LockPinsStateChangedBuffer[11][0] = GetLockPins12State;
	LockPinsStateChangedBuffer[12][0] = GetLockPins13State;
	LockPinsStateChangedBuffer[13][0] = GetLockPins14State;
	LockPinsStateChangedBuffer[14][0] = GetLockPins15State;
	LockPinsStateChangedBuffer[15][0] = GetLockPins16State;
	LockPinsStateChangedBuffer[16][0] = GetLockPins17State;
	LockPinsStateChangedBuffer[17][0] = GetLockPins18State;
	LockPinsStateChangedBuffer[18][0] = GetLockPins19State;
	LockPinsStateChangedBuffer[19][0] = GetLockPins20State;
	LockPinsStateChangedBuffer[20][0] = GetLockPins21State;
	LockPinsStateChangedBuffer[21][0] = GetLockPins22State;
	return state;
}
LOCKERRORCODE BSP_LOCKSendPinsChangedMessage(void)
{
	LOCKERRORCODE state = LOCK_OK;
	uint8_t i = 0;
	for(i = 0; i < 22; i++)
	{
		if(1 == LockPinsStateChangedBuffer[i][1])
		{
			state = BSP_CanSendDatas(CanId, i, LockPinsStateChangedBuffer[i][0], LockGoodsStateChangedBuffer[i][0]);
			LockPinsStateChangedBuffer[i][1] = 0;
		}
		
	}

	return state;

}
LOCKERRORCODE BSP_LOCKWriteCtrlPin(uint8_t nBoxNumber, CtrlState nCtrlState)
{
	LOCKERRORCODE state = LOCK_OK;
	if(nBoxNumber < 0 || nBoxNumber > 21)
	{
		state = LOCK_ERROR;
		return state;
	}

	switch (nBoxNumber)
		{
		case 0:
			HAL_GPIO_WritePin(LOCK_CTRL1_GPIO_Port,LOCK_CTRL1_Pin,(GPIO_PinState)nCtrlState);
			break;
			
		case 1:
			HAL_GPIO_WritePin(LOCK_CTRL2_GPIO_Port,LOCK_CTRL2_Pin,(GPIO_PinState)nCtrlState);
			break;
			
		case 2:
			HAL_GPIO_WritePin(LOCK_CTRL3_GPIO_Port,LOCK_CTRL3_Pin,(GPIO_PinState)nCtrlState);		
			break;
		case 3:
			HAL_GPIO_WritePin(LOCK_CTRL4_GPIO_Port,LOCK_CTRL4_Pin,(GPIO_PinState)nCtrlState);
			break;
			
		case 4:
			HAL_GPIO_WritePin(LOCK_CTRL5_GPIO_Port,LOCK_CTRL5_Pin,(GPIO_PinState)nCtrlState);
			break;
			
		case 5:
			HAL_GPIO_WritePin(LOCK_CTRL6_GPIO_Port,LOCK_CTRL6_Pin,(GPIO_PinState)nCtrlState);		
			break;		
		case 6:
			HAL_GPIO_WritePin(LOCK_CTRL7_GPIO_Port,LOCK_CTRL7_Pin,(GPIO_PinState)nCtrlState);
			break;
			
		case 7:
			HAL_GPIO_WritePin(LOCK_CTRL8_GPIO_Port,LOCK_CTRL8_Pin,(GPIO_PinState)nCtrlState);
			break;
			
		case 8:
			HAL_GPIO_WritePin(LOCK_CTRL9_GPIO_Port,LOCK_CTRL9_Pin,(GPIO_PinState)nCtrlState);		
			break;
		case 9:
			HAL_GPIO_WritePin(LOCK_CTRL10_GPIO_Port,LOCK_CTRL10_Pin,(GPIO_PinState)nCtrlState);
			break;
			
		case 10:
			HAL_GPIO_WritePin(LOCK_CTRL11_GPIO_Port,LOCK_CTRL11_Pin,(GPIO_PinState)nCtrlState);
			break;
			
		case 11:
			HAL_GPIO_WritePin(LOCK_CTRL12_GPIO_Port,LOCK_CTRL12_Pin,(GPIO_PinState)nCtrlState);
			break;
			
		case 12:
			HAL_GPIO_WritePin(LOCK_CTRL13_GPIO_Port,LOCK_CTRL13_Pin,(GPIO_PinState)nCtrlState);		
			break;
		case 13:
			HAL_GPIO_WritePin(LOCK_CTRL14_GPIO_Port,LOCK_CTRL14_Pin,(GPIO_PinState)nCtrlState);
			break;
			
		case 14:
			HAL_GPIO_WritePin(LOCK_CTRL15_GPIO_Port,LOCK_CTRL15_Pin,(GPIO_PinState)nCtrlState);
			break;
			
		case 15:
			HAL_GPIO_WritePin(LOCK_CTRL16_GPIO_Port,LOCK_CTRL16_Pin,(GPIO_PinState)nCtrlState);		
			break;		
		case 16:
			HAL_GPIO_WritePin(LOCK_CTRL17_GPIO_Port,LOCK_CTRL17_Pin,(GPIO_PinState)nCtrlState);
			break;
			
		case 17:
			HAL_GPIO_WritePin(LOCK_CTRL18_GPIO_Port,LOCK_CTRL18_Pin,(GPIO_PinState)nCtrlState);
			break;
			
		case 18:
			HAL_GPIO_WritePin(LOCK_CTRL19_GPIO_Port,LOCK_CTRL19_Pin,(GPIO_PinState)nCtrlState);		
			break;
		case 19:
			HAL_GPIO_WritePin(LOCK_CTRL20_GPIO_Port,LOCK_CTRL20_Pin,(GPIO_PinState)nCtrlState);
			break;
			
		case 20:
			HAL_GPIO_WritePin(LOCK_CTRL21_GPIO_Port,LOCK_CTRL21_Pin,(GPIO_PinState)nCtrlState);
			break;						
		case 21:
			HAL_GPIO_WritePin(LOCK_CTRL22_GPIO_Port,LOCK_CTRL22_Pin,(GPIO_PinState)nCtrlState);
			break;						
		default:
			
			break;
		}

	return state;
}


LOCKERRORCODE BSP_LOCKWriteCtrlBuffer(uint8_t nBoxNumber)
{
	LOCKERRORCODE state = LOCK_OK;
	if(nBoxNumber < 0 || nBoxNumber > 21)
	{
		state = LOCK_ERROR;
		return state;
	}
	
	CtrlBuffer[nBoxNumber] = 1;
	NeedHandBufferFlag = 1;
	return state;
}
LOCKERRORCODE BSP_LOCKCheckCtrlBuffer(void)
{
	LOCKERRORCODE state = LOCK_OK;
	if(0 == NeedHandBufferFlag)
	{
		return state;
	}
	uint8_t i = 0;
	for(i = 0; i < 22; i++)
	{
		if(1 == CtrlBuffer[i])
		{
			CtrlBuffer[i] = 0;
			BSP_LOCKWriteCtrlPin(i,CtrlOpen);
		}
	}
	NeedHandBufferFlag = 0;
	return state;
}


