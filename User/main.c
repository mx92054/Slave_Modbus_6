/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   用1.5.1版本库建的工程模板
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

#include "stm32f4xx.h"
#include "SysTick.h"
#include "Modbus_svr.h"
#include "usart_dpt.h"
#include "usart_spd1.h"
#include "usart_spd2.h"
#include "usart_spd3.h"
#include "gpio.h"
#include "bsp_innerflash.h"

extern short wReg[];
extern u8 bChanged;

int main(void)
{

	SysTick_Init();											//tick定时器初始
	GPIO_Config();											//GPIO初始化
	Flash_Read16BitDatas(FLASH_USER_START_ADDR, 200, wReg); //通信寄存器初始化

	Modbus_init(); //上位机通信初始化
	DPT_Init();		//深度计通信初始化
	SPD1_Init();   //1#编码器通信初始化
	SPD2_Init();   //2#编码器通信初始化
	SPD3_Init();   //3#编码器通信初始化

	SetTimer(0, 500);
	SetTimer(1, 1000);
	SetTimer(2, 200);
	SetTimer(3, 100);

	IWDG_Configuration(); //看门狗初始

	while (1)
	{
		Modbus_task(); //通信出来进程
		DPT_Task();
		SPD1_Task();
		SPD2_Task();
		SPD3_Task();

		if (GetTimer(0))
		{
			IWDG_Feed(); //看门狗复位
			LOGGLE_LED2;
			if (bChanged > 10)
				bChanged = 0;
		}

		if (GetTimer(1) && bSaved)
		{
			Flash_Write16BitDatas(FLASH_USER_START_ADDR, 200, wReg); //保存修改过的寄存器
			bSaved = 0;
		}

		if (GetTimer(3))
		{
			SPD1_TxCmd(); //向1#编码器发读取指令
			SPD2_TxCmd(); //向2#编码器发读取指令
			SPD3_TxCmd(); //向3#编码器发读取指令
		}
	}
}

/*********************************************END OF FILE**********************/
