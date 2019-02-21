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
#include "usart_dam.h"
#include "gpio.h"
#include "bsp_innerflash.h"

#include "spd_comm.h"

extern short wReg[];
extern u8 bChanged;

PID_Module pid1;
PID_Module pid2;
PID_Module pid3;

int main(void)
{

	SysTick_Init();												  //tick定时器初始
	GPIO_Config();												  //GPIO初始化
	Flash_Read16BitDatas(FLASH_USER_START_ADDR, 100, &wReg[100]); //通信寄存器初始化
	wReg[102]++;												  //启动次数加1
	bSaved = 1;													  //保存到EPROM

	Modbus_init(); //上位机通信初始化
	DPT_Init();	//深度计通信初始化
	SPD1_Init();   //1#编码器通信初始化
	SPD2_Init();   //2#编码器通信初始化
	SPD3_Init();   //3#编码器通信初始化
	DAM_Init();	//模拟量输出板初始化

	SetTimer(0, 500);
	SetTimer(1, 1000);
	SetTimer(2, 100);
	SetTimer(3, 100);
	SetTimer(4, 200);

	IWDG_Configuration(); //看门狗初始
	PIDMod_initialize(&pid1, 130);
	PIDMod_initialize(&pid2, 140);
	PIDMod_initialize(&pid3, 150);

	while (1)
	{
		Modbus_task(); //通信出来进程
		DPT_Task();
		SPD1_Task();
		SPD2_Task();
		SPD3_Task();
		DAM_Task();

		if (GetTimer(0))
		{
			IWDG_Feed(); //看门狗复位
			LOGGLE_LED2;
			if (bChanged > 10)
				bChanged = 0;
		}

		if (GetTimer(1) && bSaved)
		{
			Flash_Write16BitDatas(FLASH_USER_START_ADDR, 100, &wReg[100]); //保存修改过的寄存器
			bSaved = 0;
		}

		if (GetTimer(2))
		{
			PIDMod_step(&pid1);
			PIDMod_step(&pid2);
			PIDMod_step(&pid3);
		}

		if (GetTimer(3))
		{
			SPD1_TxCmd(); //向1#编码器发读取指令
			SPD2_TxCmd(); //向2#编码器发读取指令
			SPD3_TxCmd(); //向3#编码器发读取指令
		}

		if (GetTimer(4))
		{
			DAM_TxCmd();  //向模拟量输出板发出指令
		}
	}
}

/*********************************************END OF FILE**********************/
