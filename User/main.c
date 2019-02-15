/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   ��1.5.1�汾�⽨�Ĺ���ģ��
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

#include "stm32f4xx.h"
#include "SysTick.h"
#include "Modbus_svr.h"
#include "usart_cpt.h"
#include "usart_van.h"
#include "usart_spd.h"
#include "usart_alt.h"
#include "usart_dpt.h"
#include "gpio.h"
#include "bsp_innerflash.h"

extern short wReg[];
extern u8 bChanged;

extern uint8_t cpt_frame[];

/**
  * @brief  ������
  * @param  ��
  * @retval ��
  */
int main(void)
{

	SysTick_Init();
	GPIO_Config();
	//InternalFlashRead(wReg, 200);
	Flash_Read16BitDatas(FLASH_USER_START_ADDR, 200, wReg) ;

	Modbus_init();
	CPT_Init();
	VAN_Init();
	SPD_Init();
	DPT_Init();
	ALT_Init();

	SetTimer(0, 500);
	SetTimer(1, 1000);
	SetTimer(2, 200);
	SetTimer(3, 100);

	IWDG_Configuration();

	while (1)
	{
		Modbus_task();
		CPT_Task();
		VAN_Task();
		SPD_Task();
		DPT_Task();
		ALT_Task();

		if (GetTimer(0))
		{
			IWDG_Feed();
			LOGGLE_LED2;
			if (bSaved)
				Flash_Write16BitDatas(FLASH_USER_START_ADDR, 200, wReg) ;
		}

		if (GetTimer(2))
		{
			if (bChanged == 1)
				VAN_TransData();
			SPD_TxCmd();
		}

		if (GetTimer(1))
		{
			CPT_TxCmd();
			DPT_TxCmd();
			ALT_TxCmd();
		}

		if (GetTimer(3))
		{
			VAN_TxCmd();
		}
	}
}

/*********************************************END OF FILE**********************/
