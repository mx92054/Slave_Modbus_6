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
#include "usart_spd1.h"
#include "usart_spd2.h"
#include "gpio.h"
#include "bsp_innerflash.h"

extern short wReg[];
extern u8 bChanged;

extern uint8_t cpt_frame[];

/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void)
{

	SysTick_Init();
	GPIO_Config();
	Flash_Read16BitDatas(FLASH_USER_START_ADDR, 200, wReg);

	Modbus_init();
	SPD1_Init();
	SPD2_Init();

	SetTimer(0, 500);
	SetTimer(1, 1000);
	SetTimer(2, 200);
	SetTimer(3, 100);

	IWDG_Configuration();

	while (1)
	{
		Modbus_task();
		SPD1_Task();
		SPD2_Task();

		if (GetTimer(0))
		{
			IWDG_Feed();
			LOGGLE_LED2;
			if (bSaved)
			{
				Flash_Write16BitDatas(FLASH_USER_START_ADDR, 200, wReg);
				bSaved = 0 ;
			}

			if ( bChanged > 10)
				bChanged = 0;
		}

		if (GetTimer(3))
		{
			SPD1_TxCmd();
			SPD2_TxCmd();
		}
	}
}

/*********************************************END OF FILE**********************/
