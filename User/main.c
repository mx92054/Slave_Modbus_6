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
#include "gpio.h"
#include "bsp_innerflash.h"
#include <string.h>

#include "Mbsvr_comm.h"
#include "Modbus_svr.h"
#include "usart_com1.h"
#include "usart_spd1.h"
#include "usart_spd2.h"
#include "usart_spd3.h"
#include "usart_dam.h"

int main(void)
{
	SysTick_Init(); //tick��ʱ����ʼ
	GPIO_Config();  //GPIO��ʼ��

	Modbus_init(); //��λ��ͨ�ų�ʼ��

	SLV1_init();
	SLV2_init();
	SLV3_init();
	SLV4_init();
	SLV5_init();

	SetTimer(0, 500);
	SetTimer(1, 500);

	IWDG_Configuration(); //���Ź���ʼ

	while (1)
	{
		Modbus_task(); //ͨ�ų�������
		SLV1_task();
		SLV2_task();
		SLV3_task();
		SLV4_task();
		SLV5_task();

		if (GetTimer(0))
		{
			IWDG_Feed(); //���Ź���λ
			LOGGLE_LED2;
		}

		if (GetTimer(1))
		{
			if (mblock1.bSaved)
			{
				memcpy(Blk_SLV1.ptrRegs, mblock1.ptrRegs + 0, 50);
				memcpy(Blk_SLV2.ptrRegs, mblock1.ptrRegs + 50, 50);
				memcpy(Blk_SLV3.ptrRegs, mblock1.ptrRegs + 100, 50);
				memcpy(Blk_SLV4.ptrRegs, mblock1.ptrRegs + 150, 50);
				memcpy(Blk_SLV5.ptrRegs, mblock1.ptrRegs + 200, 50);
				mblock1.ptrRegs[251] += 10;
			}
			if (Blk_SLV1.bSaved)
			{
				memcpy(mblock1.ptrRegs + 0, Blk_SLV1.ptrRegs, 50);
				Blk_SLV1.bSaved = 0;
			}
			if (Blk_SLV2.bSaved)
			{
				memcpy(mblock1.ptrRegs + 50, Blk_SLV2.ptrRegs, 50);
				Blk_SLV2.bSaved = 0;
			}
			if (Blk_SLV3.bSaved)
			{
				memcpy(mblock1.ptrRegs + 100, Blk_SLV3.ptrRegs, 50);
				Blk_SLV3.bSaved = 0;
			}
			if (Blk_SLV4.bSaved)
			{
				memcpy(mblock1.ptrRegs + 150, Blk_SLV4.ptrRegs, 50);
				Blk_SLV4.bSaved = 0;
			}
			if (Blk_SLV5.bSaved)
			{
				memcpy(mblock1.ptrRegs + 200, Blk_SLV5.ptrRegs, 100);
				Blk_SLV5.bSaved = 0;
			}
			ModbusSvr_save_para(&mblock1);
		}
	}
}

/*********************************************END OF FILE**********************/
