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

	SysTick_Init();											//tick��ʱ����ʼ
	GPIO_Config();											//GPIO��ʼ��
	Flash_Read16BitDatas(FLASH_USER_START_ADDR, 200, wReg); //ͨ�żĴ�����ʼ��

	Modbus_init(); //��λ��ͨ�ų�ʼ��
	DPT_Init();		//��ȼ�ͨ�ų�ʼ��
	SPD1_Init();   //1#������ͨ�ų�ʼ��
	SPD2_Init();   //2#������ͨ�ų�ʼ��
	SPD3_Init();   //3#������ͨ�ų�ʼ��

	SetTimer(0, 500);
	SetTimer(1, 1000);
	SetTimer(2, 200);
	SetTimer(3, 100);

	IWDG_Configuration(); //���Ź���ʼ

	while (1)
	{
		Modbus_task(); //ͨ�ų�������
		DPT_Task();
		SPD1_Task();
		SPD2_Task();
		SPD3_Task();

		if (GetTimer(0))
		{
			IWDG_Feed(); //���Ź���λ
			LOGGLE_LED2;
			if (bChanged > 10)
				bChanged = 0;
		}

		if (GetTimer(1) && bSaved)
		{
			Flash_Write16BitDatas(FLASH_USER_START_ADDR, 200, wReg); //�����޸Ĺ��ļĴ���
			bSaved = 0;
		}

		if (GetTimer(3))
		{
			SPD1_TxCmd(); //��1#����������ȡָ��
			SPD2_TxCmd(); //��2#����������ȡָ��
			SPD3_TxCmd(); //��3#����������ȡָ��
		}
	}
}

/*********************************************END OF FILE**********************/
