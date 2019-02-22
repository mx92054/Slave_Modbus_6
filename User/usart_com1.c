#include "usart_com1.h"
#include "Modbus_svr.h"
#include <stdio.h>
#include "stm32f4xx_conf.h"
#include "SysTick.h"


Modbus_block Blk_SLV1;
//-------------------------------------------------------------------------------
//	@brief	�жϳ�ʼ��
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
static void SLV1_NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Ƕ�������жϿ�������ѡ�� */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	/* ����USARTΪ�ж�Դ */
	NVIC_InitStructure.NVIC_IRQChannel = SLV1_USART_IRQ;
	/* �������ȼ�Ϊ1 */
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	/* �����ȼ�Ϊ1 */
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	/* ʹ���ж� */
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	/* ��ʼ������NVIC */
	NVIC_Init(&NVIC_InitStructure);
}

//-------------------------------------------------------------------------------
//	@brief	���ڳ�ʼ��
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
static void SLV1_Config(short baud)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_AHB1PeriphClockCmd(SLV1_USART_RX_GPIO_CLK | SLV1_USART_TX_GPIO_CLK, ENABLE);

	/* ʹ�� USART ʱ�� */
	RCC_APB2PeriphClockCmd(SLV1_USART_CLK, ENABLE);

	/* GPIO��ʼ�� */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	/* ����Tx����Ϊ���ù���  */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = SLV1_USART_TX_PIN;
	GPIO_Init(SLV1_USART_TX_GPIO_PORT, &GPIO_InitStructure);

	/* ����Rx����Ϊ���ù��� */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = SLV1_USART_RX_PIN;
	GPIO_Init(SLV1_USART_RX_GPIO_PORT, &GPIO_InitStructure);

	/* ���� PXx �� USARTx_Tx*/
	GPIO_PinAFConfig(SLV1_USART_RX_GPIO_PORT, SLV1_USART_RX_SOURCE, SLV1_USART_RX_AF);

	/*  ���� PXx �� USARTx__Rx*/
	GPIO_PinAFConfig(SLV1_USART_TX_GPIO_PORT, SLV1_USART_TX_SOURCE, SLV1_USART_TX_AF);

	/* ���ô�SLV1_USART ģʽ */
	/* ���������ã�SLV1_USART_BAUDRATE */
	USART_InitStructure.USART_BaudRate = baud * 100;
	/* �ֳ�(����λ+У��λ)��8 */
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	/* ֹͣλ��1��ֹͣλ */
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	/* У��λѡ�񣺲�ʹ��У�� */
	USART_InitStructure.USART_Parity = USART_Parity_No;
	/* Ӳ�������ƣ���ʹ��Ӳ���� */
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	/* USARTģʽ���ƣ�ͬʱʹ�ܽ��պͷ��� */
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	/* ���USART��ʼ������ */
	USART_Init(USART_SLV1, &USART_InitStructure);

	/* Ƕ�������жϿ�����NVIC���� */
	SLV1_NVIC_Configuration();

	/* ʹ�ܴ��ڽ����ж� */
	USART_ITConfig(USART_SLV1, USART_IT_RXNE, ENABLE);

	/* ʹ�ܴ��� */
	USART_Cmd(USART_SLV1, ENABLE);
}

//-------------------------------------------------------------------------------
//	@brief	Э��ջ��ʼ��
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void SLV1_init(void)
{
	char buf[100] ;
	int tmp;

	ModbusSvr_block_init(&Blk_SLV1);
	
	tmp = Blk_SLV1.baudrate * 100;

	SLV1_NVIC_Configuration();
	SLV1_Config(tmp);

	sprintf(buf, " Program Initialize... Adr:%d, Baud:%d", Blk_SLV1.station, tmp);
	//Usart_SendString(USART_SLV1, buf);
}

/*-------------------------------------------------------------------------------
	@brief:		Э���������
	@param:		None
	@retval:	None
-------------------------------------------------------------------------------*/
void SLV1_task(void)
{
	ModbusSvr_task(&Blk_SLV1, USART_SLV1);
}

//-------------------------------------------------------------------------------
//	@brief	modbus recieve counter
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void SLV1_Timer(void)
{
	Blk_SLV1.nMBInterval++;
}

//-------------------------------------------------------------------------------
//	@brief	�����жϷ������
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void SLV1_USART_IRQHandler(void)
{
	u8 ch;

	if (USART_GetITStatus(USART_SLV1, USART_IT_RXNE) != RESET) //�ж϶��Ĵ����Ƿ�ǿ�
	{
		ch = USART_ReceiveData(USART_SLV1); //�����Ĵ��������ݻ��浽���ջ�������
		ModbusSvr_isr(&Blk_SLV1, ch);
	}

	if (USART_GetITStatus(USART_SLV1, USART_IT_TXE) != RESET)
	{
		USART_ITConfig(USART_SLV1, USART_IT_TXE, DISABLE);
	}
}
//-----------------end of file---------------------------------------------
