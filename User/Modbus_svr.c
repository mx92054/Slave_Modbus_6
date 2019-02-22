#include <string.h>
#include "Modbus_svr.h"
#include "stm32f4xx_conf.h"

Modbus_block mblock1;
//-------------------------------------------------------------------------------
//	@brief	�жϳ�ʼ��
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
static void MODBUS_NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Ƕ�������жϿ�������ѡ�� */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	/* ����USARTΪ�ж�Դ */
	NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USART_IRQ;
	/* �������ȼ�Ϊ1 */
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	/* �����ȼ�Ϊ1 */
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
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
static void MODBUS_Config(u32 baud)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_AHB1PeriphClockCmd(DEBUG_USART_RX_GPIO_CLK | DEBUG_USART_TX_GPIO_CLK, ENABLE);

	/* ʹ�� USART ʱ�� */
	RCC_APB2PeriphClockCmd(DEBUG_USART_CLK, ENABLE);

	/* GPIO��ʼ�� */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	/* ����Tx����Ϊ���ù���  */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_PIN;
	GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

	/* ����Rx����Ϊ���ù��� */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_PIN;
	GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);

	/* ���� PXx �� USARTx_Tx*/
	GPIO_PinAFConfig(DEBUG_USART_RX_GPIO_PORT, DEBUG_USART_RX_SOURCE, DEBUG_USART_RX_AF);

	/*  ���� PXx �� USARTx__Rx*/
	GPIO_PinAFConfig(DEBUG_USART_TX_GPIO_PORT, DEBUG_USART_TX_SOURCE, DEBUG_USART_TX_AF);

	/* ���ô�DEBUG_USART ģʽ */
	/* ���������ã�DEBUG_USART_BAUDRATE */
	USART_InitStructure.USART_BaudRate = DEBUG_USART_BAUDRATE;
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
	USART_Init(DEBUG_USARTx, &USART_InitStructure);

	/* Ƕ�������жϿ�����NVIC���� */
	MODBUS_NVIC_Configuration();

	/* ʹ�ܴ��ڽ����ж� */
	USART_ITConfig(DEBUG_USARTx, USART_IT_RXNE, ENABLE);

	/* ʹ�ܴ��� */
	USART_Cmd(DEBUG_USARTx, ENABLE);
}

//-------------------------------------------------------------------------------
//	@brief	Э��ջ��ʼ��
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void Modbus_init(void)
{
	char buf[100] ;
	int tmp;

	ModbusSvr_block_init(&mblock1);
	tmp = mblock1.baudrate * 100;

	MODBUS_NVIC_Configuration();
	MODBUS_Config(tmp);

	sprintf(buf, " Program Initialize... Adr:%d, Baud:%d", mblock1.station, tmp);
	Usart_SendString(DEBUG_USARTx, buf);
}

/*-------------------------------------------------------------------------------
	@brief:		Э���������
	@param:		None
	@retval:	None
-------------------------------------------------------------------------------*/
void Modbus_task(void)
{
	ModbusSvr_task(&mblock1, DEBUG_USARTx);
}

//-------------------------------------------------------------------------------
//	@brief	modbus recieve counter
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void ModbusTimer(void)
{
	mblock1.nMBInterval++;
}

//-------------------------------------------------------------------------------
//	@brief	�����жϷ������
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void DEBUG_USART_IRQHandler(void)
{
	u8 ch;

	if (USART_GetITStatus(DEBUG_USARTx, USART_IT_RXNE) != RESET) //�ж϶��Ĵ����Ƿ�ǿ�
	{
		ch = USART_ReceiveData(DEBUG_USARTx); //�����Ĵ��������ݻ��浽���ջ�������
		ModbusSvr_isr(&mblock1, ch);
	}

	if (USART_GetITStatus(DEBUG_USARTx, USART_IT_TXE) != RESET)
	{
		USART_ITConfig(DEBUG_USARTx, USART_IT_TXE, DISABLE);
	}
}
//-----------------end of file---------------------------------------------
