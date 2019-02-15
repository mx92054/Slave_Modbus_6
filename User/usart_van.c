#include "usart_van.h"
#include "Modbus_svr.h"

#define VAN_STATION 1   // VANE������վ��ַ
#define VAN_START_ADR 0 // VANE�����������׵�ַ
#define VAN_LENGTH 40   // VANE��������������
#define VAN_SAVE_ADR 40 // VANE������������wReg�е���ʼ��ַ

extern u16 wReg[];
extern u8 bChanged;

uint8_t VAN_frame[8] = {VAN_STATION, 0x03, 0x00, VAN_START_ADR, 0x00, VAN_LENGTH, 0x00, 0x00};
u8 VAN_buffer[256];
u8 VAN_curptr;
u8 VAN_bRecv;
u8 VAN_frame_len = 85;

u8 bVanTxCmd = 0;

//-------------------------------------------------------------------------------
//	@brief	�жϳ�ʼ��
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
static void VAN_NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Ƕ�������жϿ�������ѡ�� */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	/* ����USARTΪ�ж�Դ */
	NVIC_InitStructure.NVIC_IRQChannel = VAN_USART_IRQ;
	/* �������ȼ�Ϊ1 */
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
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
static void VAN_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_AHB1PeriphClockCmd(VAN_USART_RX_GPIO_CLK | VAN_USART_TX_GPIO_CLK, ENABLE);

	/* ʹ�� USART ʱ�� */
	VAN_USART_APBxClkCmd(VAN_USART_CLK, ENABLE);

	/* GPIO��ʼ�� */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	/* ����Tx����Ϊ���ù���  */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = VAN_USART_TX_PIN;
	GPIO_Init(VAN_USART_TX_GPIO_PORT, &GPIO_InitStructure);

	/* ����Rx����Ϊ���ù��� */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = VAN_USART_RX_PIN;
	GPIO_Init(VAN_USART_RX_GPIO_PORT, &GPIO_InitStructure);

	/* ���� PXx �� USARTx_Tx*/
	GPIO_PinAFConfig(VAN_USART_RX_GPIO_PORT, VAN_USART_RX_SOURCE, VAN_USART_RX_AF);

	/*  ���� PXx �� USARTx__Rx*/
	GPIO_PinAFConfig(VAN_USART_TX_GPIO_PORT, VAN_USART_TX_SOURCE, VAN_USART_TX_AF);

	/* ���ô�VAN_USART ģʽ */
	/* ���������ã�VAN_USART_BAUDRATE */
	USART_InitStructure.USART_BaudRate = VAN_USART_BAUDRATE;
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
	USART_Init(USART_VAN, &USART_InitStructure);

	/* Ƕ�������жϿ�����NVIC���� */
	VAN_NVIC_Configuration();

	/* ʹ�ܴ��ڽ����ж� */
	USART_ITConfig(USART_VAN, USART_IT_RXNE, ENABLE);

	/* ʹ�ܴ��� */
	USART_Cmd(USART_VAN, ENABLE);
}

//-------------------------------------------------------------------------------
//	@brief	CPTͨ�ų�ʼ������
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void VAN_Init(void)
{
	u16 uCRC;

	VAN_Config();
	uCRC = CRC16(VAN_frame, 6);
	VAN_frame[6] = uCRC & 0x00FF;
	VAN_frame[7] = (uCRC & 0xFF00) >> 8;

	VAN_curptr = 0;
	VAN_bRecv = 0;
	wReg[97] = 0;
}

//-------------------------------------------------------------------------------
//	@brief	��������֡
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void VAN_TxCmd(void)
{
	if (bVanTxCmd == 1) //��ǰ�ڽ���д������
		return;

	if (VAN_bRecv == 1) //�����ǰδ��ɽ��գ���ͨ�Ŵ������������
		wReg[97]++;

	VAN_curptr = 0;
	VAN_bRecv = 1;
	Usart_SendBytes(USART_VAN, VAN_frame, 8);
	VAN_frame_len = 2 * VAN_LENGTH + 5;
}

/**
	*@brief  send data to vane controller
	*@param
	*@retval
*/
void VAN_TransData(void)
{
	int i;
	u16 uCRC;
	u8 buffer[256];

	if (VAN_bRecv == 1) //��ǰ���ڽ��ж�ȡ����
		return;

	if (bVanTxCmd == 1) //�����ǰδ��ɽ��գ���ͨ�Ŵ������������
		wReg[97]++;

	buffer[0] = VAN_STATION;
	buffer[1] = 0x10;
	buffer[2] = 0x03;
	buffer[3] = 0xE8;
	buffer[4] = 00;
	buffer[5] = 20;
	buffer[6] = 40;
	for (i = 0; i < 20; i++)
	{
		buffer[7 + 2 * i] = (u8)(wReg[100 + i] >> 8);
		buffer[8 + 2 * i] = (u8)(wReg[100 + i] & 0x00FF);
	}

	uCRC = CRC16(buffer, 47);
	buffer[47] = uCRC & 0x00FF;
	buffer[48] = (uCRC & 0xFF00) >> 8;
	Usart_SendBytes(USART_VAN, buffer, 49);
	VAN_frame_len = 7;
	VAN_curptr = 0;
	bVanTxCmd = 1;
}

//-------------------------------------------------------------------------------
//	@brief	�������ݴ���
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void VAN_Task(void)
{
	int i;

	if (VAN_curptr < VAN_frame_len)
		return;

	if (VAN_bRecv)
	{
		if (VAN_buffer[0] != VAN_STATION || VAN_buffer[1] != 0x03)
			return;

		if (VAN_buffer[2] != 2 * VAN_LENGTH)
			return;

		for (i = 0; i < VAN_LENGTH; i++)
			wReg[VAN_SAVE_ADR + i] = VAN_buffer[2 * i + 3] << 0x08 | VAN_buffer[2 * i + 4];

		wReg[154]++;
		VAN_bRecv = 0;
		VAN_curptr = 0;
	}

	if (bVanTxCmd)
	{
		if (VAN_buffer[0] != VAN_STATION || VAN_buffer[1] != 0x10)
			return;

		bVanTxCmd = 0;
		bChanged = 0;
		VAN_curptr = 0;
	}
}

//-------------------------------------------------------------------------------
//	@brief	�����жϷ������
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void VAN_USART_IRQHandler(void)
{
	u8 ch;

	if (USART_GetITStatus(USART_VAN, USART_IT_RXNE) != RESET) //�ж϶��Ĵ����Ƿ�ǿ�
	{
		ch = USART_ReceiveData(USART_VAN); //�����Ĵ��������ݻ��浽���ջ�������
		VAN_buffer[VAN_curptr++] = ch;
	}

	if (USART_GetITStatus(USART_VAN, USART_IT_TXE) != RESET)
	{
		USART_ITConfig(USART_VAN, USART_IT_TXE, DISABLE);
	}
}

//-----------------------------End of file--------------------------------------------------
