#include "usart_spd.h"
#include "Modbus_svr.h"

#define SPD_STATION 2   // SPDE������վ��ַ
#define SPD_START_ADR 0 // SPDE�����������׵�ַ
#define SPD_LENGTH 1    // SPDE��������������
#define SPD_SAVE_ADR 80 // SPDE������������wReg�е���ʼ��ַ

extern u16 wReg[];

uint8_t SPD_frame[8] = {SPD_STATION, 0x03, 0x00, SPD_START_ADR, 0x00, SPD_LENGTH, 0x00, 0x00};
u8 SPD_buffer[256];
u8 SPD_curptr;
u8 SPD_bRecv;
u8 SPD_frame_len = 85;

//-------------------------------------------------------------------------------
//	@brief	�жϳ�ʼ��
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
static void SPD_NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Ƕ�������жϿ�������ѡ�� */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  /* ����USARTΪ�ж�Դ */
  NVIC_InitStructure.NVIC_IRQChannel = SPD_USART_IRQ;
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
static void SPD_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;

  RCC_AHB1PeriphClockCmd(SPD_USART_RX_GPIO_CLK | SPD_USART_TX_GPIO_CLK, ENABLE);

  /* ʹ�� USART ʱ�� */
  SPD_USART_APBxClkCmd(SPD_USART_CLK, ENABLE);

  /* GPIO��ʼ�� */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  /* ����Tx����Ϊ���ù���  */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = SPD_USART_TX_PIN;
  GPIO_Init(SPD_USART_TX_GPIO_PORT, &GPIO_InitStructure);

  /* ����Rx����Ϊ���ù��� */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = SPD_USART_RX_PIN;
  GPIO_Init(SPD_USART_RX_GPIO_PORT, &GPIO_InitStructure);

  /* ���� PXx �� USARTx_Tx*/
  GPIO_PinAFConfig(SPD_USART_RX_GPIO_PORT, SPD_USART_RX_SOURCE, SPD_USART_RX_AF);

  /*  ���� PXx �� USARTx__Rx*/
  GPIO_PinAFConfig(SPD_USART_TX_GPIO_PORT, SPD_USART_TX_SOURCE, SPD_USART_TX_AF);

  /* ���ô�SPD_USART ģʽ */
  /* ���������ã�SPD_USART_BAUDRATE */
  USART_InitStructure.USART_BaudRate = SPD_USART_BAUDRATE;
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
  USART_Init(USART_SPD, &USART_InitStructure);

  /* Ƕ�������жϿ�����NVIC���� */
  SPD_NVIC_Configuration();

  /* ʹ�ܴ��ڽ����ж� */
  USART_ITConfig(USART_SPD, USART_IT_RXNE, ENABLE);

  /* ʹ�ܴ��� */
  USART_Cmd(USART_SPD, ENABLE);
}

//-------------------------------------------------------------------------------
//	@brief	CPTͨ�ų�ʼ������
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void SPD_Init(void)
{
  u16 uCRC;

  SPD_Config();
  uCRC = CRC16(SPD_frame, 6);
  SPD_frame[6] = uCRC & 0x00FF;
  SPD_frame[7] = (uCRC & 0xFF00) >> 8;

  SPD_curptr = 0;
  SPD_bRecv = 0;
  wReg[96] = 0;
  SPD_frame_len = 2 * SPD_LENGTH + 5;
}

//-------------------------------------------------------------------------------
//	@brief	��������֡
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void SPD_TxCmd(void)
{
  if (SPD_bRecv == 1) //�����ǰδ��ɽ��գ���ͨ�Ŵ������������
    wReg[96]++;

  SPD_curptr = 0;
  SPD_bRecv = 1;
  Usart_SendBytes(USART_SPD, SPD_frame, 8);
}

//-------------------------------------------------------------------------------
//	@brief	�������ݴ���
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void SPD_Task(void)
{
  int i;

  if (SPD_curptr < SPD_frame_len)
    return;

  if (SPD_buffer[0] != SPD_STATION || SPD_buffer[1] != 0x03)
    return;

  if (SPD_buffer[2] != 2 * SPD_LENGTH)
    return;

  wReg[81] = wReg[80];
  for (i = 0; i < SPD_LENGTH; i++)
    wReg[80] = SPD_buffer[2 * i + 3] << 0x08 | SPD_buffer[2 * i + 4];

  wReg[83] = wReg[82];
  wReg[82] = wReg[80] - wReg[81];
  if (wReg[80] < 1024 && wReg[81] > 3072)
  {
    wReg[82] = wReg[80] - wReg[81] + 4096;
    wReg[84]++;
  }
  if (wReg[80] > 3072 && wReg[81] < 1024)
  {
    wReg[82] = wReg[80] - wReg[81] - 4096;
    wReg[84]--;
  }

  wReg[153]++ ;
  SPD_bRecv = 0;
  SPD_curptr = 0;
}

//-------------------------------------------------------------------------------
//	@brief	�����жϷ������
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void SPD_USART_IRQHandler(void)
{
  u8 ch;

  if (USART_GetITStatus(USART_SPD, USART_IT_RXNE) != RESET) //�ж϶��Ĵ����Ƿ�ǿ�
  {
    ch = USART_ReceiveData(USART_SPD); //�����Ĵ��������ݻ��浽���ջ�������
    SPD_buffer[SPD_curptr++] = ch;
  }

  if (USART_GetITStatus(USART_SPD, USART_IT_TXE) != RESET)
  {
    USART_ITConfig(USART_SPD, USART_IT_TXE, DISABLE);
  }
}

//-----------------------------End of file--------------------------------------------------
