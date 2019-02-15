#include "usart_cpt.h"
#include "Modbus_svr.h"

#define CPT_STATION 1   // CPT������վ��ַ
#define CPT_START_ADR 0 // CPT�����������׵�ַ
#define CPT_LENGTH 40   // CPT��������������
#define CPT_SAVE_ADR 0  // CPT������������wReg�е���ʼ��ַ

extern u16 wReg[];

uint8_t cpt_frame[8] = {CPT_STATION, 0x03, 0x00, CPT_START_ADR, 0x00, CPT_LENGTH, 0x00, 0x00};
u8 cpt_buffer[256];
u8 cpt_curptr;
u8 cpt_bRecv;
u8 cpt_frame_len = 85;

//-------------------------------------------------------------------------------
//	@brief	�жϳ�ʼ��
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
static void CPT_NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Ƕ�������жϿ�������ѡ�� */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  /* ����USARTΪ�ж�Դ */
  NVIC_InitStructure.NVIC_IRQChannel = CPT_USART_IRQ;
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
static void CPT_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;

  RCC_AHB1PeriphClockCmd(CPT_USART_RX_GPIO_CLK | CPT_USART_TX_GPIO_CLK, ENABLE);

  /* ʹ�� USART ʱ�� */
  CPT_USART_APBxClkCmd(CPT_USART_CLK, ENABLE);

  /* GPIO��ʼ�� */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  /* ����Tx����Ϊ���ù���  */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = CPT_USART_TX_PIN;
  GPIO_Init(CPT_USART_TX_GPIO_PORT, &GPIO_InitStructure);

  /* ����Rx����Ϊ���ù��� */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = CPT_USART_RX_PIN;
  GPIO_Init(CPT_USART_RX_GPIO_PORT, &GPIO_InitStructure);

  /* ���� PXx �� USARTx_Tx*/
  GPIO_PinAFConfig(CPT_USART_RX_GPIO_PORT, CPT_USART_RX_SOURCE, CPT_USART_RX_AF);

  /*  ���� PXx �� USARTx__Rx*/
  GPIO_PinAFConfig(CPT_USART_TX_GPIO_PORT, CPT_USART_TX_SOURCE, CPT_USART_TX_AF);

  /* ���ô�CPT_USART ģʽ */
  /* ���������ã�CPT_USART_BAUDRATE */
  USART_InitStructure.USART_BaudRate = CPT_USART_BAUDRATE;
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
  USART_Init(USART_CPT, &USART_InitStructure);

  /* Ƕ�������жϿ�����NVIC���� */
  CPT_NVIC_Configuration();

  /* ʹ�ܴ��ڽ����ж� */
  USART_ITConfig(USART_CPT, USART_IT_RXNE, ENABLE);

  /* ʹ�ܴ��� */
  USART_Cmd(USART_CPT, ENABLE);
}

//-------------------------------------------------------------------------------
//	@brief	CPTͨ�ų�ʼ������
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void CPT_Init(void)
{
  u16 uCRC;

  CPT_Config();
  uCRC = CRC16(cpt_frame, 6);
  cpt_frame[6] = uCRC & 0x00FF;
  cpt_frame[7] = (uCRC & 0xFF00) >> 8;

  cpt_curptr = 0;
  cpt_bRecv = 0;
  wReg[98] = 0;
  cpt_frame_len = 2 * CPT_LENGTH + 5;
}

//-------------------------------------------------------------------------------
//	@brief	��������֡
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void CPT_TxCmd(void)
{
  if (cpt_bRecv == 1) //�����ǰδ��ɽ��գ���ͨ�Ŵ������������
    wReg[98]++;

  cpt_curptr = 0;
  cpt_bRecv = 1;
  Usart_SendBytes(USART_CPT, cpt_frame, 8);
}

//-------------------------------------------------------------------------------
//	@brief	�������ݴ���
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void CPT_Task(void)
{
  int i;

  if (cpt_curptr < cpt_frame_len)
    return;

  if (cpt_buffer[0] != CPT_STATION || cpt_buffer[1] != 0x03)
    return;

  if (cpt_buffer[2] != 2 * CPT_LENGTH)
    return;

  for (i = 0; i < CPT_LENGTH; i++)
    wReg[CPT_SAVE_ADR + i] = cpt_buffer[2 * i + 3] << 0x08 | cpt_buffer[2 * i + 4];

  wReg[155]++;
  cpt_bRecv = 0;
  cpt_curptr = 0;
}

//-------------------------------------------------------------------------------
//	@brief	�����жϷ������
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void CPT_USART_IRQHandler(void)
{
  u8 ch;

  if (USART_GetITStatus(USART_CPT, USART_IT_RXNE) != RESET) //�ж϶��Ĵ����Ƿ�ǿ�
  {
    ch = USART_ReceiveData(USART_CPT); //�����Ĵ��������ݻ��浽���ջ�������
    cpt_buffer[cpt_curptr++] = ch;
  }

  if (USART_GetITStatus(USART_CPT, USART_IT_TXE) != RESET)
  {
    USART_ITConfig(USART_CPT, USART_IT_TXE, DISABLE);
  }
}

//-----------------------------End of file--------------------------------------------------
