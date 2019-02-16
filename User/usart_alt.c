#include "usart_alt.h"
#include "gpio.h"
#include "Modbus_svr.h"

#define ALT_SAVE_ADR  85				// ALT������������wReg�е���ʼ��ַ

extern u16 wReg[] ;

char 		ALT_buffer[256] ;
u8			ALT_curptr ;
u8			ALT_bRecv ;
float		fAlimeter ;


//-------------------------------------------------------------------------------
//	@brief	�жϳ�ʼ��
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
static void ALT_NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Ƕ�������жϿ�������ѡ�� */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  /* ����USARTΪ�ж�Դ */
  NVIC_InitStructure.NVIC_IRQChannel = ALT_USART_IRQ;
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
static void ALT_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
		
  RCC_AHB1PeriphClockCmd(ALT_USART_RX_GPIO_CLK|ALT_USART_TX_GPIO_CLK,ENABLE);

  /* ʹ�� USART ʱ�� */
  ALT_USART_APBxClkCmd(ALT_USART_CLK, ENABLE);
  
  /* GPIO��ʼ�� */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
  /* ����Tx����Ϊ���ù���  */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = ALT_USART_TX_PIN  ;  
  GPIO_Init(ALT_USART_TX_GPIO_PORT, &GPIO_InitStructure);

  /* ����Rx����Ϊ���ù��� */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = ALT_USART_RX_PIN;
  GPIO_Init(ALT_USART_RX_GPIO_PORT, &GPIO_InitStructure);
  
 /* ���� PXx �� USARTx_Tx*/
  GPIO_PinAFConfig(ALT_USART_RX_GPIO_PORT,ALT_USART_RX_SOURCE,ALT_USART_RX_AF);

  /*  ���� PXx �� USARTx__Rx*/
  GPIO_PinAFConfig(ALT_USART_TX_GPIO_PORT,ALT_USART_TX_SOURCE,ALT_USART_TX_AF);
  
  /* ���ô�ALT_USART ģʽ */
  /* ���������ã�ALT_USART_BAUDRATE */
  USART_InitStructure.USART_BaudRate = ALT_USART_BAUDRATE;
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
  USART_Init(USART_ALT, &USART_InitStructure); 
	
  /* Ƕ�������жϿ�����NVIC���� */
	ALT_NVIC_Configuration();
  
	/* ʹ�ܴ��ڽ����ж� */
	USART_ITConfig(USART_ALT, USART_IT_RXNE, ENABLE);
	
  /* ʹ�ܴ��� */
  USART_Cmd(USART_ALT, ENABLE);		
}

//-------------------------------------------------------------------------------
//	@brief	CPTͨ�ų�ʼ������
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void ALT_Init(void)
{
		ALT_Config() ;
		
		ALT_curptr = 0 ;
		ALT_bRecv = 0 ;
		wReg[94] = 0 ;
}


//-------------------------------------------------------------------------------
//	@brief	��������֡
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void ALT_TxCmd(void)
{
	if ( ALT_bRecv == 1)		//�����ǰδ��ɽ��գ���ͨ�Ŵ������������
		wReg[94]++ ;
		
	ALT_curptr = 0 ;
	ALT_bRecv = 1 ;
	Usart_SendByte(USART_ALT, 'Z') ;	
}


//-------------------------------------------------------------------------------
//	@brief	�������ݴ���
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void ALT_Task(void)
{
	if ( ALT_bRecv == 1 )
		return ;
	
	if ( sscanf(ALT_buffer,"%f", &fAlimeter) > 0 )
	{
		wReg[ALT_SAVE_ADR] = (u16)(fAlimeter*100) ;
	}
}



//-------------------------------------------------------------------------------
//	@brief	�����жϷ������
//	@param	None
//	@retval	None
//-------------------------------------------------------------------------------
void ALT_USART_IRQHandler(void)
{	
	u8 ch ;
	
	if(USART_GetITStatus(USART_ALT, USART_IT_RXNE) != RESET)	   //�ж϶��Ĵ����Ƿ�ǿ�
	{	
		ch = USART_ReceiveData(USART_ALT);   //�����Ĵ��������ݻ��浽���ջ�������		
		if ( ALT_bRecv == 1)
		{
			ALT_buffer[ALT_curptr++] = ch ;
			if ( ch == 0x0D )
			{
				ALT_buffer[ALT_curptr++] = 0 ;
				ALT_bRecv = 0 ;
			}
		}
	}

	if(USART_GetITStatus(USART_ALT, USART_IT_TXE) != RESET)                     
	{ 
   	USART_ITConfig(USART_ALT, USART_IT_TXE, DISABLE);
	}	  
}

//-----------------------------End of file--------------------------------------------------
