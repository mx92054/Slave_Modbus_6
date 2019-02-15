#ifndef __SPEED_USART__
#define __SPEED_USART__

#include "stm32f4xx.h"

//  COM1 Define

#define USART_SPD UART7

/* ��ͬ�Ĵ��ڹ��ص����߲�һ����ʱ��ʹ�ܺ���Ҳ��һ������ֲʱҪע�� 
 * ����1��6��      RCC_APB2PeriphClockCmd
 * ����2/3/4/5/7�� RCC_APB1PeriphClockCmd
 */
#define SPD_USART_CLK RCC_APB1Periph_UART7
#define SPD_USART_APBxClkCmd RCC_APB1PeriphClockCmd
#define SPD_USART_BAUDRATE 38400 //���ڲ�����

#define SPD_USART_RX_GPIO_PORT GPIOE
#define SPD_USART_RX_GPIO_CLK RCC_AHB1Periph_GPIOE
#define SPD_USART_RX_PIN GPIO_Pin_7
#define SPD_USART_RX_AF GPIO_AF_UART7
#define SPD_USART_RX_SOURCE GPIO_PinSource7

#define SPD_USART_TX_GPIO_PORT GPIOE
#define SPD_USART_TX_GPIO_CLK RCC_AHB1Periph_GPIOE
#define SPD_USART_TX_PIN GPIO_Pin_8
#define SPD_USART_TX_AF GPIO_AF_UART7
#define SPD_USART_TX_SOURCE GPIO_PinSource8

#define SPD_USART_IRQ UART7_IRQn
#define SPD_USART_IRQHandler UART7_IRQHandler

void SPD_Init(void);
void SPD_TxCmd(void);
void SPD_Task(void);

void SPD_USART_IRQHandler(void);

#endif

// --------------End of file------------------------
