#ifndef __DPT_USART__
#define __DPT_USART__

#include "stm32f4xx.h"

//  COM1 Define

#define USART_DPT                        			USART6

/* ��ͬ�Ĵ��ڹ��ص����߲�һ����ʱ��ʹ�ܺ���Ҳ��һ������ֲʱҪע�� 
 * ����1��6��      RCC_APB2PeriphClockCmd
 * ����2/3/4/5/7�� RCC_APB1PeriphClockCmd
 */
#define DPT_USART_CLK RCC_APB2Periph_USART6
#define DPT_USART_APBxClkCmd RCC_APB2PeriphClockCmd
#define DPT_USART_BAUDRATE 9600 //���ڲ�����

#define DPT_USART_RX_GPIO_PORT GPIOC
#define DPT_USART_RX_GPIO_CLK RCC_AHB1Periph_GPIOC
#define DPT_USART_RX_PIN GPIO_Pin_6
#define DPT_USART_RX_AF GPIO_AF_USART6
#define DPT_USART_RX_SOURCE GPIO_PinSource6

#define DPT_USART_TX_GPIO_PORT GPIOC
#define DPT_USART_TX_GPIO_CLK RCC_AHB1Periph_GPIOC
#define DPT_USART_TX_PIN GPIO_Pin_7
#define DPT_USART_TX_AF GPIO_AF_USART6
#define DPT_USART_TX_SOURCE GPIO_PinSource7

#define DPT_USART_IRQ USART6_IRQn
#define DPT_USART_IRQHandler USART6_IRQHandler


void 	DPT_Init(void) ;
void	DPT_Task(void) ;

void 	DPT_USART_IRQHandler(void) ;

#endif

// --------------End of file------------------------