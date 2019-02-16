#ifndef __SPEED3_USART__
#define __SPEED3_USART__

#include "stm32f4xx.h"

//COM4 Define

#define USART_SPD3 USART3

/* 不同的串口挂载的总线不一样，时钟使能函数也不一样，移植时要注意 
 * 串口1和6是      RCC_APB2PeriphClockCmd
 * 串口2/3/4/5/7是 RCC_APB1PeriphClockCmd
 */
#define SPD3_USART_CLK RCC_APB2Periph_USART6
#define SPD3_USART_APBxClkCmd RCC_APB2PeriphClockCmd
#define SPD3_USART_BAUDRATE 115200 //串口波特率

#define SPD3_USART_RX_GPIO_PORT GPIOC
#define SPD3_USART_RX_GPIO_CLK RCC_AHB1Periph_GPIOC
#define SPD3_USART_RX_PIN GPIO_Pin_6
#define SPD3_USART_RX_AF GPIO_AF_USART6
#define SPD3_USART_RX_SOURCE GPIO_PinSource6

#define SPD3_USART_TX_GPIO_PORT GPIOC
#define SPD3_USART_TX_GPIO_CLK RCC_AHB1Periph_GPIOC
#define SPD3_USART_TX_PIN GPIO_Pin_7
#define SPD3_USART_TX_AF GPIO_AF_USART6
#define SPD3_USART_TX_SOURCE GPIO_PinSource7

#define SPD3_USART_IRQ USART6_IRQn
#define SPD3_USART_IRQHandler USART6_IRQHandler

void SPD3_Init(void);
void SPD3_TxCmd(void);
void SPD3_Task(void);

void SPD3_USART_IRQHandler(void);

#endif

// --------------End of file------------------------
