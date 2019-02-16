#ifndef __SPEED1_USART__
#define __SPEED1_USART__

#include "stm32f4xx.h"

//COM2 Define

#define USART_SPD1 USART3

/* 不同的串口挂载的总线不一样，时钟使能函数也不一样，移植时要注意 
 * 串口1和6是      RCC_APB2PeriphClockCmd
 * 串口2/3/4/5/7是 RCC_APB1PeriphClockCmd
 */
#define SPD1_USART_CLK RCC_APB1Periph_USART3
#define SPD1_USART_APBxClkCmd RCC_APB1PeriphClockCmd
#define SPD1_USART_BAUDRATE 38400 //串口波特率

#define SPD1_USART_RX_GPIO_PORT GPIOB
#define SPD1_USART_RX_GPIO_CLK RCC_AHB1Periph_GPIOB
#define SPD1_USART_RX_PIN GPIO_Pin_10
#define SPD1_USART_RX_AF GPIO_AF_USART3
#define SPD1_USART_RX_SOURCE GPIO_PinSource10

#define SPD1_USART_TX_GPIO_PORT GPIOB
#define SPD1_USART_TX_GPIO_CLK RCC_AHB1Periph_GPIOB
#define SPD1_USART_TX_PIN GPIO_Pin_11
#define SPD1_USART_TX_AF GPIO_AF_USART3
#define SPD1_USART_TX_SOURCE GPIO_PinSource11

#define SPD1_USART_IRQ USART3_IRQn
#define SPD1_USART_IRQHandler USART3_IRQHandler

void SPD1_Init(void);
void SPD1_TxCmd(void);
void SPD1_Task(void);

void SPD1_USART_IRQHandler(void);

#endif

// --------------End of file------------------------
