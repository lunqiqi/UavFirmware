#ifndef __UART_SYSLINK_H
#define __UART_SYSLINK_H
#include <stdbool.h>
#include "sys.h"

/********************************************************************************	 

********************************************************************************/

#define UARTSLK_TYPE             USART2	
#define UARTSLK_PERIF            RCC_APB1Periph_USART2
#define ENABLE_UARTSLK_RCC       RCC_APB1PeriphClockCmd
#define UARTSLK_IRQ              USART2_IRQn

#define UARTSLK_DMA_IRQ          DMA1_Stream6_IRQn	
#define UARTSLK_DMA_IT_TC        DMA_IT_TC
#define UARTSLK_DMA_STREAM       DMA1_Stream6
#define UARTSLK_DMA_CH           DMA_Channel_4
#define UARTSLK_DMA_IT_TCIF    	 DMA_IT_TCIF6

#define UARTSLK_GPIO_PERIF       RCC_AHB1Periph_GPIOA 
#define UARTSLK_GPIO_PORT        GPIOA
#define UARTSLK_GPIO_TX_PIN      GPIO_Pin_2
#define UARTSLK_GPIO_RX_PIN      GPIO_Pin_3
#define UARTSLK_GPIO_AF_TX_PIN   GPIO_PinSource2
#define UARTSLK_GPIO_AF_RX_PIN   GPIO_PinSource3
#define UARTSLK_GPIO_AF_TX       GPIO_AF_USART2
#define UARTSLK_GPIO_AF_RX       GPIO_AF_USART2

#define UARTSLK_TXEN_PERIF       RCC_AHB1Periph_GPIOA
#define UARTSLK_TXEN_PORT        GPIOA
#define UARTSLK_TXEN_PIN         GPIO_Pin_0
#define UARTSLK_TXEN_EXTI        EXTI_Line0


void uartslkInit(void);		/*���ڳ�ʼ��*/
bool uartslkTest(void);
bool uartslkGetDataWithTimout(u8 *c);	/*�ӽ��ն��ж�ȡ����(����ʱ����)*/
void uartslkSendData(u32 size, u8* data);	/*����ԭʼ����*/
void uartslkSendDataIsrBlocking(u32 size, u8* data);/*�жϷ�ʽ����ԭʼ����*/
int uartslkPutchar(int ch);		/*����һ���ַ�������*/
void uartslkSendDataDmaBlocking(u32 size, u8* data);/*ͨ��DMA����ԭʼ����*/
void uartslkIsr(void);		/*�����жϷ�����*/
void uartslkDmaIsr(void);	/*DMA�жϷ�����*/
void uartslkTxenFlowctrlIsr(void);

#endif /* __UART_SYSLINK_H */
