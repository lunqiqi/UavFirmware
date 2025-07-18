#ifndef __UART1_H
#define __UART1_H
#include "sys.h"
#include <stdbool.h>

/********************************************************************************	 

********************************************************************************/


void uart1Init(u32 baudrate);	/*����1��ʼ��*/
bool uart1Test(void);			/*����1����*/
bool uart1GetDataWithTimout(u8 *c);	/*����ʽ����һ���ַ�*/		
void uart1SendData(uint32_t size, u8* data);	/*����ԭʼ����*/
int uart1Putchar(int ch);
void uart1Getchar(char * ch);

#endif /* __UART1_H */
