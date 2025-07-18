#ifndef __LED_H
#define __LED_H
#include "sys.h"
#include <stdbool.h>

/********************************************************************************	 

********************************************************************************/

#define LED_NUM 5

typedef enum 
{
	LED_BLUE_L = 0, 
	LED_GREEN_L, 
	LED_RED_L, 
	LED_GREEN_R, 
	LED_RED_R,
} led_e;

#define DATA_RX_LED		LED_GREEN_L	/*�������ݽ���ָʾ��*/
#define DATA_TX_LED		LED_RED_L	/*�������ݷ���ָʾ��*/
#define CHG_LED 		LED_BLUE_L	/*���ָʾ��*/
#define LOWBAT_LED		LED_RED_R	/*��ص͵���ָʾ��*/
#define SYS_LED   		LED_GREEN_R	/*ϵͳ����ָʾ��*/
#define ERR_LED1       	LED_RED_L	/*����ָʾ��1*/
#define ERR_LED2       	LED_RED_R	/*����ָʾ��2*/


void ledInit(void);		/* LED��ʼ�� */
bool ledTest(void);		/* LED���� */
void ledSetAll(void);	/* ��������LED */
void ledClearAll(void);	/* �ر�����LED */
void ledSet(led_e led, bool value);	/* ����ĳ��LED��״̬ */
void ledFlashOne(led_e led, u32 onTime, u32 offTime);	/*LED��˸1��*/


#endif
