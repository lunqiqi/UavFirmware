#ifndef __VL53LXX_I2C_H
#define __VL53LXX_I2C_H
#include "sys.h" 
#include "stdbool.h"

/********************************************************************************	 

********************************************************************************/

/*IO��������*/
#define SDA_IN()  {GPIOB->MODER&=~(3<<(4*2));GPIOB->MODER|=0<<4*2;}	//PB4����ģʽ
#define SDA_OUT() {GPIOB->MODER&=~(3<<(4*2));GPIOB->MODER|=1<<4*2;} //PB4���ģʽ
/*IO��������*/	 
#define VL53_SCL    PBout(5) 	//SCL
#define VL53_SDA    PBout(4) 	//SDA	 
#define READ_SDA	PBin(4)  	//����SDA 


//VL53���в�������
void vl53IICInit(void);			/*��ʼ��VL53��IO��*/				 
u8 vl53IICReadByte(u8 devaddr,u8 addr, u8* data);		/*��һ�ֽ�*/
void vl53IICWriteByte(u8 devaddr,u8 addr,u8 data);		/*дһ�ֽ�*/
void vl53IICRead(u8 devaddr,u8 addr,u8 len,u8 *rbuf);	/*������ȡ����ֽ�*/
void vl53IICWrite(u8 devaddr,u8 addr,u8 len,u8 *wbuf);	/*����д�����ֽ�*/
bool vl53IICWriteBit(u8 devaddr,u8 addr, u8 bitNum, u8 data);	/*iic д��ĳ��λ*/

void vl53l1Read(u8 devaddr,u16 addr,u8 len,u8 *rbuf);	/*������ȡ����ֽ�*/
void vl53l1Write(u8 devaddr,u16 addr,u8 len,u8 *wbuf);	/*����д�����ֽ�*/
	
#endif 


