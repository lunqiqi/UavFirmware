#ifndef __IIC1_H
#define __IIC1_H
#include "sys.h" 
#include "stdbool.h"

/********************************************************************************	 

********************************************************************************/

/*IO��������*/
#define SDA_IN()  {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=0<<9*2;}	//PB9����ģʽ
#define SDA_OUT() {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=1<<9*2;} //PB9���ģʽ
/*IO��������*/	 
#define IIC1_SCL    PBout(8) 	//SCL
#define IIC1_SDA    PBout(9) 	//SDA	 
#define READ_SDA	PBin(9)  	//����SDA 


//IIC1���в�������
void iicDevInit(void);					/*��ʼ��IIC1��IO��*/				 
//u8 iicDevReadByte(u8 devaddr,u8 addr);	/*��һ�ֽ�*/
u8 iicDevReadByte(u8 devaddr,u8 addr, u8* data);	/*��һ�ֽ�*/
void iicDevWriteByte(u8 devaddr,u8 addr,u8 data);	/*дһ�ֽ�*/
void iicDevRead(u8 devaddr,u8 addr,u8 len,u8 *rbuf);/*������ȡ����ֽ�*/
void iicDevWrite(u8 devaddr,u8 addr,u8 len,u8 *wbuf);/*����д�����ֽ�*/
bool iicDevWriteBit(u8 devaddr,u8 addr, u8 bitNum, u8 data);	/*iic д��ĳ��λ*/

#endif
















