#include "vl53lxx_i2c.h"
#include "delay.h"
#include "stdbool.h"	

/********************************************************************************	 


static void vl53IICStart(void);			//����iic��ʼ�ź�
static void vl53IICStop(void);	  		//����iicֹͣ�ź�
static void vl53IICAck(void);			//iic����ACK�ź�
static void vl53IICNAck(void);			//iic������ACK�ź� 
static u8 vl53IICWaitAck(void);			//iic�ȴ�ACK�ź�
static void vl53IICSendByte(u8 txd);	//iic����һ���ֽ�
static u8 vl53IICReceiveByte(u8 ack);	//iic��ȡһ���ֽ�

//��ʼ��iic
void vl53IICInit(void)
{	
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/*ʹ��VL53ʱ��*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	/*SCL PB5   SDA PB4*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_SetBits(GPIOB,GPIO_Pin_5);//PB5�����
	GPIO_SetBits(GPIOB,GPIO_Pin_4);//PB4�����
}
//����VL53��ʼ�ź�
static void vl53IICStart(void)
{
	SDA_OUT();     //sda�����
	VL53_SDA=1;	  	  
	VL53_SCL=1;
	delay_us(4);
 	VL53_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	VL53_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����VL53ֹͣ�ź�
static void vl53IICStop(void)
{
	SDA_OUT();//sda�����
	VL53_SCL=0;
	VL53_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	VL53_SCL=1; 
	VL53_SDA=1;//����I2C���߽����ź�
	delay_us(4);							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
static u8 vl53IICWaitAck(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
	VL53_SDA=1;delay_us(1);	   
	VL53_SCL=1;delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			vl53IICStop();
			return 1;
		}
	}
	VL53_SCL=0;//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
static void vl53IICAck(void)
{
	VL53_SCL=0;
	SDA_OUT();
	VL53_SDA=0;
	delay_us(1);
	VL53_SCL=1;
	delay_us(1);
	VL53_SCL=0;
}
//������ACKӦ��		    
static void vl53IICNAck(void)
{
	VL53_SCL=0;
	SDA_OUT();
	VL53_SDA=1;
	delay_us(1);
	VL53_SCL=1;
	delay_us(1);
	VL53_SCL=0;
}					 				     
//VL53����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
static void vl53IICSendByte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    VL53_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        VL53_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(1);   
		VL53_SCL=1;
		delay_us(1); 
		VL53_SCL=0;	
		delay_us(1);
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
static u8 vl53IICReceiveByte(u8 ack)
{
	u8 i,receive=0;
	SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        VL53_SCL=0; 
        delay_us(1);
		VL53_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		delay_us(1); 
    }					 
    if (!ack)
        vl53IICNAck();//����nACK
    else
        vl53IICAck(); //����ACK   
    return receive;
}

//��ָ����ַ����һ������
//ReadAddr:��ʼ�����ĵ�ַ  
//����ֵ  :����������
u8 vl53IICReadByte(u8 devaddr,u8 addr, u8* data)
{				  
	u8 temp=0;		  	    																 
	vl53IICStart();  
	vl53IICSendByte(devaddr);//��������д���� 	   
	vl53IICWaitAck(); 
	vl53IICSendByte(addr);   //���͵͵�ַ
	vl53IICWaitAck();	

	vl53IICStart();  	 	   
	vl53IICSendByte(devaddr|1);//��������������			   
	vl53IICWaitAck();	 
	temp=vl53IICReceiveByte(0);			   
	vl53IICStop();//����һ��ֹͣ����	 
	*data = temp;
	return temp;
}
//����������ֽ�
//addr:��ʼ��ַ
//rbuf:�����ݻ���
//len:���ݳ���
void vl53IICRead(u8 devaddr,u8 addr,u8 len,u8 *rbuf)
{
	int i=0;
	vl53IICStart();  
	vl53IICSendByte(devaddr);  
	vl53IICWaitAck();	
	vl53IICSendByte(addr);//��ַ����  
	vl53IICWaitAck();	

	vl53IICStart();  	
	vl53IICSendByte(devaddr|1);  	
	vl53IICWaitAck();	
	for(i=0; i<len; i++)
	{
		if(i==len-1)
		{
			rbuf[i] = vl53IICReceiveByte(0);//���һ���ֽڲ�Ӧ��
		}
		else
			rbuf[i] = vl53IICReceiveByte(1);
	}
	vl53IICStop( );	
}
//����������ֽ�
//addr:��ʼ��ַ
//rbuf:�����ݻ���
//len:���ݳ���
void vl53l1Read(u8 devaddr,u16 addr,u8 len,u8 *rbuf)
{
	int i=0;
	vl53IICStart();  
	vl53IICSendByte(devaddr);  
	vl53IICWaitAck();	
	vl53IICSendByte(addr>>8);  //��ַ��λ
	vl53IICWaitAck();
	vl53IICSendByte(addr&0x00FF);  //��ַ��λ
	vl53IICWaitAck();	

	vl53IICStart();  	
	vl53IICSendByte(devaddr|1);  	
	vl53IICWaitAck();	
	for(i=0; i<len; i++)
	{
		if(i==len-1)
		{
			rbuf[i] = vl53IICReceiveByte(0);//���һ���ֽڲ�Ӧ��
		}
		else
			rbuf[i] = vl53IICReceiveByte(1);
	}
	vl53IICStop( );	
}
//��ָ����ַд��һ������
//WriteAddr :д�����ݵ�Ŀ�ĵ�ַ    
//DataToWrite:Ҫд�������
void vl53IICWriteByte(u8 devaddr,u8 addr,u8 data)
{				   	  	    																 
	vl53IICStart();  
	vl53IICSendByte(devaddr); //��������д���� 	 
	vl53IICWaitAck();	   
	vl53IICSendByte(addr);   //���͵͵�ַ
	vl53IICWaitAck(); 	 										  		   
	vl53IICSendByte(data); //�����ֽ�							   
	vl53IICWaitAck();  		    	   
	vl53IICStop();		//����һ��ֹͣ���� 	 
}

//����д����ֽ�
//addr:��ʼ��ַ
//wbuf:д���ݻ���
//len:���ݵĳ���
void vl53IICWrite(u8 devaddr,u8 addr,u8 len,u8 *wbuf)
{
	int i=0;
	vl53IICStart();  
	vl53IICSendByte(devaddr);  	
	vl53IICWaitAck();	
	vl53IICSendByte(addr);  //��ַ����
	vl53IICWaitAck();	
	for(i=0; i<len; i++)
	{
		vl53IICSendByte(wbuf[i]);  
		vl53IICWaitAck();		
	}
	vl53IICStop( );	
}
//����д����ֽ�
//addr:��ʼ��ַ
//wbuf:д���ݻ���
//len:���ݵĳ���
void vl53l1Write(u8 devaddr,u16 addr,u8 len,u8 *wbuf)
{
	int i=0;
	vl53IICStart();  
	vl53IICSendByte(devaddr);  	
	vl53IICWaitAck();	
	vl53IICSendByte(addr>>8);  //��ַ��λ
	vl53IICWaitAck();
	vl53IICSendByte(addr&0x00FF);  //��ַ��λ
	vl53IICWaitAck();	
	for(i=0; i<len; i++)
	{
		vl53IICSendByte(wbuf[i]);  
		vl53IICWaitAck();		
	}
	vl53IICStop( );	
}
//iic д��ĳ��λ
bool vl53IICWriteBit(u8 devaddr,u8 addr, u8 bitNum, u8 data)
{
    u8 byte;
    vl53IICReadByte(devaddr, addr, &byte);
    byte = (data != 0) ? (byte | (1 << bitNum)) : (byte & ~(1 << bitNum));
    vl53IICWriteByte(devaddr, addr, byte);
	return true;
}







