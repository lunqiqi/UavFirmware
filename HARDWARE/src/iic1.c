#include "iic1.h"
#include "delay.h"
#include "stdbool.h"	




static bool isInit;

static void iicDevStart(void);			//����iic��ʼ�ź�
static void iicDevStop(void);	  		//����iicֹͣ�ź�
static void iicDevAck(void);			//iic����ACK�ź�
static void iicDevNAck(void);			//iic������ACK�ź� 
static u8 iicDevWaitAck(void);			//iic�ȴ�ACK�ź�
static void iicDevSendByte(u8 txd);		//iic����һ���ֽ�
static u8 iicDevReceiveByte(u8 ack);	//iic��ȡһ���ֽ�

//��ʼ��iic
void iicDevInit(void)
{	
	if(isInit)	return;
	
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/*ʹ��IIC1ʱ��*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	/*SCL PB6   SDA PB7*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOB, &GPIO_InitStructure);	

	IIC1_SCL=1;
	IIC1_SDA=1;

	isInit=true;
}
//����IIC1��ʼ�ź�
static void iicDevStart(void)
{
	SDA_OUT();     //sda�����
	IIC1_SDA=1;	  	  
	IIC1_SCL=1;
	delay_us(4);
 	IIC1_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC1_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IIC1ֹͣ�ź�
static void iicDevStop(void)
{
	SDA_OUT();//sda�����
	IIC1_SCL=0;
	IIC1_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC1_SCL=1; 
	IIC1_SDA=1;//����I2C���߽����ź�
	delay_us(4);							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
static u8 iicDevWaitAck(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
	IIC1_SDA=1;delay_us(1);	   
	IIC1_SCL=1;delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			iicDevStop();
			return 1;
		}
	}
	IIC1_SCL=0;//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
static void iicDevAck(void)
{
	IIC1_SCL=0;
	SDA_OUT();
	IIC1_SDA=0;
	delay_us(1);
	IIC1_SCL=1;
	delay_us(1);
	IIC1_SCL=0;
}
//������ACKӦ��		    
static void iicDevNAck(void)
{
	IIC1_SCL=0;
	SDA_OUT();
	IIC1_SDA=1;
	delay_us(1);
	IIC1_SCL=1;
	delay_us(1);
	IIC1_SCL=0;
}					 				     
//IIC1����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
static void iicDevSendByte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC1_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        IIC1_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(1);   
		IIC1_SCL=1;
		delay_us(1); 
		IIC1_SCL=0;	
		delay_us(1);
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
static u8 iicDevReceiveByte(u8 ack)
{
	u8 i,receive=0;
	SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        IIC1_SCL=0; 
        delay_us(1);
		IIC1_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		delay_us(1); 
    }					 
    if (!ack)
        iicDevNAck();//����nACK
    else
        iicDevAck(); //����ACK   
    return receive;
}

//��ָ����ַ����һ������
//ReadAddr:��ʼ�����ĵ�ַ  
//����ֵ  :����������
u8 iicDevReadByte(u8 devaddr,u8 addr, u8* data)
{				  
	u8 temp=0;		  	    																 
	iicDevStart();  
	iicDevSendByte(devaddr);//��������д���� 	   
	iicDevWaitAck(); 
	iicDevSendByte(addr);   //���͵͵�ַ
	iicDevWaitAck();	

	iicDevStart();  	 	   
	iicDevSendByte(devaddr|1);//��������������			   
	iicDevWaitAck();	 
	temp=iicDevReceiveByte(0);			   
	iicDevStop();//����һ��ֹͣ����	 
	*data = temp;
	return temp;
}
//����������ֽ�
//addr:��ʼ��ַ
//rbuf:�����ݻ���
//len:���ݳ���
void iicDevRead(u8 devaddr,u8 addr,u8 len,u8 *rbuf)
{
	int i=0;
	iicDevStart();  
	iicDevSendByte(devaddr);  
	iicDevWaitAck();	
	iicDevSendByte(addr);//��ַ����  
	iicDevWaitAck();	

	iicDevStart();  	
	iicDevSendByte(devaddr|1);  	
	iicDevWaitAck();	
	for(i=0; i<len; i++)
	{
		if(i==len-1)
		{
			rbuf[i] = iicDevReceiveByte(0);//���һ���ֽڲ�Ӧ��
		}
		else
			rbuf[i] = iicDevReceiveByte(1);
	}
	iicDevStop( );	
}
//��ָ����ַд��һ������
//WriteAddr :д�����ݵ�Ŀ�ĵ�ַ    
//DataToWrite:Ҫд�������
void iicDevWriteByte(u8 devaddr,u8 addr,u8 data)
{				   	  	    																 
	iicDevStart();  
	iicDevSendByte(devaddr); //��������д���� 	 
	iicDevWaitAck();	   
	iicDevSendByte(addr);   //���͵͵�ַ
	iicDevWaitAck(); 	 										  		   
	iicDevSendByte(data); //�����ֽ�							   
	iicDevWaitAck();  		    	   
	iicDevStop();		//����һ��ֹͣ���� 	 
}

//����д����ֽ�
//addr:��ʼ��ַ
//wbuf:д���ݻ���
//len:���ݵĳ���
void iicDevWrite(u8 devaddr,u8 addr,u8 len,u8 *wbuf)
{
	int i=0;
	iicDevStart();  
	iicDevSendByte(devaddr);  	
	iicDevWaitAck();	
	iicDevSendByte(addr);  //��ַ����
	iicDevWaitAck();	
	for(i=0; i<len; i++)
	{
		iicDevSendByte(wbuf[i]);  
		iicDevWaitAck();		
	}
	iicDevStop( );	
}
//iic д��ĳ��λ
bool iicDevWriteBit(u8 devaddr,u8 addr, u8 bitNum, u8 data)
{
    u8 byte;
    iicDevReadByte(devaddr, addr, &byte);
    byte = (data != 0) ? (byte | (1 << bitNum)) : (byte & ~(1 << bitNum));
    iicDevWriteByte(devaddr, addr, byte);
	return true;
}























