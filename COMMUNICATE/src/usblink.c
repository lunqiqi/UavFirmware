#include <stdbool.h>
#include <string.h>
#include "config.h"
#include "usblink.h"
#include "atkp.h"
#include "config_param.h"
#include "ledseq.h"
#include "pm.h"
#include "usbd_cdc_vcp.h"
#include "usbd_usr.h"

/*FreeRTOS���ͷ�ļ�*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/********************************************************************************	 

********************************************************************************/

#define USBLINK_TX_QUEUE_SIZE 	30 /*���ն��и���*/

static enum
{
	waitForStartByte1,
	waitForStartByte2,
	waitForMsgID,
	waitForDataLength,
	waitForData,
	waitForChksum1,
}rxState;

static bool isInit;
static atkp_t rxPacket;
static xQueueHandle  txQueue;


/*usb���ӳ�ʼ��*/
void usblinkInit()
{
	if(isInit) return;
	
	usbd_cdc_vcp_Init();
	/*�������Ͷ��У�USBLINK_TX_QUEUE_SIZE����Ϣ*/
	txQueue = xQueueCreate(USBLINK_TX_QUEUE_SIZE, sizeof(atkp_t));
	ASSERT(txQueue);
	isInit = true;
}

/*usb���ӷ���atkpPacket*/
bool usblinkSendPacket(const atkp_t *p)
{
	ASSERT(p);
	ASSERT(p->dataLen <= ATKP_MAX_DATA_SIZE);
	return xQueueSend(txQueue, p, 0);	
}

//��ȡʣ�����txQueue����
int usblinkGetFreeTxQueuePackets(void)	
{
	return (USBLINK_TX_QUEUE_SIZE - uxQueueMessagesWaiting(txQueue));
}

//USB����ATKPPacket����
void usblinkTxTask(void *param)
{
	atkp_t p;
	u8 sendBuffer[64];
	u8 cksum;
	u8 dataLen;
	while(1)
	{
		xQueueReceive(txQueue, &p, portMAX_DELAY);
		
		sendBuffer[0] = UP_BYTE1;
		sendBuffer[1] = UP_BYTE2;
		sendBuffer[2] = p.msgID;
		sendBuffer[3] = p.dataLen;
		memcpy(&sendBuffer[4], p.data, p.dataLen);
		cksum = 0;
		for (int i = 0; i < p.dataLen+4; i++)
		{
			cksum += sendBuffer[i];
		}
		dataLen = p.dataLen + 5;
		sendBuffer[dataLen - 1] = cksum;
		usbsendData(sendBuffer, dataLen);
		ledseqRun(DATA_TX_LED, seq_linkup);
	}
}

//USB���⴮�ڽ���ATKPPacket����
void usblinkRxTask(void *param)
{
	u8 c;
	u8 dataIndex = 0;
	u8 cksum = 0;
	rxState = waitForStartByte1;
	while(1)
	{
		if (usbGetDataWithTimout(&c))
		{
			switch(rxState)
			{
				case waitForStartByte1:
					rxState = (c == DOWN_BYTE1) ? waitForStartByte2 : waitForStartByte1;
					cksum = c;
					break;
				case waitForStartByte2:
					rxState = (c == DOWN_BYTE2) ? waitForMsgID : waitForStartByte1;
					cksum += c;
					break;
				case waitForMsgID:
					rxPacket.msgID = c;
					rxState = waitForDataLength;
					cksum += c;
					break;
				case waitForDataLength:
					if (c <= ATKP_MAX_DATA_SIZE)
					{
						rxPacket.dataLen = c;
						dataIndex = 0;
						rxState = (c > 0) ? waitForData : waitForChksum1;	/*c=0,���ݳ���Ϊ0��У��1*/
						cksum += c;
					} else 
					{
						rxState = waitForStartByte1;
					}
					break;
				case waitForData:
					rxPacket.data[dataIndex] = c;
					dataIndex++;
					cksum += c;
					if (dataIndex == rxPacket.dataLen)
					{
						rxState = waitForChksum1;
					}
					break;
				case waitForChksum1:
					if (cksum == c)	/*����У����ȷ*/
					{
						ledseqRun(DATA_RX_LED, seq_linkup);
						atkpReceivePacketBlocking(&rxPacket);
					} 
					else	/*У�����*/
					{
						rxState = waitForStartByte1;	
						IF_DEBUG_ASSERT(1);
					}
					rxState = waitForStartByte1;
					break;
				default:
					ASSERT(0);
					break;
			}
		}
		else	/*��ʱ����*/
		{
			rxState = waitForStartByte1;
		}
	}
}

