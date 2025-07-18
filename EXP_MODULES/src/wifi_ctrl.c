#include "string.h"
#include <math.h>
#include "wifi_ctrl.h"
#include "uart1.h"
#include "config_param.h"
#include "commander.h"

/*FreeRTOS���ͷ�ļ�*/
#include "FreeRTOS.h"
#include "task.h"

/********************************************************************************	 

********************************************************************************/

typedef enum
{
	waitForStart,
	waitForData,
	waitForChksum,
	waitForEnd
} WifilinkRxState;

TaskHandle_t wifCtrlTaskHandle = NULL;

static bool isInit = false;
static u8 rawWifiData[8];
static ctrlVal_t wifiCtrl;/*���͵�commander��̬��������*/

/*wifi��Դ����*/
void wifiPowerControl(bool state)
{
	if(state == true)
		WIFI_POWER_ENABLE = true;
	else
		WIFI_POWER_ENABLE = false;
}

/*wifiģ���ʼ��*/
void wifiModuleInit(void)
{
	if(!isInit)	/*�״β���wifi����ͷ*/
	{
		GPIO_InitTypeDef GPIO_InitStructure;
		
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
		
		/* ����wifi��Դ���ƽ���� */
		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
				
		wifiPowerControl(true);	
		vTaskDelay(50);
		uart1Init(19200);	/*����1��ʼ���������ʹ̶�19200*/
		
		xTaskCreate(wifiLinkTask, "WIFILINK", 150, NULL, 4, &wifCtrlTaskHandle);		/*����ͨ����������*/
		
		isInit = true;
	}
	else
	{
		wifiPowerControl(true);
		vTaskDelay(50);
		vTaskResume(wifCtrlTaskHandle);
	}	
}

static bool wifiDataCrc(u8 *data)
{
	u8 temp=(data[1]^data[2]^data[3]^data[4]^data[5])&0xff;
	if(temp==data[6])
		return true;
	return false;
}

/*�������*/
static void wifiCmdProcess(u8 data)
{
	wifiCmd_t wifiCmd = *(wifiCmd_t*)&data;
	
	if(getCommanderCtrlMode() == true)/*��ǰ�������Ϊ����ģʽ*/
	{
		if(wifiCmd.keyFlight) /*һ�����*/
		{
			setCommanderKeyFlight(true);
			setCommanderKeyland(false);	
		}
		if(wifiCmd.keyLand) /*һ������*/
		{
			setCommanderKeyFlight(false);
			setCommanderKeyland(true);
		}
		if(wifiCmd.emerStop) /*����ͣ��*/
		{
			setCommanderKeyFlight(false);
			setCommanderKeyland(false);
			setCommanderEmerStop(true);
		}else
		{
			setCommanderEmerStop(false);
		}
	}
	else/*��ǰ�������Ϊ�ֶ���ģʽ*/
	{
		setCommanderCtrlMode(0);
		setCommanderKeyFlight(false);
		setCommanderKeyland(false);
	}

	setCommanderFlightmode(wifiCmd.flightMode);
	
	if(wifiCmd.flipOne) /*�̶����򷭹�*/
	{
	}
	if(wifiCmd.flipFour) /*4D����*/
	{
	}
	if(wifiCmd.ledControl) /*�ƹ����*/
	{		
	}
	if(wifiCmd.gyroCalib) /*����У׼*/
	{
	}
}

static void wifiDataHandle(u8 *data)
{
	static u16 lastThrust;
	
	wifiCtrl.roll   = ((float)data[1]-(float)0x80)*0.25f;	/*roll: ��9.5 ��19.2 ��31.7*/
	wifiCtrl.pitch  = ((float)data[2]-(float)0x80)*0.25f;	/*pitch:��9.5 ��19.2 ��31.7*/
	wifiCtrl.yaw    = ((float)data[4]-(float)0x80)*1.6f;	/*yaw : ��203.2*/				
	wifiCtrl.thrust = (u16)data[3] << 8;					/*thrust :0~63356*/
	
	if(wifiCtrl.thrust==32768 && lastThrust<10000)/*�ֶ����л�������*/
	{
		setCommanderCtrlMode(1);
		setCommanderKeyFlight(false);
		setCommanderKeyland(false);
	}
	else if(wifiCtrl.thrust==0 && lastThrust>256)/*�����л����ֶ���*/
	{
		setCommanderCtrlMode(0);
		wifiCtrl.thrust = 0;
	}
	lastThrust = wifiCtrl.thrust;

	wifiCmdProcess(data[5]);/*λ��־�������*/
	flightCtrldataCache(WIFI, wifiCtrl);
}

void wifiLinkTask(void *param)
{
	u8 c;
	u8 dataIndex=0;
	WifilinkRxState rxState=waitForStart;

	while(1)
	{
		if(getModuleID() != WIFI_CAMERA)	/*�Ƴ�wifi����ͷ*/
		{
			vTaskSuspend(wifCtrlTaskHandle);
		}
		
		if (uart1GetDataWithTimout(&c))
		{
			switch(rxState)
			{
				case waitForStart:
					if(c == 0x66)					/*��ʼ����ȷ*/
					{
						dataIndex=1;
						rawWifiData[0] = c;
						rxState = waitForData;
					} else							/*��ʼ������*/
					{
						rxState = waitForStart;
					}
					break;				
				case waitForData:
					rawWifiData[dataIndex] = c;
					dataIndex++;
					if (dataIndex == 6)				/*���ݽ�����ɣ�У��*/
					{
						rxState = waitForChksum;
					}
					break;
				case waitForChksum:
					rawWifiData[6] = c;
					if (wifiDataCrc(rawWifiData))	/*У����ȷ���жϽ�����*/
					{
						rxState = waitForEnd;
					} else
					{
						rxState = waitForStart;		/*У�����*/
					}
					break;
				case waitForEnd:
					if (c == 0x99)					/*��������ȷ*/
					{
						rawWifiData[7] = c;
						wifiDataHandle(rawWifiData);/*�������յ�������*/
					} else
					{
						rxState = waitForStart;		/*����������*/
						IF_DEBUG_ASSERT(1);
					}
					rxState = waitForStart;
					break;
				default:
					ASSERT(0);
					break;
			}
		}
		else	/*��ʱ����*/
		{
			rxState = waitForStart;
		}
	}
}






