#include "string.h"
#include "stdbool.h"
#include "optical_flow.h"
#include "config_param.h"
#include "commander.h"
#include "delay.h"
#include "maths.h"
#include "state_estimator.h"

#include "filter.h"
#include "arm_math.h"

/*FreeRTOS���ͷ�ļ�*/
#include "FreeRTOS.h"
#include "task.h"

/********************************************************************************	 

********************************************************************************/

#define NCS_PIN					PAout(8)
#define OPTICAL_POWER_ENABLE	PBout(0)

#define RESOLUTION			(0.2131946f)/*1m�߶��� 1�����ض�Ӧ��λ�ƣ���λcm*/
#define OULIER_LIMIT 		(100)		/*������������޷�*/
#define VEL_LIMIT			(150.f)		/*�����ٶ��޷�*/

#define VEL_LPF_FILTER			/*��ͨ�˲�*/
//#define AVERAGE_FILTER		/*��ֵ�˲�*/

static bool isInit = false;
static u8 outlierCount = 0;			/*���ݲ����ü���*/

opFlow_t opFlow;	/*����*/


TaskHandle_t opFlowTaskHandle = NULL;

#if defined(__CC_ARM) 
	#pragma anon_unions	/*����֧�ֽṹ��������*/
#endif

typedef __packed struct motionBurst_s 
{
	__packed union 
	{
		uint8_t motion;
		__packed struct 
		{
			uint8_t frameFrom0    : 1;
			uint8_t runMode       : 2;
			uint8_t reserved1     : 1;
			uint8_t rawFrom0      : 1;
			uint8_t reserved2     : 2;
			uint8_t motionOccured : 1;
		};
	};

	uint8_t observation;
	int16_t deltaX;
	int16_t deltaY;

	uint8_t squal;

	uint8_t rawDataSum;
	uint8_t maxRawData;
	uint8_t minRawData;

	uint16_t shutter;
} motionBurst_t;

motionBurst_t currentMotion;

static void InitRegisters(void);

//������Դ����
void opticalFlowPowerControl(bool state)
{
	if(state == true)
		OPTICAL_POWER_ENABLE = true;
	else
		OPTICAL_POWER_ENABLE = false;
}

static void registerWrite(uint8_t reg, uint8_t value)
{
	// ���λΪ1 д�Ĵ���
	reg |= 0x80u;

	spiBeginTransaction();
	
	NCS_PIN = 0;
	
	delay_us(50);
	spiExchange(1, &reg, &reg);
	delay_us(50);
	spiExchange(1, &value, &value);
	delay_us(50);

	NCS_PIN = 1;
	
	spiEndTransaction();
	delay_us(200);
}

static uint8_t registerRead(uint8_t reg)
{
	uint8_t data = 0;

	// ���λΪ0 ���Ĵ���
	reg &= ~0x80u;

	spiBeginTransaction();
	
	NCS_PIN = 0;
	
	delay_us(50);	
	spiExchange(1, &reg, &reg);
	delay_us(500);
	spiExchange(1, &data, &data);	
	delay_us(50);
	
	NCS_PIN = 1;
	
	spiEndTransaction();
	delay_us(200);

	return data;
}

static void readMotion(motionBurst_t * motion)
{
	uint8_t address = 0x16;

	spiBeginTransaction();
	
	NCS_PIN = 0;
	
	delay_us(50);	
	spiExchange(1, &address, &address);
	delay_us(50);
	spiExchange(sizeof(motionBurst_t), (uint8_t*)motion, (uint8_t*)motion);	
	delay_us(50);
	
	NCS_PIN = 1;
	
	spiEndTransaction();
	delay_us(50);

	uint16_t realShutter = (motion->shutter >> 8) & 0x0FF;
	realShutter |= (motion->shutter & 0x0ff) << 8;
	motion->shutter = realShutter;
}

static void InitRegisters(void)
{	
	registerWrite(0x7F, 0x00);
	registerWrite(0x61, 0xAD);
	registerWrite(0x7F, 0x03);
	registerWrite(0x40, 0x00);
	registerWrite(0x7F, 0x05);
	registerWrite(0x41, 0xB3);
	registerWrite(0x43, 0xF1);
	registerWrite(0x45, 0x14);
	registerWrite(0x5B, 0x32);
	registerWrite(0x5F, 0x34);
	registerWrite(0x7B, 0x08);
	registerWrite(0x7F, 0x06);
	registerWrite(0x44, 0x1B);
	registerWrite(0x40, 0xBF);
	registerWrite(0x4E, 0x3F);
	registerWrite(0x7F, 0x08);
	registerWrite(0x65, 0x20);
	registerWrite(0x6A, 0x18);
	registerWrite(0x7F, 0x09);
	registerWrite(0x4F, 0xAF);
	registerWrite(0x5F, 0x40);
	registerWrite(0x48, 0x80);
	registerWrite(0x49, 0x80);
	registerWrite(0x57, 0x77);
	registerWrite(0x60, 0x78);
	registerWrite(0x61, 0x78);
	registerWrite(0x62, 0x08);
	registerWrite(0x63, 0x50);
	registerWrite(0x7F, 0x0A);
	registerWrite(0x45, 0x60);
	registerWrite(0x7F, 0x00);
	registerWrite(0x4D, 0x11);
	registerWrite(0x55, 0x80);
	registerWrite(0x74, 0x1F);
	registerWrite(0x75, 0x1F);
	registerWrite(0x4A, 0x78);
	registerWrite(0x4B, 0x78);
	registerWrite(0x44, 0x08);
	registerWrite(0x45, 0x50);
	registerWrite(0x64, 0xFF);
	registerWrite(0x65, 0x1F);
	registerWrite(0x7F, 0x14);
	registerWrite(0x65, 0x67);
	registerWrite(0x66, 0x08);
	registerWrite(0x63, 0x70);
	registerWrite(0x7F, 0x15);
	registerWrite(0x48, 0x48);
	registerWrite(0x7F, 0x07);
	registerWrite(0x41, 0x0D);
	registerWrite(0x43, 0x14);
	registerWrite(0x4B, 0x0E);
	registerWrite(0x45, 0x0F);
	registerWrite(0x44, 0x42);
	registerWrite(0x4C, 0x80);
	registerWrite(0x7F, 0x10);
	registerWrite(0x5B, 0x02);
	registerWrite(0x7F, 0x07);
	registerWrite(0x40, 0x41);
	registerWrite(0x70, 0x00);

	vTaskDelay(10); // delay 10ms

	registerWrite(0x32, 0x44);
	registerWrite(0x7F, 0x07);
	registerWrite(0x40, 0x40);
	registerWrite(0x7F, 0x06);
	registerWrite(0x62, 0xF0);
	registerWrite(0x63, 0x00);
	registerWrite(0x7F, 0x0D);
	registerWrite(0x48, 0xC0);
	registerWrite(0x6F, 0xD5);
	registerWrite(0x7F, 0x00);
	registerWrite(0x5B, 0xA0);
	registerWrite(0x4E, 0xA8);
	registerWrite(0x5A, 0x50);
	registerWrite(0x40, 0x80);
	
//	/*��ʼ��LED_N*/
//	registerWrite(0x7F, 0x0E);
//	registerWrite(0x72, 0x0F);
//	registerWrite(0x7F, 0x00);
}
/*��λ��������*/
static void resetOpFlowData(void)
{
	for(u8 i=0; i<2; i++)
	{
		opFlow.pixSum[i] = 0;
		opFlow.pixComp[i] = 0;
		opFlow.pixValid[i] = 0;
		opFlow.pixValidLast[i] = 0;
	}
}

/*����������*/
void opticalFlowTask(void *param)
{	
	static u16 count = 0;	
	u32 lastWakeTime = getSysTickCnt();
		
	opFlow.isOpFlowOk = true;
	
	while(1) 
	{
		vTaskDelayUntil(&lastWakeTime, 10);		/*100Hz 10ms������ʱ*/
		
		readMotion(&currentMotion);

		if(currentMotion.minRawData == 0 && currentMotion.maxRawData == 0)
		{
			if(count++ > 100 && opFlow.isOpFlowOk == true)
			{
				count = 0;
				opFlow.isOpFlowOk = false;		/*��������*/
				vTaskSuspend(opFlowTaskHandle);	/*�����������*/
			}		
		}else
		{
			count = 0;
		}
		/*����2֮֡������ر仯������ʵ�ʰ�װ������� (pitch:x)  (roll:y)*/
		int16_t pixelDx = currentMotion.deltaY;
		int16_t pixelDy = -currentMotion.deltaX;

		if (ABS(pixelDx) < OULIER_LIMIT && ABS(pixelDy) < OULIER_LIMIT) 
		{
			opFlow.pixSum[X] += pixelDx;
			opFlow.pixSum[Y] += pixelDy;
		}else 
		{
			outlierCount++;
		}
	}
}

#ifdef AVERAGE_FILTER

#define GROUP		2
#define FILTER_NUM	3

/*�޷���ֵ�˲���*/
void velFilter(float* in, float* out)
{	
	static u8 i=0;
	static float filter_buf[GROUP][FILTER_NUM] = {0.0};
	double filter_sum[GROUP] = {0.0};		
	
	for(u8 j=0;j<GROUP;j++)
	{
		if(filter_buf[j][i] == 0.0f)
		{
			filter_buf[j][i] = in[j];
			out[j] = in[j];			
		} else 
		{			
			if(fabs(in[j]) < VEL_LIMIT)
			{
				filter_buf[j][i] = in[j];
			}
			for(u8 cnt=0;cnt<FILTER_NUM;cnt++)
			{
				filter_sum[j] += filter_buf[j][cnt];
			}
			out[j]=filter_sum[j] /FILTER_NUM;
		}
	}
	if(++i >= FILTER_NUM)	i = 0;
}
#endif

bool getOpFlowData(state_t *state, float dt)
{
	static u8 cnt = 0;
	float height = 0.01f * getFusedHeight();/*��ȡ�߶���Ϣ ��λm*/
	
	if(opFlow.isOpFlowOk && height<4.0f)	/*4m��Χ�ڣ���������*/
	{
		cnt= 0;
		opFlow.isDataValid = true;
		
		float coeff = RESOLUTION * height;
		float tanRoll = tanf(state->attitude.roll * DEG2RAD);
		float tanPitch = tanf(state->attitude.pitch * DEG2RAD);
		
		opFlow.pixComp[X] = 480.f * tanPitch;	/*���ز�����������*/
		opFlow.pixComp[Y] = 480.f * tanRoll;
		opFlow.pixValid[X] = (opFlow.pixSum[X] + opFlow.pixComp[X]);	/*ʵ���������*/
		opFlow.pixValid[Y] = (opFlow.pixSum[Y] + opFlow.pixComp[Y]);		
		
		if(height < 0.05f)	/*����������Χ����5cm*/
		{
			coeff = 0.0f;
		}
		opFlow.deltaPos[X] = coeff * (opFlow.pixValid[X] - opFlow.pixValidLast[X]);	/*2֮֡��λ�Ʊ仯������λcm*/
		opFlow.deltaPos[Y] = coeff * (opFlow.pixValid[Y] - opFlow.pixValidLast[Y]);	
		opFlow.pixValidLast[X] = opFlow.pixValid[X];	/*��һ��ʵ���������*/
		opFlow.pixValidLast[Y] = opFlow.pixValid[Y];
		opFlow.deltaVel[X] = opFlow.deltaPos[X] / dt;	/*�ٶ� cm/s*/
		opFlow.deltaVel[Y] = opFlow.deltaPos[Y] / dt;
		
#ifdef AVERAGE_FILTER
		velFilter(opFlow.deltaVel, opFlow.velLpf);		/*�޷���ֵ�˲���*/
#else
		opFlow.velLpf[X] += (opFlow.deltaVel[X] - opFlow.velLpf[X]) * 0.15f;	/*�ٶȵ�ͨ cm/s*/
		opFlow.velLpf[Y] += (opFlow.deltaVel[Y] - opFlow.velLpf[Y]) * 0.15f;	/*�ٶȵ�ͨ cm/s*/
#endif			
		opFlow.velLpf[X] = constrainf(opFlow.velLpf[X], -VEL_LIMIT, VEL_LIMIT);	/*�ٶ��޷� cm/s*/
		opFlow.velLpf[Y] = constrainf(opFlow.velLpf[Y], -VEL_LIMIT, VEL_LIMIT);	/*�ٶ��޷� cm/s*/
	
		opFlow.posSum[X] += opFlow.deltaPos[X];	/*�ۻ�λ�� cm*/
		opFlow.posSum[Y] += opFlow.deltaPos[Y];	/*�ۻ�λ�� cm*/
	}
	else if(opFlow.isDataValid == true)
	{
		if(cnt++ > 100)	/*��������߶ȣ��л�Ϊ����ģʽ*/
		{
			cnt = 0;
			opFlow.isDataValid = false;
		}	
		resetOpFlowData();	
	}
	
	return opFlow.isOpFlowOk;	/*���ع���״̬*/
}
/*��ʼ������ģ��*/
void opticalFlowInit(void)
{
	if (!isInit) /*��һ�γ�ʼ��ͨ��IO*/
	{
		GPIO_InitTypeDef GPIO_InitStructure;

		//��ʼ��CS����	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��ʱ��
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��ʱ��
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;	
		GPIO_Init(GPIOA, &GPIO_InitStructure);		

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
		GPIO_Init(GPIOB, &GPIO_InitStructure);		
	}
	else 
	{
		resetOpFlowData();
		opFlow.isOpFlowOk = true;				
	}
	
	opticalFlowPowerControl(true);	/*�򿪵�Դ*/
	vTaskDelay(50);
	
	NCS_PIN = 1;
	spi2Init();
	vTaskDelay(40);

	uint8_t chipId = registerRead(0);
	uint8_t invChipId = registerRead(0x5f);
//	printf("Motion chip is: 0x%x\n", chipId);
//	printf("si pihc noitoM: 0x%x\n", invChipId);

	// �ϵ縴λ
	registerWrite(0x3a, 0x5a);
	vTaskDelay(5);

	InitRegisters();
	vTaskDelay(5);
	
	if (isInit) 
	{
		vTaskResume(opFlowTaskHandle);	/*�ָ���������*/
	}
	else if(opFlowTaskHandle == NULL)
	{
		xTaskCreate(opticalFlowTask, "OPTICAL_FLOW", 300, NULL, 4, &opFlowTaskHandle);	/*��������ģ������*/
	}		

	vl53lxxInit();	/*��ʼ��vl53lxx*/
	
	isInit = true;
}

/*��ȡ��������״̬*/
bool getOpDataState(void)
{
	return opFlow.isDataValid;
}
	
	

