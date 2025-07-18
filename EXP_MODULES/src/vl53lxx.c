#include "system.h"
#include "vl53lxx_i2c.h"
#include "vl53lxx.h"
#include "vl53l1_api.h"

#include "FreeRTOS.h"
#include "task.h"

/********************************************************************************	 

********************************************************************************/

TaskHandle_t vl53l0xTaskHandle = NULL;
TaskHandle_t vl53l1xTaskHandle = NULL;

u16 vl53lxxId = 0;	/*vl53оƬID*/
bool isEnableVl53lxx = true;		/*�Ƿ�ʹ�ܼ���*/

static bool isInitvl53l0x = false;	/*��ʼ��vl53l0x*/
static bool isInitvl53l1x = false;	/*��ʼ��vl53l1x*/
static bool reInitvl53l0x = false;	/*�ٴγ�ʼ��vl53l0x*/
static bool reInitvl53l1x = false;	/*�ٴγ�ʼ��vl53l1x*/

static u8 count = 0;
static u8 validCnt = 0;
static u8 inValidCnt = 0;

static u16 range_last = 0;
float quality = 1.0f;

zRange_t vl53lxx;


void vl53l0xTask(void* arg);
void vl53l1xTask(void* arg);
	
void vl53lxxInit(void)
{
	vl53IICInit();	
	delay_ms(10);
	
	/*vl53l0x ��ʼ��*/
	vl53lxxId = vl53l0xGetModelID();
	if(vl53lxxId == VL53L0X_ID)
	{
		if (isInitvl53l0x)
		{
			reInitvl53l0x = true;
			vTaskResume(vl53l0xTaskHandle);	/*�ָ�����������*/
		}
		else	/*�״ν���vl53l0x����ģ��*/
		{	
			isInitvl53l0x = true;
			xTaskCreate(vl53l0xTask, "VL5310X", 300, NULL, 5, &vl53l0xTaskHandle);	/*����������ģ������*/
		}
		return;
	}			
	delay_ms(10);
	
	/*vl53l1x ��ʼ��*/
	VL53L1_RdWord(&dev, 0x010F, &vl53lxxId);
	if(vl53lxxId == VL53L1X_ID)
	{
		if (isInitvl53l1x)
		{
			reInitvl53l1x = true;
			vTaskResume(vl53l1xTaskHandle);	/*�ָ�����������*/
		}
		else	/*�״ν���vl53l1x����ģ��*/
		{		
			isInitvl53l1x = true;			
			xTaskCreate(vl53l1xTask, "VL53L1X", 300, NULL, 5, &vl53l1xTaskHandle);	/*����������ģ������*/
		}
		return;
	}	
	
	vl53lxxId = 0;
}

void vl53l0xTask(void* arg)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	
	vl53l0xSetParam();	/*����vl53l0x ����*/
		
	while (1) 
	{
		if(reInitvl53l0x == true)
		{
			count = 0;
			reInitvl53l0x = false;			
			vl53l0xSetParam();	/*����vl53l0x ����*/
			xLastWakeTime = xTaskGetTickCount();
			
		}else
		{
			range_last = vl53l0xReadRangeContinuousMillimeters() * 0.1f;	//��λcm

			if(range_last < VL53L0X_MAX_RANGE)			
				validCnt++;			
			else 			
				inValidCnt++;			
			
			if(inValidCnt + validCnt == 10)
			{
				quality += (validCnt/10.f - quality) * 0.1f;	/*��ͨ*/
				validCnt = 0;
				inValidCnt = 0;
			}
			
			if(range_last >= 6550)	/*vl53 ����*/
			{
				if(++count > 30)
				{
					count = 0;
					vTaskSuspend(vl53l0xTaskHandle);	/*���𼤹�������*/					
				}				
			}else count = 0;						

			vTaskDelayUntil(&xLastWakeTime, measurement_timing_budget_ms);
		}		
	}
}


void vl53l1xTask(void* arg)
{
	int status;
	u8 isDataReady = 0;
	TickType_t xLastWakeTime = xTaskGetTickCount();;
	static VL53L1_RangingMeasurementData_t rangingData;

	vl53l1xSetParam();	/*����vl53l1x ����*/
	
	while(1) 
	{
		if(reInitvl53l1x == true)
		{
			count = 0;
			reInitvl53l1x = false;			
			vl53l1xSetParam();	/*����vl53l1x ����*/
			xLastWakeTime = xTaskGetTickCount();
		}else
		{	
			status = VL53L1_GetMeasurementDataReady(&dev, &isDataReady);
						
			if(isDataReady)
			{
				status = VL53L1_GetRangingMeasurementData(&dev, &rangingData);
				if(status==0)
				{
					range_last = rangingData.RangeMilliMeter * 0.1f;	/*��λcm*/				
				}
				status = VL53L1_ClearInterruptAndStartMeasurement(&dev);
			}	
			
			if(range_last < VL53L1X_MAX_RANGE)			
				validCnt++;			
			else 			
				inValidCnt++;			
			
			if(inValidCnt + validCnt == 10)
			{
				quality += (validCnt/10.f - quality) * 0.1f;	/*��ͨ*/
				validCnt = 0;
				inValidCnt = 0;
			}
			
			if(getModuleID() != OPTICAL_FLOW)
			{
				if(++count > 10)
				{
					count = 0;
					VL53L1_StopMeasurement(&dev);
					vTaskSuspend(vl53l1xTaskHandle);	/*���𼤹�������*/					
				}				
			}else count = 0;
						
			vTaskDelayUntil(&xLastWakeTime, 50);
		}		
	}
}

bool vl53lxxReadRange(zRange_t* zrange)
{
	if(vl53lxxId == VL53L0X_ID) 
	{
		zrange->quality = quality;		//���Ŷ�
		vl53lxx.quality = quality;
		
		if (range_last != 0 && range_last < VL53L0X_MAX_RANGE) 
		{			
			zrange->distance = (float)range_last;	//��λ[cm]
			vl53lxx.distance = 	zrange->distance;		
			return true;
		}
	}
	else if(vl53lxxId == VL53L1X_ID) 
	{
		zrange->quality = quality;		//���Ŷ�
		vl53lxx.quality = quality;
		
		if (range_last != 0 && range_last < VL53L1X_MAX_RANGE) 
		{			
			zrange->distance = (float)range_last;	//��λ[cm]	
			vl53lxx.distance = 	zrange->distance;
			return true;
		}
	}
	
	return false;
}
/*ʹ�ܼ���*/
void setVl53lxxState(u8 enable)
{
	isEnableVl53lxx = enable;
}


