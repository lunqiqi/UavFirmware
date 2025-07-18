#include <math.h>
#include "maths.h"
#include "commander.h"
#include "anomal_detec.h"
#include "remoter_ctrl.h"
#include "stabilizer.h"
#include "module_mgt.h"



#if defined(DETEC_ENABLED)

static u16 outFlipCnt = 0;		


static bool detecFreeFall(float accZ, float accMAG)	/*����������*/
{
	static u16 cnt;

	/*��������*/
	if(fabs(accMAG) < DETEC_FF_THRESHOLD && fabs(accZ + 1.f) < DETEC_FF_THRESHOLD)	
	{	
		if(++cnt >= (DETEC_FF_COUNT))
		{
			return true;
		}		
	}
	else
	{
		cnt=0;
	}
	
	return false;
}

static bool detecTumbled(const state_t *state)	/*��ײ���*/
{
	static u16 cnt;
	
	float fAbsRoll  = fabs(state->attitude.roll);
	float fAbsPitch = fabs(state->attitude.pitch);
	float fMax = (fAbsRoll >= fAbsPitch) ? fAbsRoll : fAbsPitch;
	
	if(fMax > DETEC_TU_THRESHOLD)
	{
		if(++cnt >= DETEC_TU_COUNT)
		{
			return true;
		}
	}else 
	{
		cnt=0;
	}
	
	return false;
}
#endif

/*�쳣���*/
void anomalDetec(const sensorData_t *sensorData, const state_t *state, const control_t *control)
{
#if defined(DETEC_ENABLED)
	
	if(control->flipDir != CENTER) 
	{
		outFlipCnt = 1000;
		return;
	}	

	if(state->isRCLocked == false && 		//ң��������״̬
	getCommanderKeyFlight() == false &&		//δ����״̬
	(getCommanderCtrlMode() & 0x01) == 0x01)//����ģʽ
	{
		float accMAG = (sensorData->acc.x*sensorData->acc.x) +
						(sensorData->acc.y*sensorData->acc.y) +
						(sensorData->acc.z*sensorData->acc.z);

		if(detecFreeFall(state->acc.z/980.f, accMAG) == true)/*����������*/
		{				
			setCommanderKeyFlight(true);
			setFastAdjustPosParam(35, 10, 0.f);	/*���ÿ��ٵ���λ�ò���*/
		}
	}
	
	if(outFlipCnt > 0)	
	{
		outFlipCnt--;
	}
	if(outFlipCnt == 0 && detecTumbled(state)==true)/*��ײ���*/
	{
		setCommanderKeyFlight(false);
		setCommanderKeyland(false);
	}			

#endif
}


