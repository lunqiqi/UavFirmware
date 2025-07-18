#include <math.h>
#include "flip.h"
#include "config_param.h"
#include "commander.h"
#include "stabilizer.h"

/*FreeRTOS���ͷ�ļ�*/
#include "FreeRTOS.h"
#include "task.h"

/********************************************************************************	 


#define FLIP_RATE		RATE_500_HZ				/*����*/	
#define MID_ANGLE		(180.f * FLIP_RATE)		/*�м�Ƕ� �Ŵ�500��*/
#define MAX_FLIP_RATE	1380					/* <2000 */
#define DELTA_RATE		(30000.f/MAX_FLIP_RATE)	/*��������*/

#define FLIP_TIMEOUT		800			/*�������̳�ʱʱ��*/
#define SPEED_UP_TIMEOUT	400			/*����������ʱʱ��*/
#define REVER_SPEEDUP_TIME	160			/*�������ʱ��*/


static enum
{
	FLIP_IDLE = 0,
	FLIP_SET,
	FLIP_SPEED_UP,
	FLIP_SLOW_DOWN,
	FLIP_PERIOD,
	FLIP_FINISHED,
	REVER_SPEED_UP,
	FLIP_ERROR,
}flipState = FLIP_IDLE;

u8 fstate;				/*����״̬*/
enum dir_e flipDir;		/*��������*/
static u16 maxRateCnt = 0;			/*������ʼ���*/
static float desiredVelZ = 105.f;	/*�������������ٶ�*/
static float currentRate = 0.f;		/*��ǰ����*/
static float currentAngle = 0.f;	/*��ǰ�Ƕ� �Ŵ�500��*/
bool isExitFlip = true;				/*�Ƿ��˳��շ�*/

/********************************************************
* Flyer ������� 
*********************************************************/
void flyerFlipCheck(setpoint_t* setpoint, control_t* control, state_t* state)
{
	static u16 flipThrust = 0;
	static u16 tempThrust = 0;
	static u16 reverTime = 0;
	static u16 flipTimeout = 0;	
	static float pitchTemp = 0.0;
	static float rollTemp = 0.0;
	static float yawTemp = 0.0;
	static float deltaThrust = 0.0;
	static u16 exitFlipCnt = 0;
	static u16 flipThrustMax = 0;

	fstate = (u8)flipState;
	
	switch(flipState)
	{
		case FLIP_IDLE:	/*��������״̬*/
		{
			if(flipDir!=CENTER)
			{
				if(control->thrust > 28000 && state->velocity.z > -20.f)
				{
					flipState = FLIP_SET;
					exitFlipCnt = 500;		/*�շ���ɣ���ʱ1S(500Hz)�˳��շ� */
					isExitFlip = false;
				}					
				else
				{
					flipDir = CENTER;					
				}					
			}else if(isExitFlip == false)
			{
				if(exitFlipCnt > 0) 
					exitFlipCnt--;
				else
					isExitFlip = true;
			}				
			break;
		}
		case FLIP_SET:	/*��������*/
		{
			currentRate = 0.f;
			maxRateCnt = 0;
			currentAngle = 0.f;
			
			flipTimeout = 0;
			control->flipDir = flipDir;
			flipThrust = -9000.0f + 1.2f * configParam.thrustBase;
			deltaThrust = configParam.thrustBase / 90.0f;
			flipThrustMax = configParam.thrustBase + 20000;
			if(flipThrustMax > 62000) flipThrustMax = 62000;
			tempThrust = flipThrust; 
					
			rollTemp = state->attitude.roll;
			pitchTemp = state->attitude.pitch;									
			yawTemp = state->attitude.yaw;
			
			flipState = FLIP_SPEED_UP;
			break;
		}
		case FLIP_SPEED_UP:	/*������������*/
		{
			if(state->velocity.z < desiredVelZ)
			{
				setpoint->mode.z = modeDisable;
				if(tempThrust < flipThrustMax)
					tempThrust += deltaThrust;
				setpoint->thrust = tempThrust;
				
				if(flipTimeout++ > SPEED_UP_TIMEOUT)	/*��ʱ����*/
				{
					flipTimeout = 0;
					flipState = FLIP_SLOW_DOWN;			/*ֱ�ӽ�����һ��״̬*/
				}														
			}else	
			{	
				flipTimeout = 0;				
				flipState = FLIP_SLOW_DOWN;
			}		
			break;
		}
		case FLIP_SLOW_DOWN:	/*���ٹ���*/
		{
			if(tempThrust > flipThrust)
			{
				tempThrust -= (6500.f - flipThrust / 10.0f);
				setpoint->mode.z = modeDisable;
				setpoint->thrust = tempThrust;
			}else
			{
				flipState = FLIP_PERIOD;
			}
		}
		case FLIP_PERIOD:	/*��������*/
		{
			if(flipTimeout++ > FLIP_TIMEOUT)	/*��ʱ����*/
			{
				flipTimeout = 0;
				flipState = FLIP_ERROR;
			}
			
			setpoint->mode.z = modeDisable;
			setpoint->thrust = flipThrust - 3*currentRate;
			
			currentAngle += currentRate;		/*��ǰ�Ƕ� �Ŵ�500��*/
			
			if(currentAngle < MID_ANGLE)		/*�ϰ�Ȧ*/
			{
				if(currentRate < MAX_FLIP_RATE)/*С��������ʣ����ʼ�������*/
					currentRate += DELTA_RATE;
				else			/*����������ʣ����ʱ���*/
					maxRateCnt++;					
			}else	/*�°�Ȧ*/
			{
				if(maxRateCnt > 0)
				{						
					maxRateCnt--;			
				}else
				{
					if(currentRate >= DELTA_RATE && currentAngle < 2*MID_ANGLE)
					{
						currentRate -= DELTA_RATE;	
					}																
					else							
						flipState = FLIP_FINISHED;						
				}
			}
			
			switch(control->flipDir)	
			{
				case FORWARD:	/* pitch+ */
					setpoint->attitude.pitch = currentRate;	
					setpoint->attitude.roll = state->attitude.roll = rollTemp;
					setpoint->attitude.yaw = state->attitude.yaw = yawTemp;
					break;				
				case BACK:		/* pitch- */
					setpoint->attitude.pitch = -currentRate;
					setpoint->attitude.roll = state->attitude.roll = rollTemp;
					setpoint->attitude.yaw = state->attitude.yaw = yawTemp;
					break;
				case LEFT:		/* roll- */
					setpoint->attitude.roll = -currentRate;	
					setpoint->attitude.pitch = state->attitude.pitch = pitchTemp;
					setpoint->attitude.yaw = state->attitude.yaw = yawTemp;
					break;
				case RIGHT:		/* roll+ */					
					setpoint->attitude.roll = currentRate;
					setpoint->attitude.pitch = state->attitude.pitch = pitchTemp;
					setpoint->attitude.yaw = state->attitude.yaw = yawTemp;
					break;
				default :break;
			}
			break;
		}
		case FLIP_FINISHED:	/*�������*/
		{
			setpoint->mode.z = modeDisable;
			setpoint->thrust = tempThrust;
			tempThrust = flipThrust;
			
			reverTime = 0;
			flipTimeout = 0;
			flipDir = CENTER;	
			control->flipDir = flipDir;

			flipState = REVER_SPEED_UP;
			break;
		}
		case REVER_SPEED_UP:	/*������ɺ� �������*/
		{			
			if(reverTime++<REVER_SPEEDUP_TIME)	
			{
				if(tempThrust < flipThrustMax)
					tempThrust += 2.0f * deltaThrust;
				setpoint->mode.z = modeDisable;
				setpoint->thrust = tempThrust;
			}else
			{				
				flipTimeout = 0;
				flipState = FLIP_IDLE;					
//				if(getCommanderKeyFlight())	/*����ģʽ*/
//				{
//					setpoint->thrust = 0;
//					setpoint->mode.z = modeAbs;	
//				}
			}
			break;
		}
		case FLIP_ERROR:
		{
			reverTime = 0;
			flipDir = CENTER;	
			control->flipDir = CENTER;
			
			setpoint->mode.z = modeDisable;
			setpoint->thrust = 0;
			if(flipTimeout++ > 1)
			{
				flipTimeout = 0;
				
				if(getCommanderKeyFlight())	/*����ģʽ*/
				{
					setpoint->thrust = 0;
					setpoint->mode.z = modeAbs;				
				}
				
				flipState = FLIP_IDLE;
			}
			break;
		}
		default : break;
	}			
}


//���÷�������
void setFlipDir(u8 dir)
{
	flipDir = (enum dir_e)dir;	
}

