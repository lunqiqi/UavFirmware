#ifndef __PID_H
#define __PID_H
#include <stdbool.h>
#include "config_param.h"

������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
 *
 * �޸�˵��:
 * �汾V1.3 ����PID�ṹ������һ��(out)��
********************************************************************************/
#define DEFAULT_PID_INTEGRATION_LIMIT 		500.0 //Ĭ��pid�Ļ����޷�
#define DEFAULT_PID_OUTPUT_LIMIT      		0.0	  //Ĭ��pid����޷���0Ϊ���޷�

typedef struct
{
	float desired;		//< set point
	float error;        //< error
	float prevError;    //< previous error
	float integ;        //< integral
	float deriv;        //< derivative
	float kp;           //< proportional gain
	float ki;           //< integral gain
	float kd;           //< derivative gain
	float outP;         //< proportional output (debugging)
	float outI;         //< integral output (debugging)
	float outD;         //< derivative output (debugging)
	float iLimit;       //< integral limit
	float outputLimit;  //< total PID output limit, absolute value. '0' means no limit.
	float dt;           //< delta-time dt
	float out;			//< out
} PidObject;

/*pid�ṹ���ʼ��*/
void pidInit(PidObject* pid, const float desired, const pidInit_t pidParam, const float dt);
void pidSetIntegralLimit(PidObject* pid, const float limit);/*pid�����޷�����*/
void pidSetOutputLimit(PidObject* pid, const float limit);
void pidSetDesired(PidObject* pid, const float desired);	/*pid��������ֵ*/
float pidUpdate(PidObject* pid, const float error);			/*pid����*/
float pidGetDesired(PidObject* pid);	/*pid��ȡ����ֵ*/
bool pidIsActive(PidObject* pid);		/*pid״̬*/
void pidReset(PidObject* pid);			/*pid�ṹ�帴λ*/
void pidSetError(PidObject* pid, const float error);/*pidƫ������*/
void pidSetKp(PidObject* pid, const float kp);		/*pid Kp����*/
void pidSetKi(PidObject* pid, const float ki);		/*pid Ki����*/
void pidSetKd(PidObject* pid, const float kd);		/*pid Kd����*/
void pidSetDt(PidObject* pid, const float dt);		/*pid dt����*/

#endif /* __PID_H */
