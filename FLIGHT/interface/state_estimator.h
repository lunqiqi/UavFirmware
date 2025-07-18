#ifndef __STATE_ESTIMATOR_H
#define __STATE_ESTIMATOR_H
#include "stabilizer_types.h"

/********************************************************************************	 

********************************************************************************/

typedef struct
{
	float vAccDeadband; /* ���ٶ����� */
	float accBias[3];	/* ���ٶ� ƫ��(cm/s/s)*/
	float acc[3];		/* ������ٶ� ��λ(cm/s/s)*/
	float vel[3];		/* �����ٶ� ��λ(cm/s)*/
	float pos[3]; 		/* ����λ�� ��λ(cm)*/
} estimator_t;

void positionEstimate(sensorData_t* sensorData, state_t* state, float dt);	
float getFusedHeight(void);	/*��ȡ�ںϸ߶�*/
void estRstHeight(void);	/*��λ����߶�*/
void estRstAll(void);		/*��λ���й���*/

#endif /* __STATE_ESTIMATOR_H */


