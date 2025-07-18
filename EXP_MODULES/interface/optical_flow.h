#ifndef __OPTICAL_FLOW_H
#define __OPTICAL_FLOW_H
#include "sys.h"
#include <stdbool.h>
#include "spi.h"
#include "stabilizer_types.h"
#include "module_mgt.h"
#include "vl53lxx.h"

/********************************************************************************	 

 * �汾V1.3 ���ӹ������ݽṹ��opFlow_t�����ڴ�Ź����������ݣ������û����ԡ�
********************************************************************************/

typedef struct opFlow_s 
{
	float pixSum[2];		/*�ۻ�����*/
	float pixComp[2];		/*���ز���*/
	float pixValid[2];		/*��Ч����*/
	float pixValidLast[2];	/*��һ����Ч����*/
	
	float deltaPos[2];		/*2֮֡���λ�� ��λcm*/
	float deltaVel[2];		/*�ٶ� ��λcm/s*/
	float posSum[2];		/*�ۻ�λ�� ��λcm*/
	float velLpf[2];		/*�ٶȵ�ͨ ��λcm/s*/
	
	bool isOpFlowOk;		/*����״̬*/
	bool isDataValid;		/*������Ч*/

} opFlow_t;

extern opFlow_t opFlow;

void opticalFlowPowerControl(bool state);	//������Դ����
bool getOpFlowData(state_t *state, float dt);	//��ȡ��������
void opticalFlowInit(void);		/*��ʼ������ģ��*/
bool getOpDataState(void);		/*��������״̬*/

#endif
