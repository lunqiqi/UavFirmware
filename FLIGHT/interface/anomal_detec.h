#ifndef __ANOMAL_DETEC_H
#define __ANOMAL_DETEC_H
#include "stabilizer_types.h"

/********************************************************************************	 

********************************************************************************/


#define DETEC_ENABLED

#define DETEC_FF_THRESHOLD 	0.05f	/* accZ�ӽ�-1.0�̶� ��ʾFree Fall */
#define DETEC_FF_COUNT 		50  	/* ������������� 1000Hz�������� */

#define DETEC_TU_THRESHOLD 	60		/* ��ײ�����ֵ60��*/
#define DETEC_TU_COUNT 		100  	/* ��ײ������ 1000Hz�������� */

/*�쳣���*/
void anomalDetec(const sensorData_t *sensorData, const state_t *state, const control_t *control);

#endif	/*__ANOMAL_DETEC_H*/

