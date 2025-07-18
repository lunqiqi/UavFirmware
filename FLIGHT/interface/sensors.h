#ifndef __SENSORS_H
#define __SENSORS_H
#include "stabilizer_types.h"

/********************************************************************************	 

********************************************************************************/

//#define SENSORS_ENABLE_MAG_AK8963
#define SENSORS_ENABLE_PRESSURE_BMP280	/*��ѹ��ʹ��bmp280*/

#define BARO_UPDATE_RATE		RATE_50_HZ
#define SENSOR9_UPDATE_RATE   	RATE_500_HZ
#define SENSOR9_UPDATE_DT     	(1.0f/SENSOR9_UPDATE_RATE)

	
void sensorsTask(void *param);
void sensorsInit(void);			/*��������ʼ��*/
bool sensorsTest(void);			/*����������*/
bool sensorsAreCalibrated(void);	/*����������У׼*/
void sensorsAcquire(sensorData_t *sensors, const u32 tick);/*��ȡ����������*/
void getSensorRawData(Axis3i16* acc, Axis3i16* gyro, Axis3i16* mag);
bool getIsMPU9250Present(void);
bool getIsBaroPresent(void);

/* ������������������ */
bool sensorsReadGyro(Axis3f *gyro);
bool sensorsReadAcc(Axis3f *acc);
bool sensorsReadMag(Axis3f *mag);
bool sensorsReadBaro(baro_t *baro);

#endif //__SENSORS_H
