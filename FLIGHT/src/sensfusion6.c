#include <math.h>
#include "sensfusion6.h"
#include "config.h"
#include "ledseq.h"
#include "maths.h"

/********************************************************************************	 


#define ACCZ_SAMPLE		350

float Kp = 0.4f;		/*��������*/
float Ki = 0.001f;		/*��������*/
float exInt = 0.0f;
float eyInt = 0.0f;
float ezInt = 0.0f;		/*��������ۼ�*/

static float q0 = 1.0f;	/*��Ԫ��*/
static float q1 = 0.0f;
static float q2 = 0.0f;
static float q3 = 0.0f;	
static float rMat[3][3];/*��ת����*/

static float maxError = 0.f;		/*������*/
bool isGravityCalibrated = false;	/*�Ƿ�УУ׼���*/
static float baseAcc[3] = {0.f,0.f,1.0f};	/*��̬���ٶ�*/


static float invSqrt(float x);	/*���ٿ�ƽ����*/

static void calBaseAcc(float* acc)	/*���㾲̬���ٶ�*/
{
	static u16 cnt = 0;
	static float accZMin = 1.5;
	static float accZMax = 0.5;
	static float sumAcc[3] = {0.f};
	
	for(u8 i=0; i<3; i++)
		sumAcc[i] += acc[i];
		
	if(acc[2] < accZMin)	accZMin = acc[2];
	if(acc[2] > accZMax)	accZMax = acc[2];
	
	if(++cnt >= ACCZ_SAMPLE) /*��������*/
	{
		cnt = 0;
		maxError = accZMax - accZMin;
		accZMin = 1.5;
		accZMax = 0.5;
		
		if(maxError < 0.015f)
		{
			for(u8 i=0; i<3; i++)
				baseAcc[i] = sumAcc[i] / ACCZ_SAMPLE;
			
			isGravityCalibrated = true;
			
			ledseqRun(SYS_LED, seq_calibrated);	/*У׼ͨ��ָʾ��*/
		}
		
		for(u8 i=0; i<3; i++)		
			sumAcc[i] = 0.f;		
	}	
}

/*������ת����*/
void imuComputeRotationMatrix(void)
{
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q3 = q2 * q3;

    rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    rMat[0][1] = 2.0f * (q1q2 + -q0q3);
    rMat[0][2] = 2.0f * (q1q3 - -q0q2);

    rMat[1][0] = 2.0f * (q1q2 - -q0q3);
    rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    rMat[1][2] = 2.0f * (q2q3 + -q0q1);

    rMat[2][0] = 2.0f * (q1q3 + -q0q2);
    rMat[2][1] = 2.0f * (q2q3 - -q0q1);
    rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}

void imuUpdate(Axis3f acc, Axis3f gyro, state_t *state , float dt)	/*�����ں� �����˲�*/
{
	float normalise;
	float ex, ey, ez;
	float halfT = 0.5f * dt;
	float accBuf[3] = {0.f};
	Axis3f tempacc = acc;
	
	gyro.x = gyro.x * DEG2RAD;	/* ��ת���� */
	gyro.y = gyro.y * DEG2RAD;
	gyro.z = gyro.z * DEG2RAD;

	/* ���ٶȼ������Чʱ,���ü��ٶȼƲ���������*/
	if((acc.x != 0.0f) || (acc.y != 0.0f) || (acc.z != 0.0f))
	{
		/*��λ�����ټƲ���ֵ*/
		normalise = invSqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
		acc.x *= normalise;
		acc.y *= normalise;
		acc.z *= normalise;

		/*���ټƶ�ȡ�ķ������������ټƷ���Ĳ�ֵ����������˼���*/
		ex = (acc.y * rMat[2][2] - acc.z * rMat[2][1]);
		ey = (acc.z * rMat[2][0] - acc.x * rMat[2][2]);
		ez = (acc.x * rMat[2][1] - acc.y * rMat[2][0]);
		
		/*����ۼƣ�����ֳ������*/
		exInt += Ki * ex * dt ;  
		eyInt += Ki * ey * dt ;
		ezInt += Ki * ez * dt ;
		
		/*�ò���������PI����������ƫ�����������ݶ����е�ƫ����*/
		gyro.x += Kp * ex + exInt;
		gyro.y += Kp * ey + eyInt;
		gyro.z += Kp * ez + ezInt;
	}
	/* һ�׽����㷨����Ԫ���˶�ѧ���̵���ɢ����ʽ�ͻ��� */
	float q0Last = q0;
	float q1Last = q1;
	float q2Last = q2;
	float q3Last = q3;
	q0 += (-q1Last * gyro.x - q2Last * gyro.y - q3Last * gyro.z) * halfT;
	q1 += ( q0Last * gyro.x + q2Last * gyro.z - q3Last * gyro.y) * halfT;
	q2 += ( q0Last * gyro.y - q1Last * gyro.z + q3Last * gyro.x) * halfT;
	q3 += ( q0Last * gyro.z + q1Last * gyro.y - q2Last * gyro.x) * halfT;
	
	/*��λ����Ԫ��*/
	normalise = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= normalise;
	q1 *= normalise;
	q2 *= normalise;
	q3 *= normalise;
	
	imuComputeRotationMatrix();	/*������ת����*/
	
	/*����roll pitch yaw ŷ����*/
	state->attitude.pitch = -asinf(rMat[2][0]) * RAD2DEG; 
	state->attitude.roll = atan2f(rMat[2][1], rMat[2][2]) * RAD2DEG;
	state->attitude.yaw = atan2f(rMat[1][0], rMat[0][0]) * RAD2DEG;
	
	if (!isGravityCalibrated)	/*δУ׼*/
	{		
//		accBuf[0] = tempacc.x* rMat[0][0] + tempacc.y * rMat[0][1] + tempacc.z * rMat[0][2];	/*accx*/
//		accBuf[1] = tempacc.x* rMat[1][0] + tempacc.y * rMat[1][1] + tempacc.z * rMat[1][2];	/*accy*/
		accBuf[2] = tempacc.x* rMat[2][0] + tempacc.y * rMat[2][1] + tempacc.z * rMat[2][2];	/*accz*/
		calBaseAcc(accBuf);		/*���㾲̬���ٶ�*/				
	}
}

/*���嵽����*/
void imuTransformVectorBodyToEarth(Axis3f * v)
{
    /* From body frame to earth frame */
    const float x = rMat[0][0] * v->x + rMat[0][1] * v->y + rMat[0][2] * v->z;
    const float y = rMat[1][0] * v->x + rMat[1][1] * v->y + rMat[1][2] * v->z;
    const float z = rMat[2][0] * v->x + rMat[2][1] * v->y + rMat[2][2] * v->z;

	float yawRad = atan2f(rMat[1][0], rMat[0][0]);
	float cosy = cosf(yawRad);
	float siny = sinf(yawRad);
	float vx = x * cosy + y * siny;
	float vy = y * cosy - x * siny;	
	
    v->x = vx;
    v->y = -vy;
    v->z = z - baseAcc[2] *  980.f;	/*ȥ���������ٶ�*/
}

/*���򵽻���*/
void imuTransformVectorEarthToBody(Axis3f * v)
{
    v->y = -v->y;

    /* From earth frame to body frame */
    const float x = rMat[0][0] * v->x + rMat[1][0] * v->y + rMat[2][0] * v->z;
    const float y = rMat[0][1] * v->x + rMat[1][1] * v->y + rMat[2][1] * v->z;
    const float z = rMat[0][2] * v->x + rMat[1][2] * v->y + rMat[2][2] * v->z;

    v->x= x;
    v->y = y;
    v->z = z;
}

// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float invSqrt(float x)	/*���ٿ�ƽ����*/
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

bool getIsCalibrated(void)
{
	return isGravityCalibrated;
}

