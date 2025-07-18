#include "state_estimator.h"
#include "attitude_pid.h"
#include "position_pid.h"
#include "maths.h"
#include "vl53lxx.h"
#include "stabilizer.h"
#include "sensfusion6.h"
#include "optical_flow.h"

/********************************************************************************	 

********************************************************************************/

#define ACC_LIMIT			(1000.f)/*���ٶ��޷� ��λcm/s/s*/
#define ACC_LIMIT_MAX		(1800.f)/*�����ٶ��޷� ��λcm/s/s*/
#define VELOCITY_LIMIT		(130.f)	/*�ٶ��޷� ��λcm/s*/
#define VELOCITY_LIMIT_MAX	(500.f)	/*����ٶ��޷� ��λcm/s*/

#define GRAVITY_CMSS 		(980.f)	/*�������ٶ� ��λcm/s/s*/
#define INAV_ACC_BIAS_ACCEPTANCE_VALUE	(GRAVITY_CMSS * 0.25f)   // Max accepted bias correction of 0.25G - unlikely we are going to be that much off anyway


static float wBaro = 0.35f;			/*��ѹУ��Ȩ��*/
static float wOpflowP = 1.0f;		/*����λ��У��Ȩ��*/
static float wOpflowV = 2.0f;		/*�����ٶ�У��Ȩ��*/
static float wAccBias = 0.01f;		/*���ٶ�У��Ȩ��*/

static bool isRstHeight = false;	/*��λ�߶�*/
static bool isRstAll = true;		/*��λ����*/

static float fusedHeight;			/*�ںϸ߶ȣ���ɵ�Ϊ0*/
static float fusedHeightLpf = 0.f;	/*�ںϸ߶ȣ���ͨ*/
static float startBaroAsl = 0.f;	/*��ɵ㺣��*/


/*����ϵͳ*/
static estimator_t estimator = 
{
	.vAccDeadband = 4.0f,
	.accBias[0] =  0.0f,
	.accBias[1] =  0.0f,
	.accBias[2] =  0.0f,
	.acc[0] = 0.0f,
	.acc[1] = 0.0f,
	.acc[2] = 0.0f,
	.vel[0] = 0.0f,
	.vel[1] = 0.0f,
	.vel[2] = 0.0f,
	.pos[0] = 0.0f,
	.pos[1] = 0.0f,
	.pos[2] = 0.0f,
};

/* Inertial filter, implementation taken from PX4 implementation by Anton Babushkin <rk3dov@gmail.com> */
static void inavFilterPredict(int axis, float dt, float acc)
{
    estimator.pos[axis] += estimator.vel[axis] * dt + acc * dt * dt / 2.0f;
    estimator.vel[axis] += acc * dt;
}
/*λ��У��*/
static void inavFilterCorrectPos(int axis, float dt, float e, float w)
{
    float ewdt = e * w * dt;
    estimator.pos[axis] += ewdt;
    estimator.vel[axis] += w * ewdt;
}
/*�ٶ�У��*/
static void inavFilterCorrectVel(int axis, float dt, float e, float w)
{
   estimator.vel[axis] += e * w * dt;
}

void positionEstimate(sensorData_t* sensorData, state_t* state, float dt) 
{	
	static float rangeLpf = 0.f;
	static float accLpf[3] = {0.f};		/*���ٶȵ�ͨ*/	
	float weight = wBaro;

	float relateHight = sensorData->baro.asl - startBaroAsl;	/*��ѹ��Ը߶�*/
	
	if(getModuleID()==OPTICAL_FLOW && isEnableVl53lxx==true)	/*����ģ�����,��ʹ�ü���*/
	{
		vl53lxxReadRange(&sensorData->zrange);	/*��ȡ��������*/
	
//		rangeLpf = sensorData->zrange.distance;
		rangeLpf += (sensorData->zrange.distance - rangeLpf) * 0.1f;	/*��ͨ ��λcm*/		
			
		float quality = sensorData->zrange.quality;

		if(quality < 0.3f)	/*����������жȣ��������ݲ�����*/
		{
			quality = 0.f;
		}else
		{
			weight = quality;
			startBaroAsl = sensorData->baro.asl - rangeLpf;
		}
		fusedHeight = rangeLpf * quality + (1.0f - quality) * relateHight;/*�ںϸ߶�*/	
	}
	else	/*�޹���ģ��*/
	{
		fusedHeight = relateHight;	/*�ںϸ߶�*/
	}
	fusedHeightLpf += (fusedHeight - fusedHeightLpf) * 0.1f;	/*�ںϸ߶� ��ͨ*/
	
	if(isRstHeight)
	{	
		isRstHeight = false;
		
		weight = 0.95f;		/*����Ȩ�أ����ٵ���*/	
		
		startBaroAsl = sensorData->baro.asl;
		
		if(getModuleID() == OPTICAL_FLOW)
		{
			if(sensorData->zrange.distance < VL53L0X_MAX_RANGE)
			{
				startBaroAsl -= sensorData->zrange.distance;
				fusedHeight = sensorData->zrange.distance;
			}
		}
		
		estimator.pos[Z] = fusedHeight;
	}
	else if(isRstAll)
	{
		isRstAll = false;
		
		accLpf[Z] = 0.f;	
		fusedHeight  = 0.f;
		fusedHeightLpf = 0.f;
		startBaroAsl = sensorData->baro.asl;
		
		if(getModuleID() == OPTICAL_FLOW)
		{
			if(sensorData->zrange.distance < VL53L0X_MAX_RANGE)
			{
				startBaroAsl -= sensorData->zrange.distance;
				fusedHeight = sensorData->zrange.distance;
			}
		}
		
		estimator.vel[Z] = 0.f;
		estimator.pos[Z] = fusedHeight;
	}	
	
	
	Axis3f accelBF;
	
	accelBF.x = sensorData->acc.x * GRAVITY_CMSS - estimator.accBias[X];
	accelBF.y = sensorData->acc.y * GRAVITY_CMSS - estimator.accBias[Y];
	accelBF.z = sensorData->acc.z * GRAVITY_CMSS - estimator.accBias[Z];	
	
	/* Rotate vector to Earth frame - from Forward-Right-Down to North-East-Up*/
	imuTransformVectorBodyToEarth(&accelBF);	
	
	estimator.acc[X] = applyDeadbandf(accelBF.x, estimator.vAccDeadband);/*ȥ�������ļ��ٶ�*/
	estimator.acc[Y] = applyDeadbandf(accelBF.y, estimator.vAccDeadband);/*ȥ�������ļ��ٶ�*/
	estimator.acc[Z] = applyDeadbandf(accelBF.z, estimator.vAccDeadband);/*ȥ�������ļ��ٶ�*/
	
	for(u8 i=0; i<3; i++)
		accLpf[i] += (estimator.acc[i] - accLpf[i]) * 0.1f;	/*���ٶȵ�ͨ*/
		
	bool isKeyFlightLand = ((getCommanderKeyFlight()==true)||(getCommanderKeyland()==true));	/*���߷ɻ��߽���״̬*/
	
	if(isKeyFlightLand == true)		/*���߷ɻ��߽���״̬*/
	{
		state->acc.x = constrainf(accLpf[X], -ACC_LIMIT, ACC_LIMIT);	/*���ٶ��޷�*/
		state->acc.y = constrainf(accLpf[Y], -ACC_LIMIT, ACC_LIMIT);	/*���ٶ��޷�*/
		state->acc.z = constrainf(accLpf[Z], -ACC_LIMIT, ACC_LIMIT);	/*���ٶ��޷�*/
	}else
	{
		state->acc.x = constrainf(estimator.acc[X], -ACC_LIMIT_MAX, ACC_LIMIT_MAX);	/*�����ٶ��޷�*/
		state->acc.y = constrainf(estimator.acc[Y], -ACC_LIMIT_MAX, ACC_LIMIT_MAX);	/*�����ٶ��޷�*/
		state->acc.z = constrainf(estimator.acc[Z], -ACC_LIMIT_MAX, ACC_LIMIT_MAX);	/*�����ٶ��޷�*/
	}		

	
	float errPosZ = fusedHeight - estimator.pos[Z];
	
	/* λ��Ԥ��: Z-axis */
	inavFilterPredict(Z, dt, estimator.acc[Z]);
	/* λ��У��: Z-axis */
	inavFilterCorrectPos(Z, dt, errPosZ, weight);	

	if(getModuleID() == OPTICAL_FLOW)	/*����ģ�����*/
	{		
		float opflowDt = dt;
		
		float opResidualX = opFlow.posSum[X] - estimator.pos[X];
		float opResidualY = opFlow.posSum[Y] - estimator.pos[Y];
		float opResidualXVel = opFlow.velLpf[X] - estimator.vel[X];
		float opResidualYVel = opFlow.velLpf[Y] - estimator.vel[Y];
		
		float opWeightScaler = 1.0f;
		
		float wXYPos = wOpflowP * opWeightScaler;
		float wXYVel = wOpflowV * sq(opWeightScaler);
		
		/* λ��Ԥ��: XY-axis */
		inavFilterPredict(X, opflowDt, estimator.acc[X]);
		inavFilterPredict(Y, opflowDt, estimator.acc[Y]);
		/* λ��У��: XY-axis */
		inavFilterCorrectPos(X, opflowDt, opResidualX, wXYPos);
		inavFilterCorrectPos(Y, opflowDt, opResidualY, wXYPos);
		/* �ٶ�У��: XY-axis */
		inavFilterCorrectVel(X, opflowDt, opResidualXVel, wXYVel);
		inavFilterCorrectVel(Y, opflowDt, opResidualYVel, wXYVel);
	}
	
	/*���ٶ�ƫ��У��*/
	Axis3f accelBiasCorr = {{ 0, 0, 0}};
	
	accelBiasCorr.z -= errPosZ  * sq(wBaro);
	float accelBiasCorrMagnitudeSq = sq(accelBiasCorr.x) + sq(accelBiasCorr.y) + sq(accelBiasCorr.z);
	if (accelBiasCorrMagnitudeSq < sq(INAV_ACC_BIAS_ACCEPTANCE_VALUE)) 
	{
		/* transform error vector from NEU frame to body frame */
		imuTransformVectorEarthToBody(&accelBiasCorr);

		/* Correct accel bias */
		estimator.accBias[X] += accelBiasCorr.x * wAccBias * dt;
		estimator.accBias[Y] += accelBiasCorr.y * wAccBias * dt;
		estimator.accBias[Z] += accelBiasCorr.z * wAccBias * dt;
	}	

	if(isKeyFlightLand == true)		/*���߷ɻ��߽���״̬*/
	{
		state->velocity.x = constrainf(estimator.vel[X], -VELOCITY_LIMIT, VELOCITY_LIMIT);	/*�ٶ��޷� VELOCITY_LIMIT*/
		state->velocity.y = constrainf(estimator.vel[Y], -VELOCITY_LIMIT, VELOCITY_LIMIT);	/*�ٶ��޷� VELOCITY_LIMIT*/
		state->velocity.z = constrainf(estimator.vel[Z], -VELOCITY_LIMIT, VELOCITY_LIMIT);	/*�ٶ��޷� VELOCITY_LIMIT*/
	}else
	{
		state->velocity.x = constrainf(estimator.vel[X], -VELOCITY_LIMIT_MAX, VELOCITY_LIMIT_MAX);	/*����ٶ��޷� VELOCITY_LIMIT_MAX*/
		state->velocity.y = constrainf(estimator.vel[Y], -VELOCITY_LIMIT_MAX, VELOCITY_LIMIT_MAX);	/*����ٶ��޷� VELOCITY_LIMIT_MAX*/
		state->velocity.z = constrainf(estimator.vel[Z], -VELOCITY_LIMIT_MAX, VELOCITY_LIMIT_MAX);	/*����ٶ��޷� VELOCITY_LIMIT_MAX*/
	}
	
	state->position.x = estimator.pos[X];
	state->position.y = estimator.pos[Y];
	state->position.z = estimator.pos[Z];	
}

/*��ȡ�ںϸ߶� ��λcm*/	
float getFusedHeight(void)
{
	return fusedHeightLpf;
}

/*��λ����߶�*/
void estRstHeight(void)
{
	isRstHeight = true;
}

/*��λ���й���*/
void estRstAll(void)
{
	isRstAll = true;
}


