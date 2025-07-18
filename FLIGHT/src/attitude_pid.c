#include <stdbool.h>
#include "pid.h"
#include "sensors.h"
#include "attitude_pid.h"

/********************************************************************************	 

********************************************************************************/

/*�ǶȻ������޷�*/
#define PID_ANGLE_ROLL_INTEGRATION_LIMIT    30.0
#define PID_ANGLE_PITCH_INTEGRATION_LIMIT   30.0
#define PID_ANGLE_YAW_INTEGRATION_LIMIT     180.0

/*���ٶȻ������޷�*/
#define PID_RATE_ROLL_INTEGRATION_LIMIT		500.0
#define PID_RATE_PITCH_INTEGRATION_LIMIT	500.0
#define PID_RATE_YAW_INTEGRATION_LIMIT		50.0

PidObject pidAngleRoll;
PidObject pidAnglePitch;
PidObject pidAngleYaw;
PidObject pidRateRoll;
PidObject pidRatePitch;
PidObject pidRateYaw;

static inline int16_t pidOutLimit(float in)
{
	if (in > INT16_MAX)
		return INT16_MAX;
	else if (in < -INT16_MAX)
		return -INT16_MAX;
	else
		return (int16_t)in;
}

void attitudeControlInit(float ratePidDt, float anglePidDt)
{
	pidInit(&pidAngleRoll, 0, configParam.pidAngle.roll, anglePidDt);			/*roll  �Ƕ�PID��ʼ��*/
	pidInit(&pidAnglePitch, 0, configParam.pidAngle.pitch, anglePidDt);			/*pitch �Ƕ�PID��ʼ��*/
	pidInit(&pidAngleYaw, 0, configParam.pidAngle.yaw, anglePidDt);				/*yaw   �Ƕ�PID��ʼ��*/
	pidSetIntegralLimit(&pidAngleRoll, PID_ANGLE_ROLL_INTEGRATION_LIMIT);		/*roll  �ǶȻ����޷�����*/
	pidSetIntegralLimit(&pidAnglePitch, PID_ANGLE_PITCH_INTEGRATION_LIMIT);		/*pitch �ǶȻ����޷�����*/
	pidSetIntegralLimit(&pidAngleYaw, PID_ANGLE_YAW_INTEGRATION_LIMIT);			/*yaw   �ǶȻ����޷�����*/
	
	pidInit(&pidRateRoll, 0, configParam.pidRate.roll, ratePidDt);				/*roll  ���ٶ�PID��ʼ��*/
	pidInit(&pidRatePitch, 0, configParam.pidRate.pitch, ratePidDt);			/*pitch ���ٶ�PID��ʼ��*/
	pidInit(&pidRateYaw, 0, configParam.pidRate.yaw, ratePidDt);				/*yaw   ���ٶ�PID��ʼ��*/
	pidSetIntegralLimit(&pidRateRoll, PID_RATE_ROLL_INTEGRATION_LIMIT);			/*roll  ���ٶȻ����޷�����*/
	pidSetIntegralLimit(&pidRatePitch, PID_RATE_PITCH_INTEGRATION_LIMIT);		/*pitch ���ٶȻ����޷�����*/
	pidSetIntegralLimit(&pidRateYaw, PID_RATE_YAW_INTEGRATION_LIMIT);			/*yaw   ���ٶȻ����޷�����*/
}

bool attitudeControlTest()
{
	return true;
}

void attitudeRatePID(Axis3f *actualRate,attitude_t *desiredRate,control_t *output)	/* ���ٶȻ�PID */
{
	output->roll = pidOutLimit(pidUpdate(&pidRateRoll, desiredRate->roll - actualRate->x));
	output->pitch = pidOutLimit(pidUpdate(&pidRatePitch, desiredRate->pitch - actualRate->y));
	output->yaw = pidOutLimit(pidUpdate(&pidRateYaw, desiredRate->yaw - actualRate->z));
}

void attitudeAnglePID(attitude_t *actualAngle,attitude_t *desiredAngle,attitude_t *outDesiredRate)	/* �ǶȻ�PID */
{
	outDesiredRate->roll = pidUpdate(&pidAngleRoll, desiredAngle->roll - actualAngle->roll);
	outDesiredRate->pitch = pidUpdate(&pidAnglePitch, desiredAngle->pitch - actualAngle->pitch);

	float yawError = desiredAngle->yaw - actualAngle->yaw ;
	if (yawError > 180.0f) 
		yawError -= 360.0f;
	else if (yawError < -180.0) 
		yawError += 360.0f;
	outDesiredRate->yaw = pidUpdate(&pidAngleYaw, yawError);
}

void attitudeControllerResetRollAttitudePID(void)
{
    pidReset(&pidAngleRoll);
}

void attitudeControllerResetPitchAttitudePID(void)
{
    pidReset(&pidAnglePitch);
}

void attitudeResetAllPID(void)	/*��λPID*/
{
	pidReset(&pidAngleRoll);
	pidReset(&pidAnglePitch);
	pidReset(&pidAngleYaw);
	pidReset(&pidRateRoll);
	pidReset(&pidRatePitch);
	pidReset(&pidRateYaw);
}

void attitudePIDwriteToConfigParam(void)
{
	configParam.pidAngle.roll.kp = pidAngleRoll.kp;
	configParam.pidAngle.roll.ki = pidAngleRoll.ki;
	configParam.pidAngle.roll.kd = pidAngleRoll.kd;
	
	configParam.pidAngle.pitch.kp = pidAnglePitch.kp;
	configParam.pidAngle.pitch.ki = pidAnglePitch.ki;
	configParam.pidAngle.pitch.kd = pidAnglePitch.kd;
	
	configParam.pidAngle.yaw.kp = pidAngleYaw.kp;
	configParam.pidAngle.yaw.ki = pidAngleYaw.ki;
	configParam.pidAngle.yaw.kd = pidAngleYaw.kd;
	
	configParam.pidRate.roll.kp = pidRateRoll.kp;
	configParam.pidRate.roll.ki = pidRateRoll.ki;
	configParam.pidRate.roll.kd = pidRateRoll.kd;
	
	configParam.pidRate.pitch.kp = pidRatePitch.kp;
	configParam.pidRate.pitch.ki = pidRatePitch.ki;
	configParam.pidRate.pitch.kd = pidRatePitch.kd;
	
	configParam.pidRate.yaw.kp = pidRateYaw.kp;
	configParam.pidRate.yaw.ki = pidRateYaw.ki;
	configParam.pidRate.yaw.kd = pidRateYaw.kd;
}

