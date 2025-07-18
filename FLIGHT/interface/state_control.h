#ifndef __STATE_CONTROL_H
#define __STATE_CONTROL_H
#include "stabilizer_types.h"

/********************************************************************************	 

********************************************************************************/

//#define ENABLE_PID_TUNING	/* ʹ��PID���� yawֵ������ */

void stateControlInit(void);
bool stateControlTest(void);
void stateControl(control_t *control, sensorData_t *sensors, state_t *state, setpoint_t *setpoint, const u32 tick);

#endif /*__STATE_CONTROL_H */

