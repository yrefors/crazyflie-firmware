#ifndef __PAN_SETPOINT_H__
#define __PAN_SETPOINT_H__

#include "stabilizer_types.h"

#ifdef CRAZYFLIE_FW

void panSetpointInit(void);
bool panSetpointTest(void);
void panSetpointUpdate(
  setpoint_t *setpoint,
  const sensorData_t *sensorData,
  const state_t *state,
  stabilizerStep_t stabilizerStep);

#endif

#endif