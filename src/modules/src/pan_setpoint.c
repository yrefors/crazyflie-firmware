/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 */

#include <math.h>

#include "pan_adaptive_outer_loop.h"
#include "pan_setpoint.h"

#ifdef CRAZYFLIE_FW

#include "commander.h"
#include "log.h"
#include "param.h"

static uint8_t panSetpointEnable = 0;
static uint8_t panSetpointAcceptHighLevel = 1;
static uint8_t panSetpointAcceptCrtp = 1;
static uint8_t panSetpointAcceptExtrx = 0;

static struct {
  float positionGain;
  float velocityGain;
  float massAdaptationGain;
  float alphaAdaptationGain;
  float referenceVelocityDotTimeConstant;
  float maxReferenceVelocityDot;
  float maxSpeed;
  float maxAccel;
} params = {
  .positionGain = 1.0f,
  .velocityGain = 1.2f,
  .massAdaptationGain = 0.15f,
  .alphaAdaptationGain = 0.25f,
  .referenceVelocityDotTimeConstant = 0.15f,
  .maxReferenceVelocityDot = 6.0f,
  .maxSpeed = 2.0f,
  .maxAccel = 4.0f,
};

static struct {
  pan_adaptive_outer_loop_state_t outerLoop;
} adaptiveState = {
  .outerLoop = {
    .hat_m = 1.0f,
    .hat_alpha = 0.0f,
    .last_reference_velocity = {0.0f, 0.0f, 0.0f},
    .last_reference_velocity_dot = {0.0f, 0.0f, 0.0f},
    .initialized = 0,
  },
};

static struct {
  float positionErrorNorm;
  float velocityErrorNorm;
  float referenceVelocityNorm;
  float referenceVelocityDotNorm;
  float accelCommandNorm;
  float hatM;
  float hatAlpha;
  uint8_t activePriority;
} metrics;

static float vectorNorm3(float x, float y, float z)
{
  return sqrtf((x * x) + (y * y) + (z * z));
}

static pan_vec3_t panVecFromCfVec(const struct vec3_s *value)
{
  return (pan_vec3_t){
    .x = value->x,
    .y = value->y,
    .z = value->z,
  };
}

static struct vec3_s cfVecFromPanVec(pan_vec3_t value)
{
  return (struct vec3_s){
    .x = value.x,
    .y = value.y,
    .z = value.z,
  };
}

static void clampVectorNorm(struct vec3_s *value, float limit)
{
  if (limit <= 0.0f) {
    value->x = 0.0f;
    value->y = 0.0f;
    value->z = 0.0f;
    return;
  }

  const float magnitude = vectorNorm3(value->x, value->y, value->z);
  if (magnitude <= limit || magnitude <= 1e-6f) {
    return;
  }

  const float scale = limit / magnitude;
  value->x *= scale;
  value->y *= scale;
  value->z *= scale;
}

static int shouldHandlePriority(int priority)
{
  switch (priority) {
    case COMMANDER_PRIORITY_HIGHLEVEL:
      return panSetpointAcceptHighLevel != 0;
    case COMMANDER_PRIORITY_CRTP:
      return panSetpointAcceptCrtp != 0;
    case COMMANDER_PRIORITY_EXTRX:
      return panSetpointAcceptExtrx != 0;
    default:
      return 0;
  }
}

void panSetpointInit(void)
{
}

bool panSetpointTest(void)
{
  return true;
}

void panSetpointUpdate(
  setpoint_t *setpoint,
  const sensorData_t *sensorData,
  const state_t *state,
  stabilizerStep_t stabilizerStep)
{
  (void)sensorData;

  if (!panSetpointEnable) {
    return;
  }

  const int activePriority = commanderGetActivePriority();
  metrics.activePriority = activePriority;
  if (!shouldHandlePriority(activePriority)) {
    return;
  }

  if (setpoint->mode.x != modeAbs || setpoint->mode.y != modeAbs || setpoint->mode.z != modeAbs) {
    return;
  }

  struct vec3_s positionError = {
    .x = setpoint->position.x - state->position.x,
    .y = setpoint->position.y - state->position.y,
    .z = setpoint->position.z - state->position.z,
  };
  struct vec3_s referenceVelocity = {
    .x = setpoint->velocity.x + (params.positionGain * positionError.x),
    .y = setpoint->velocity.y + (params.positionGain * positionError.y),
    .z = setpoint->velocity.z + (params.positionGain * positionError.z),
  };
  clampVectorNorm(&referenceVelocity, params.maxSpeed);

  const float dt = (stabilizerStep > 0U) ? (1.0f / 1000.0f) : 0.0f;
  pan_adaptive_outer_loop_config_t outerLoopConfig = {
    .velocity_gain = params.velocityGain,
    .mass_adaptation_gain = params.massAdaptationGain,
    .alpha_adaptation_gain = params.alphaAdaptationGain,
    .reference_velocity_dot_time_constant_s = params.referenceVelocityDotTimeConstant,
    .max_reference_velocity_dot = params.maxReferenceVelocityDot,
    .max_total_accel = params.maxAccel,
  };
  pan_adaptive_outer_loop_target_t outerLoopTarget = {
    .reference_velocity = panVecFromCfVec(&referenceVelocity),
    .feedforward_accel = (pan_vec3_t){0.0f, 0.0f, 0.0f},
    .tracking_accel_seed = panVecFromCfVec(&referenceVelocity),
  };
  pan_adaptive_outer_loop_output_t outerLoopOutput;
  panAdaptiveOuterLoopStep(
    &outerLoopConfig,
    &adaptiveState.outerLoop,
    panVecFromCfVec(&state->velocity),
    &outerLoopTarget,
    dt,
    &outerLoopOutput);

  setpoint->velocity = referenceVelocity;
  setpoint->acceleration = cfVecFromPanVec(outerLoopOutput.total_accel);

  metrics.positionErrorNorm = vectorNorm3(positionError.x, positionError.y, positionError.z);
  metrics.velocityErrorNorm = vectorNorm3(
    outerLoopOutput.velocity_error.x,
    outerLoopOutput.velocity_error.y,
    outerLoopOutput.velocity_error.z);
  metrics.referenceVelocityNorm = vectorNorm3(referenceVelocity.x, referenceVelocity.y, referenceVelocity.z);
  metrics.referenceVelocityDotNorm = vectorNorm3(
    outerLoopOutput.reference_velocity_dot.x,
    outerLoopOutput.reference_velocity_dot.y,
    outerLoopOutput.reference_velocity_dot.z);
  metrics.accelCommandNorm = vectorNorm3(
    outerLoopOutput.total_accel.x,
    outerLoopOutput.total_accel.y,
    outerLoopOutput.total_accel.z);
  metrics.hatM = adaptiveState.outerLoop.hat_m;
  metrics.hatAlpha = adaptiveState.outerLoop.hat_alpha;
}

LOG_GROUP_START(panSp)
  LOG_ADD(LOG_FLOAT, posErrNorm, &metrics.positionErrorNorm)
  LOG_ADD(LOG_FLOAT, velErrNorm, &metrics.velocityErrorNorm)
  LOG_ADD(LOG_FLOAT, refVelNorm, &metrics.referenceVelocityNorm)
  LOG_ADD(LOG_FLOAT, refVelDotNorm, &metrics.referenceVelocityDotNorm)
  LOG_ADD(LOG_FLOAT, accelNorm, &metrics.accelCommandNorm)
  LOG_ADD(LOG_FLOAT, hatM, &metrics.hatM)
  LOG_ADD(LOG_FLOAT, hatAlpha, &metrics.hatAlpha)
  LOG_ADD(LOG_UINT8, activePriority, &metrics.activePriority)
LOG_GROUP_STOP(panSp)

PARAM_GROUP_START(panSp)
  PARAM_ADD_CORE(PARAM_UINT8, enable, &panSetpointEnable)
  PARAM_ADD_CORE(PARAM_UINT8, acceptHl, &panSetpointAcceptHighLevel)
  PARAM_ADD_CORE(PARAM_UINT8, acceptCrtp, &panSetpointAcceptCrtp)
  PARAM_ADD_CORE(PARAM_UINT8, acceptExtrx, &panSetpointAcceptExtrx)
  PARAM_ADD(PARAM_FLOAT, posGain, &params.positionGain)
  PARAM_ADD(PARAM_FLOAT, velGain, &params.velocityGain)
  PARAM_ADD(PARAM_FLOAT, massGain, &params.massAdaptationGain)
  PARAM_ADD(PARAM_FLOAT, alphaGain, &params.alphaAdaptationGain)
  PARAM_ADD(PARAM_FLOAT, refVelDtTau, &params.referenceVelocityDotTimeConstant)
  PARAM_ADD(PARAM_FLOAT, maxRefVelDot, &params.maxReferenceVelocityDot)
  PARAM_ADD(PARAM_FLOAT, maxSpeed, &params.maxSpeed)
  PARAM_ADD(PARAM_FLOAT, maxAccel, &params.maxAccel)
PARAM_GROUP_STOP(panSp)

#endif