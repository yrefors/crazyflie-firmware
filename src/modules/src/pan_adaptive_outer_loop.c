#include "pan_adaptive_outer_loop.h"

#include <math.h>
#include <stddef.h>

static float panVectorNorm3(pan_vec3_t value)
{
  return sqrtf((value.x * value.x) + (value.y * value.y) + (value.z * value.z));
}

static float panClampf(float value, float minValue, float maxValue)
{
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}

static float panDot3(pan_vec3_t left, pan_vec3_t right)
{
  return (left.x * right.x) + (left.y * right.y) + (left.z * right.z);
}

static pan_vec3_t panVecAdd(pan_vec3_t left, pan_vec3_t right)
{
  return (pan_vec3_t){
    .x = left.x + right.x,
    .y = left.y + right.y,
    .z = left.z + right.z,
  };
}

static pan_vec3_t panVecSub(pan_vec3_t left, pan_vec3_t right)
{
  return (pan_vec3_t){
    .x = left.x - right.x,
    .y = left.y - right.y,
    .z = left.z - right.z,
  };
}

static pan_vec3_t panVecScale(pan_vec3_t value, float scalar)
{
  return (pan_vec3_t){
    .x = value.x * scalar,
    .y = value.y * scalar,
    .z = value.z * scalar,
  };
}

static void panClampVectorNorm(pan_vec3_t *value, float limit)
{
  if (value == NULL) {
    return;
  }

  if (limit <= 0.0f) {
    *value = (pan_vec3_t){0.0f, 0.0f, 0.0f};
    return;
  }

  const float magnitude = panVectorNorm3(*value);
  if (magnitude <= limit || magnitude <= 1e-6f) {
    return;
  }

  const float scale = limit / magnitude;
  *value = panVecScale(*value, scale);
}

void panAdaptiveOuterLoopReset(pan_adaptive_outer_loop_state_t *state)
{
  if (state == NULL) {
    return;
  }

  state->hat_m = 1.0f;
  state->hat_alpha = 0.0f;
  state->last_reference_velocity = (pan_vec3_t){0.0f, 0.0f, 0.0f};
  state->last_reference_velocity_dot = (pan_vec3_t){0.0f, 0.0f, 0.0f};
  state->initialized = 0;
}

void panAdaptiveOuterLoopStep(
  const pan_adaptive_outer_loop_config_t *config,
  pan_adaptive_outer_loop_state_t *state,
  pan_vec3_t measured_velocity,
  const pan_adaptive_outer_loop_target_t *target,
  float dt,
  pan_adaptive_outer_loop_output_t *output)
{
  if (config == NULL || state == NULL || target == NULL || output == NULL) {
    return;
  }

  if (!state->initialized) {
    state->hat_m = 1.0f;
    state->hat_alpha = 0.0f;
    state->last_reference_velocity = target->reference_velocity;
    state->last_reference_velocity_dot = (pan_vec3_t){0.0f, 0.0f, 0.0f};
    state->initialized = 1;
  }

  output->velocity_error = panVecSub(measured_velocity, target->reference_velocity);
  output->reference_velocity_dot = (pan_vec3_t){0.0f, 0.0f, 0.0f};

  if (dt > 1e-6f) {
    pan_vec3_t rawReferenceVelocityDot = panVecScale(
      panVecSub(target->reference_velocity, state->last_reference_velocity),
      1.0f / dt);
    panClampVectorNorm(&rawReferenceVelocityDot, config->max_reference_velocity_dot);

    const float timeConstant = fmaxf(config->reference_velocity_dot_time_constant_s, 0.0f);
    const float filterGain =
      (timeConstant <= 1e-6f) ? 1.0f : panClampf(dt / (timeConstant + dt), 0.0f, 1.0f);
    output->reference_velocity_dot = panVecAdd(
      state->last_reference_velocity_dot,
      panVecScale(
        panVecSub(rawReferenceVelocityDot, state->last_reference_velocity_dot),
        filterGain));

    state->hat_m = fmaxf(
      0.1f,
      state->hat_m -
        (dt * config->mass_adaptation_gain *
          (panDot3(output->velocity_error, output->reference_velocity_dot) + state->hat_m)));
    state->hat_alpha = fmaxf(
      0.0f,
      state->hat_alpha +
        (dt * config->alpha_adaptation_gain *
          (panDot3(output->velocity_error, output->velocity_error) - state->hat_alpha)));
  }

  output->feedforward_accel = panVecAdd(
    target->feedforward_accel,
    panVecScale(output->reference_velocity_dot, state->hat_m));
  output->tracking_accel = panVecAdd(
    target->tracking_accel_seed,
    panVecScale(output->velocity_error, -(config->velocity_gain + state->hat_alpha)));

  output->total_accel = panVecAdd(output->feedforward_accel, output->tracking_accel);
  output->unclipped_total_accel_norm = panVectorNorm3(output->total_accel);
  panClampVectorNorm(&output->total_accel, config->max_total_accel);

  const float clippedNorm = panVectorNorm3(output->total_accel);
  output->total_accel_clip_fraction = 1.0f;
  if (output->unclipped_total_accel_norm > 1e-9f) {
    output->total_accel_clip_fraction = clippedNorm / output->unclipped_total_accel_norm;
  }

  state->last_reference_velocity = target->reference_velocity;
  state->last_reference_velocity_dot = output->reference_velocity_dot;
}