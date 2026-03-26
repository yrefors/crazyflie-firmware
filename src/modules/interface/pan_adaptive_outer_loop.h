#ifndef PAN_ADAPTIVE_OUTER_LOOP_H
#define PAN_ADAPTIVE_OUTER_LOOP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct pan_vec3_s {
  float x;
  float y;
  float z;
} pan_vec3_t;

typedef struct pan_adaptive_outer_loop_config_s {
  float velocity_gain;
  float mass_adaptation_gain;
  float alpha_adaptation_gain;
  float reference_velocity_dot_time_constant_s;
  float max_reference_velocity_dot;
  float max_total_accel;
} pan_adaptive_outer_loop_config_t;

typedef struct pan_adaptive_outer_loop_target_s {
  pan_vec3_t reference_velocity;
  pan_vec3_t feedforward_accel;
  pan_vec3_t tracking_accel_seed;
} pan_adaptive_outer_loop_target_t;

typedef struct pan_adaptive_outer_loop_state_s {
  float hat_m;
  float hat_alpha;
  pan_vec3_t last_reference_velocity;
  pan_vec3_t last_reference_velocity_dot;
  uint8_t initialized;
} pan_adaptive_outer_loop_state_t;

typedef struct pan_adaptive_outer_loop_output_s {
  pan_vec3_t reference_velocity_dot;
  pan_vec3_t velocity_error;
  pan_vec3_t feedforward_accel;
  pan_vec3_t tracking_accel;
  pan_vec3_t total_accel;
  float unclipped_total_accel_norm;
  float total_accel_clip_fraction;
} pan_adaptive_outer_loop_output_t;

void panAdaptiveOuterLoopReset(pan_adaptive_outer_loop_state_t *state);

void panAdaptiveOuterLoopStep(
  const pan_adaptive_outer_loop_config_t *config,
  pan_adaptive_outer_loop_state_t *state,
  pan_vec3_t measured_velocity,
  const pan_adaptive_outer_loop_target_t *target,
  float dt,
  pan_adaptive_outer_loop_output_t *output);

#ifdef __cplusplus
}
#endif

#endif