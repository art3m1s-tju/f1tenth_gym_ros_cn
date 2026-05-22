#ifndef PNC_RC_LQR_CORE_H_
#define PNC_RC_LQR_CORE_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct PncLqrResult {
  double steering;
  double feedback;
  double feedforward;
  double gain_y;
  double gain_psi;
} PncLqrResult;

double pnc_wrap_angle(double angle);

double pnc_compute_curvature_limited_speed(
    double target_speed,
    double curvature_ref,
    double delta_cmd,
    double wheelbase,
    double max_lateral_accel,
    double min_speed);

PncLqrResult pnc_compute_lqr_steering(
    double lateral_error,
    double heading_error,
    double curvature_ref,
    double speed,
    double wheelbase,
    double dt,
    double min_model_speed,
    double q_lateral,
    double q_heading,
    double r_steering,
    double feedforward_gain);

#ifdef __cplusplus
}
#endif

#endif
