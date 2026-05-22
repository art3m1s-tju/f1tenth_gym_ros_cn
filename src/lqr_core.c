#include "pnc_rc/lqr_core.h"

#include <math.h>

static double pnc_max(double a, double b) { return a > b ? a : b; }
static double pnc_min(double a, double b) { return a < b ? a : b; }

static double pnc_clamp(double value, double lower, double upper) {
  return pnc_min(pnc_max(value, lower), upper);
}

double pnc_wrap_angle(double angle) {
  return atan2(sin(angle), cos(angle));
}

double pnc_compute_curvature_limited_speed(
    double target_speed,
    double curvature_ref,
    double delta_cmd,
    double wheelbase,
    double max_lateral_accel,
    double min_speed) {
  const double safe_wheelbase = pnc_max(wheelbase, 1e-6);
  const double steering_curvature = fabs(tan(delta_cmd)) / safe_wheelbase;
  const double curvature =
      pnc_max(pnc_max(fabs(curvature_ref), steering_curvature), 1e-6);
  const double curve_speed = sqrt(max_lateral_accel / curvature);
  return pnc_clamp(pnc_min(target_speed, curve_speed), min_speed, target_speed);
}

static void pnc_solve_lqr_gain(
    double speed,
    double wheelbase,
    double dt,
    double min_model_speed,
    double q_lateral,
    double q_heading,
    double r_steering,
    double *gain_y,
    double *gain_psi) {
  const double model_speed = pnc_max(fabs(speed), min_model_speed);
  const double safe_wheelbase = pnc_max(wheelbase, 1e-6);
  const double safe_dt = pnc_max(dt, 1e-4);

  const double a01 = safe_dt * model_speed;
  const double b1 = safe_dt * model_speed / safe_wheelbase;
  const double q0 = pnc_max(q_lateral, 1e-9);
  const double q1 = pnc_max(q_heading, 1e-9);
  const double r = pnc_max(r_steering, 1e-9);

  double p00 = q0;
  double p01 = 0.0;
  double p11 = q1;

  for (int iter = 0; iter < 200; ++iter) {
    const double pb0 = b1 * p01;
    const double pb1 = b1 * p11;
    const double s = r + b1 * pb1;

    const double bpa0 = pb0;
    const double bpa1 = pb0 * a01 + pb1;
    const double k0 = bpa0 / s;
    const double k1 = bpa1 / s;

    const double apa00 = p00;
    const double apa01 = p00 * a01 + p01;
    const double apa11 = p00 * a01 * a01 + 2.0 * p01 * a01 + p11;

    const double n00 = q0 + apa00 - bpa0 * k0;
    const double n01 = apa01 - bpa0 * k1;
    const double n11 = q1 + apa11 - bpa1 * k1;

    const double diff =
        fabs(n00 - p00) + fabs(n01 - p01) + fabs(n11 - p11);
    p00 = n00;
    p01 = n01;
    p11 = n11;
    if (diff < 1e-10) {
      break;
    }
  }

  const double pb0 = b1 * p01;
  const double pb1 = b1 * p11;
  const double s = r + b1 * pb1;
  *gain_y = pb0 / s;
  *gain_psi = (pb0 * a01 + pb1) / s;
}

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
    double feedforward_gain) {
  PncLqrResult result;
  pnc_solve_lqr_gain(
      speed,
      wheelbase,
      dt,
      min_model_speed,
      q_lateral,
      q_heading,
      r_steering,
      &result.gain_y,
      &result.gain_psi);

  result.feedback =
      -(result.gain_y * lateral_error + result.gain_psi * heading_error);
  result.feedforward = feedforward_gain * atan(wheelbase * curvature_ref);
  result.steering = result.feedback + result.feedforward;
  return result;
}
