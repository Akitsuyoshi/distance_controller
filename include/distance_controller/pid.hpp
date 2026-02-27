#ifndef DISTANCE_CONTROLLER__PID_HPP_
#define DISTANCE_CONTROLLER__PID_HPP_

#include <algorithm>

namespace distance_controller {

/**
 * Discrete-time PID controller
 *
 * Features:
 *  - Configurable gains (kp, ki, kd)
 *  - Integral windup protection
 *  - Backward difference derivative
 *  - Clamp outputs to maximum output limits
 */
class PID {
public:
  PID() = default;

  PID(double p, double i, double d) : kp_(p), ki_(i), kd_(d) {}

  void set_gain(double p, double i, double d) {
    kp_ = p;
    ki_ = i;
    kd_ = d;
  }

  void set_limit(double out_limit, double integ_limit) {
    out_limit_ = out_limit;
    integ_limit_ = integ_limit;
  }

  void reset() {
    integral_ = 0.0;
    prev_err_ = 0.0;
  }

  double compute(double err, double dt, double feed_forward_gain = 0.0) {
    if (dt <= 0.0) {
      return 0.0;
    } else {
      // Clamp dt to prevent instability from large timing gaps
      dt = std::clamp(dt, 0.001, 0.1);
    }
    // Proportional
    double p_term = kp_ * err;
    // Integral
    integral_ += err * dt;
    integral_ = std::clamp(integral_, -integ_limit_, integ_limit_);
    double i_term = ki_ * integral_;
    // Derivative
    double deriv = (err - prev_err_) / dt;
    double d_term = kd_ * deriv;
    prev_err_ = err;
    // PID output
    double output = p_term + i_term + d_term;
    output += feed_forward_gain * err;

    return std::clamp(output, -out_limit_, out_limit_);
  }

private:
  // Gains
  double kp_{0.0};
  double ki_{0.0};
  double kd_{0.0};

  // Errors
  double integral_{0.0};
  double prev_err_{0.0};

  // Limits
  double integ_limit_{0.0}; // to prevent integral explosion
  double out_limit_{0.0};   // to limit PID output
};

} // namespace distance_controller

#endif // DISTANCE_CONTROLLER__PID_HPP_