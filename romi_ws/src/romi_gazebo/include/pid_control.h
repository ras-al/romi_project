#pragma once

#include <cstdint>

/**
 * PID Controller
 * 
 * Implements a proportional-integral-derivative controller with
 * anti-windup (integral clamping when output saturates).
 */
class PidControl {
public: 
  PidControl(const float kp, 
             const float ki, 
             const float kd);

  float getTarget() const { return target; };

  void setTarget(const float t);

  float process(const float obs, 
                const float dt, 
                const float min_range, 
                const float max_range);

private: 
  const float Kp;
  const float Ki;
  const float Kd;

  float target = 0.0f;
  float prev_err = 0.0f;
  float err_integral = 0.0f;
};

