#include "pid_control.h"

PidControl::PidControl(const float kp, const float ki, const float kd)
  : Kp(kp), 
    Ki(ki), 
    Kd(kd) {};

void PidControl::setTarget(const float t) { 
  target = t; 
}

float PidControl::process(const float obs, const float dt, const float min_range, const float max_range) {
  const float err = target - obs; 
  const float der_err = (err - prev_err) / dt;
  prev_err = err; 
  err_integral += (err * dt); 
  float out = (Kp * err) + (Ki * err_integral) + (Kd * der_err);
  
  // Anti-windup: prevent integral from growing when output is saturated
  if (out < min_range) {
    out = min_range;
    err_integral -= (err * dt);
  } else if (out > max_range) {
    out = max_range;
    err_integral -= (err * dt);
  }
  
  return out; 
}

