#pragma once

#include <cstdint>
#include <random>
#include "pid_control.h"

/**
 * Wheel with Encoder Simulation
 * 
 * Simulates a motor with encoder using:
 * - PID velocity controller
 * - Second-order motor dynamics model
 * - Gaussian noise for encoder readings
 */
class WheelWithEncoder {
public:
  WheelWithEncoder(const float kp, const float ki, const float kd)
    : pid(kp, ki, kd) {}

  void setTarget(const int t) {
    pid.setTarget(static_cast<float>(t));
  }
  
  int getTarget() const {
    return static_cast<int>(pid.getTarget());
  }
  
  float update() {
    // Use internal previous velocity (for backward compatibility)
    return update(prev_vel_1);
  }

  float update(float actual_velocity) {
    // PID control output (PWM command)
    // Compares target (from setTarget) vs actual (from physics/feedback)
    // Error = target - actual, so we pass actual as observation
    const float pwm = pid.process(actual_velocity, 0.025, -250.0f, 250.0f);
    
    // Second-order motor dynamics model
    // Uses previous velocity state for motor dynamics
    const float curr_vel = 0.95f * prev_vel_1 - 0.239f * prev_vel_2 + 0.1232f * prev_pwm_1;
    
    prev_pwm_1 = pwm;
    prev_vel_2 = prev_vel_1;

    // Do not add artificial integer noise, as this directly controls the physical Gazebo joints 
    // and causes severe robotic shaking. Return the smooth dynamics velocity.
    prev_vel_1 = curr_vel;
    return prev_vel_1;
  }

private:
  float prev_pwm_1 = 0.0f;
  float prev_vel_1 = 0.0f;
  float prev_vel_2 = 0.0f;

  PidControl pid;
};

