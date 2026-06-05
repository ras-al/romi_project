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
  
  int update() {
    // Use internal previous velocity (for backward compatibility)
    return update(prev_vel_1);
  }

  int update(float actual_velocity) {
    // PID control output (PWM command)
    // Compares target (from setTarget) vs actual (from physics/feedback)
    // Error = target - actual, so we pass actual as observation
    const float pwm = pid.process(actual_velocity, 0.025, -250.0f, 250.0f);
    
    // Second-order motor dynamics model
    // Uses previous velocity state for motor dynamics
    const float curr_vel = 0.95f * prev_vel_1 - 0.239f * prev_vel_2 + 0.1232f * prev_pwm_1;
    
    prev_pwm_1 = pwm;
    prev_vel_2 = prev_vel_1;

    // Add Gaussian noise to simulate encoder noise
    std::random_device rd{};
    std::mt19937 gen{ rd() };
    
    // Only add noise when motor is running
    float noise_std = (pid.getTarget() == 0.0f) ? 0.0f : 2.0f;
    std::normal_distribution<float> dist(curr_vel, noise_std);
    
    prev_vel_1 = std::ceil(dist(gen));
    return static_cast<int>(prev_vel_1);
  }

private:
  float prev_pwm_1 = 0.0f;
  float prev_vel_1 = 0.0f;
  float prev_vel_2 = 0.0f;

  PidControl pid;
};

