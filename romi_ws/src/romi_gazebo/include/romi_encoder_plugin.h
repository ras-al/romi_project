#pragma once

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/twist.pb.h>
#include <ignition/msgs/odometry.pb.h>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/Element.hh>

#include <memory>
#include <string>
#include <chrono>

// Forward declarations
class WheelWithEncoder;
class PidControl;

namespace romi {
namespace gazebo {

/**
 * Gazebo Ignition plugin for Romi robot wheel encoders.
 * 
 * Uses WheelWithEncoder class to model DC motor wheel encoder pairs.
 * Implements hybrid approach:
 * - cmd_vel sets target velocities
 * - Actual joint velocities from physics provide feedback
 * - Publishes odometry based on encoder readings
 */
class RomiEncoderPlugin : public ignition::gazebo::System,
                            public ignition::gazebo::ISystemConfigure,
                            public ignition::gazebo::ISystemPreUpdate,
                            public ignition::gazebo::ISystemUpdate,
                            public ignition::gazebo::ISystemPostUpdate {
public:
  RomiEncoderPlugin();
  ~RomiEncoderPlugin() override;

  // ISystemConfigure: Called once when the plugin is loaded
  void Configure(const ignition::gazebo::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 ignition::gazebo::EntityComponentManager &_ecm,
                 ignition::gazebo::EventManager &_eventMgr) override;

  // ISystemPreUpdate: Called before physics update
  void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;

  // ISystemUpdate: Called during update
  void Update(const ignition::gazebo::UpdateInfo &_info,
             ignition::gazebo::EntityComponentManager &_ecm) override;

  // ISystemPostUpdate: Called after physics update
  void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                 const ignition::gazebo::EntityComponentManager &_ecm) override;

private:
  // Callback for cmd_vel topic
  void OnCmdVel(const ignition::msgs::Twist &_msg);

  // Helper functions
  void LoadParameters(const std::shared_ptr<const sdf::Element> &_sdf);
  void SetupTransport();
  void FindJoints(ignition::gazebo::EntityComponentManager &_ecm);
  
  // Conversion functions
  double AngularVelocityToEncoderTicks(double angular_vel_rad_per_sec, double dt) const;
  double EncoderTicksToAngularVelocity(int encoder_ticks, double dt) const;
  double EncoderTicksToDistance(int encoder_ticks) const;
  int DistanceToEncoderTicks(double distance_m) const;
  int AngularVelocityToEncoderTicksInt(double angular_vel_rad_per_sec, double dt) const;

  // Odometry computation
  void UpdateOdometry(int left_ticks, int right_ticks, double dt);
  void PublishOdometry(const ignition::gazebo::UpdateInfo &_info);

  // Entity and component references
  ignition::gazebo::Entity model_entity_;
  ignition::gazebo::Entity left_joint_entity_;
  ignition::gazebo::Entity right_joint_entity_;

  // Wheel encoder models
  std::unique_ptr<WheelWithEncoder> left_wheel_;
  std::unique_ptr<WheelWithEncoder> right_wheel_;

  // Transport (for topics)
  ignition::transport::Node node_;
  ignition::transport::Node::Publisher odom_pub_;
  std::string cmd_vel_topic_;
  std::string odometry_topic_;

  // Robot parameters
  double wheel_radius_;           // Wheel radius in meters
  double wheel_base_;              // Distance between wheels in meters
  int encoder_ticks_per_rotation_; // Encoder resolution
  double control_dt_;              // Control loop period (25ms = 0.025s)

  // PID parameters
  float pid_kp_;
  float pid_ki_;
  float pid_kd_;

  // Odometry state
  double x_;      // X position in meters
  double y_;      // Y position in meters
  double yaw_;    // Heading in radians

  // Previous values for differential drive
  int prev_left_ticks_;
  int prev_right_ticks_;
  double prev_time_;
  double last_encoder_update_time_;  // Time of last wheel encoder update

  // Joint velocity storage (from physics)
  double left_joint_velocity_;
  double right_joint_velocity_;

  // Last computed values for maintaining continuous velocity commands
  double last_left_target_angular_vel_;
  double last_right_target_angular_vel_;
  int last_left_encoder_ticks_;
  int last_right_encoder_ticks_;
  bool first_encoder_update_;
};

}  // namespace gazebo
}  // namespace romi

