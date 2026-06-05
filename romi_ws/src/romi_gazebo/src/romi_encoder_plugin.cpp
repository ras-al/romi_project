#include "romi_encoder_plugin.h"

#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/Util.hh>

#include <ignition/msgs/odometry.pb.h>
#include <ignition/msgs/twist.pb.h>

#include <cmath>
#include <iostream>

// Include WheelWithEncoder and PidControl
#include "wheel_with_encoder.h"
#include "pid_control.h"

using namespace ignition;
using namespace gazebo;

namespace romi {
namespace gazebo {

RomiEncoderPlugin::RomiEncoderPlugin()
    : wheel_radius_(0.035),
      wheel_base_(0.14),
      encoder_ticks_per_rotation_(1440),
      control_dt_(0.025),
      pid_kp_(0.1613f),
      pid_ki_(12.9032f),
      pid_kd_(0.0f),
      x_(0.0),
      y_(0.0),
      yaw_(0.0),
      prev_left_ticks_(0),
      prev_right_ticks_(0),
      prev_time_(-1.0),
      last_encoder_update_time_(-1.0),
      left_joint_velocity_(0.0),
      right_joint_velocity_(0.0),
      last_left_target_angular_vel_(0.0),
      last_right_target_angular_vel_(0.0),
      last_left_encoder_ticks_(0),
      last_right_encoder_ticks_(0),
      first_encoder_update_(true),
      cmd_vel_topic_("/model/romi/cmd_vel"),
      odometry_topic_("/model/romi/odometry") {
  std::cout << "[RomiEncoderPlugin] Constructor: Plugin created" << std::endl;
}

RomiEncoderPlugin::~RomiEncoderPlugin() = default;

void RomiEncoderPlugin::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &_eventMgr) {
  
  model_entity_ = _entity;
  
  // Load parameters from SDF
  if (_sdf) {
    LoadParameters(_sdf);
  }
  
  // Create wheel encoder models
  left_wheel_ = std::make_unique<WheelWithEncoder>(pid_kp_, pid_ki_, pid_kd_);
  right_wheel_ = std::make_unique<WheelWithEncoder>(pid_kp_, pid_ki_, pid_kd_);
  
  // Find joint entities
  FindJoints(_ecm);
  
  // Setup transport (topics)
  SetupTransport();
  
  std::cout << "[RomiEncoderPlugin] Configured successfully:" << std::endl;
  std::cout << "  Wheel radius: " << wheel_radius_ << " m" << std::endl;
  std::cout << "  Wheel base: " << wheel_base_ << " m" << std::endl;
  std::cout << "  Encoder ticks/rotation: " << encoder_ticks_per_rotation_ << std::endl;
  std::cout << "  Control dt: " << control_dt_ << " s (" << (1.0/control_dt_) << " Hz)" << std::endl;
  std::cout << "  PID gains: Kp=" << pid_kp_ << ", Ki=" << pid_ki_ << ", Kd=" << pid_kd_ << std::endl;
  std::cout << "  Cmd vel topic: " << cmd_vel_topic_ << std::endl;
  std::cout << "  Odometry topic: " << odometry_topic_ << std::endl;
}

void RomiEncoderPlugin::PreUpdate(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm) {
  
  // Read actual joint velocities from physics
  if (left_joint_entity_ != kNullEntity) {
    auto left_vel_comp = _ecm.Component<components::JointVelocity>(left_joint_entity_);
    if (left_vel_comp) {
      // JointVelocity is a vector, get the first component (angular velocity)
      auto velocities = left_vel_comp->Data();
      if (!velocities.empty()) {
        left_joint_velocity_ = velocities[0];
      } else {
        left_joint_velocity_ = 0.0;
      }
    } else {
      left_joint_velocity_ = 0.0;
    }
  }
  
  if (right_joint_entity_ != kNullEntity) {
    auto right_vel_comp = _ecm.Component<components::JointVelocity>(right_joint_entity_);
    if (right_vel_comp) {
      auto velocities = right_vel_comp->Data();
      if (!velocities.empty()) {
        right_joint_velocity_ = velocities[0];
      } else {
        right_joint_velocity_ = 0.0;
      }
    } else {
      right_joint_velocity_ = 0.0;
    }
  }
  
  // Calculate current simulation time
  double current_time = std::chrono::duration<double>(_info.simTime).count();
  
  // Throttle wheel encoder updates to control_dt_ (25ms) intervals
  bool should_update_encoders = false;
  double encoder_dt = control_dt_;
  
  if (last_encoder_update_time_ < 0.0) {
    // First update - initialize
    last_encoder_update_time_ = current_time;
    should_update_encoders = true;
  } else {
    double time_since_last_update = current_time - last_encoder_update_time_;
    
    // Only update if at least control_dt_ (25ms) has elapsed
    if (time_since_last_update >= control_dt_) {
      should_update_encoders = true;
      encoder_dt = time_since_last_update;
      last_encoder_update_time_ = current_time;
    }
  }
  
  // Always maintain velocity commands (set every PreUpdate)
  // But only update encoder models at 25ms intervals
  double left_target_angular_vel = last_left_target_angular_vel_;
  double right_target_angular_vel = last_right_target_angular_vel_;
  int left_encoder_ticks = last_left_encoder_ticks_;
  int right_encoder_ticks = last_right_encoder_ticks_;
  
  // Force first update to happen immediately
  if (first_encoder_update_) {
    should_update_encoders = true;
    encoder_dt = control_dt_;
    first_encoder_update_ = false;
  }
  
  if (should_update_encoders) {
    // Clamp dt to reasonable values
    if (encoder_dt > 0.1 || encoder_dt < 0.0) {
      encoder_dt = control_dt_;
    }
    
    // Check if target is zero (stopping command)
    // Get current target from wheel encoder models
    bool left_target_zero = (left_wheel_->getTarget() == 0);
    bool right_target_zero = (right_wheel_->getTarget() == 0);
    
    if (left_target_zero && right_target_zero) {
      // Both targets are zero - set velocity commands to zero immediately
      left_target_angular_vel = 0.0;
      right_target_angular_vel = 0.0;
      left_encoder_ticks = 0;
      right_encoder_ticks = 0;
    } else {
      // Convert joint velocities (rad/s) to encoder ticks per sample
      int left_actual_ticks = AngularVelocityToEncoderTicksInt(left_joint_velocity_, encoder_dt);
      int right_actual_ticks = AngularVelocityToEncoderTicksInt(right_joint_velocity_, encoder_dt);
      
      // Update wheel encoder models with actual velocities as feedback (hybrid approach)
      left_encoder_ticks = left_wheel_->update(static_cast<float>(left_actual_ticks));
      right_encoder_ticks = right_wheel_->update(static_cast<float>(right_actual_ticks));
      
      // Apply motor dynamics output as joint velocity commands
      // Convert encoder ticks back to angular velocities
      left_target_angular_vel = EncoderTicksToAngularVelocity(left_encoder_ticks, encoder_dt);
      right_target_angular_vel = EncoderTicksToAngularVelocity(right_encoder_ticks, encoder_dt);
    }
    
    // Store for next iteration
    last_left_target_angular_vel_ = left_target_angular_vel;
    last_right_target_angular_vel_ = right_target_angular_vel;
    last_left_encoder_ticks_ = left_encoder_ticks;
    last_right_encoder_ticks_ = right_encoder_ticks;
    
    // Update odometry from encoder readings
    UpdateOdometry(left_encoder_ticks, right_encoder_ticks, encoder_dt);
    
    prev_time_ = current_time;
  }
  
  // Always set joint velocity commands in PreUpdate (before physics step)
  // This ensures commands are applied every physics step, not just when encoders update
  if (left_joint_entity_ != kNullEntity) {
    std::vector<double> left_vel_data = {left_target_angular_vel};
    _ecm.SetComponentData<components::JointVelocityCmd>(left_joint_entity_, left_vel_data);
  }
  
  if (right_joint_entity_ != kNullEntity) {
    std::vector<double> right_vel_data = {right_target_angular_vel};
    _ecm.SetComponentData<components::JointVelocityCmd>(right_joint_entity_, right_vel_data);
  }
}

void RomiEncoderPlugin::Update(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm) {
  
  // Update phase is now empty - all work moved to PreUpdate
  // This ensures velocity commands are set before physics step
}

void RomiEncoderPlugin::PostUpdate(
    const UpdateInfo &_info,
    const EntityComponentManager &_ecm) {
  
  // Publish odometry
  PublishOdometry(_info);
}

void RomiEncoderPlugin::OnCmdVel(const msgs::Twist &_msg) {
  // Extract linear and angular velocities
  double linear_x = _msg.linear().x();
  double angular_z = _msg.angular().z();
  
  // Convert to individual wheel velocities using differential drive kinematics
  double v_left = linear_x - (angular_z * wheel_base_ / 2.0);
  double v_right = linear_x + (angular_z * wheel_base_ / 2.0);
  
  // Convert wheel velocities (m/s) to angular velocities (rad/s)
  double left_angular_vel = v_left / wheel_radius_;
  double right_angular_vel = v_right / wheel_radius_;
  
  // Convert angular velocities to encoder ticks per sample
  int left_target_ticks = AngularVelocityToEncoderTicksInt(left_angular_vel, control_dt_);
  int right_target_ticks = AngularVelocityToEncoderTicksInt(right_angular_vel, control_dt_);
  
  // Set targets in wheel encoder models
  left_wheel_->setTarget(left_target_ticks);
  right_wheel_->setTarget(right_target_ticks);
}

void RomiEncoderPlugin::LoadParameters(const std::shared_ptr<const sdf::Element> &_sdf) {
  // Load robot parameters
  if (_sdf->HasElement("wheel_radius")) {
    wheel_radius_ = _sdf->Get<double>("wheel_radius");
  }
  if (_sdf->HasElement("wheel_base")) {
    wheel_base_ = _sdf->Get<double>("wheel_base");
  }
  if (_sdf->HasElement("encoder_ticks_per_rotation")) {
    encoder_ticks_per_rotation_ = _sdf->Get<int>("encoder_ticks_per_rotation");
  }
  if (_sdf->HasElement("control_dt")) {
    control_dt_ = _sdf->Get<double>("control_dt");
  }
  
  // Load PID parameters
  if (_sdf->HasElement("pid_kp")) {
    pid_kp_ = _sdf->Get<float>("pid_kp");
  }
  if (_sdf->HasElement("pid_ki")) {
    pid_ki_ = _sdf->Get<float>("pid_ki");
  }
  if (_sdf->HasElement("pid_kd")) {
    pid_kd_ = _sdf->Get<float>("pid_kd");
  }
  
  // Load topic names
  if (_sdf->HasElement("cmd_vel_topic")) {
    cmd_vel_topic_ = _sdf->Get<std::string>("cmd_vel_topic");
  }
  if (_sdf->HasElement("odometry_topic")) {
    odometry_topic_ = _sdf->Get<std::string>("odometry_topic");
  }
}

void RomiEncoderPlugin::SetupTransport() {
  // Subscribe to cmd_vel topic
  if (!node_.Subscribe(cmd_vel_topic_, &RomiEncoderPlugin::OnCmdVel, this)) {
    std::cerr << "[RomiEncoderPlugin] ERROR: Failed to subscribe to " << cmd_vel_topic_ << std::endl;
  }
  
  // Advertise odometry topic
  odom_pub_ = node_.Advertise<ignition::msgs::Odometry>(odometry_topic_);
  if (!odom_pub_) {
    std::cerr << "[RomiEncoderPlugin] ERROR: Failed to advertise " << odometry_topic_ << std::endl;
  }
}

void RomiEncoderPlugin::FindJoints(EntityComponentManager &_ecm) {
  // Find joints by name - search all joints and match by name
  _ecm.Each<components::Joint, components::Name>(
      [&](const Entity &_jointEntity,
          components::Joint *_joint,
          components::Name *_name) -> bool {
        std::string joint_name = _name->Data();
        
        // Check if this joint belongs to our model by checking parent
        auto parent_comp = _ecm.Component<components::ParentEntity>(_jointEntity);
        if (parent_comp && parent_comp->Data() == model_entity_) {
          if (joint_name == "left_wheel_joint") {
            left_joint_entity_ = _jointEntity;
            // Enable velocity component for reading
            _ecm.CreateComponent(left_joint_entity_, components::JointVelocity());
            // Create velocity command component (initialize to zero)
            _ecm.CreateComponent(left_joint_entity_, components::JointVelocityCmd({0.0}));
          } else if (joint_name == "right_wheel_joint") {
            right_joint_entity_ = _jointEntity;
            // Enable velocity component for reading
            _ecm.CreateComponent(right_joint_entity_, components::JointVelocity());
            // Create velocity command component (initialize to zero)
            _ecm.CreateComponent(right_joint_entity_, components::JointVelocityCmd({0.0}));
          }
        }
        
        return true;
      });
  
  if (left_joint_entity_ == kNullEntity) {
    std::cerr << "[RomiEncoderPlugin] ERROR: Could not find joint: left_wheel_joint" << std::endl;
  }
  if (right_joint_entity_ == kNullEntity) {
    std::cerr << "[RomiEncoderPlugin] ERROR: Could not find joint: right_wheel_joint" << std::endl;
  }
}

double RomiEncoderPlugin::AngularVelocityToEncoderTicks(
    double angular_vel_rad_per_sec, double dt) const {
  // Convert rad/s to rotations per second
  double rotations_per_sec = angular_vel_rad_per_sec / (2.0 * M_PI);
  // Convert to encoder ticks per second
  double ticks_per_sec = rotations_per_sec * encoder_ticks_per_rotation_;
  // Convert to encoder ticks per sample period
  return ticks_per_sec * dt;
}

int RomiEncoderPlugin::AngularVelocityToEncoderTicksInt(
    double angular_vel_rad_per_sec, double dt) const {
  return static_cast<int>(std::round(AngularVelocityToEncoderTicks(angular_vel_rad_per_sec, dt)));
}

double RomiEncoderPlugin::EncoderTicksToAngularVelocity(
    int encoder_ticks, double dt) const {
  // Convert encoder ticks per sample to ticks per second
  double ticks_per_sec = static_cast<double>(encoder_ticks) / dt;
  // Convert to rotations per second
  double rotations_per_sec = ticks_per_sec / encoder_ticks_per_rotation_;
  // Convert to rad/s
  return rotations_per_sec * 2.0 * M_PI;
}

double RomiEncoderPlugin::EncoderTicksToDistance(int encoder_ticks) const {
  // Convert encoder ticks to distance in meters
  double wheel_circumference = 2.0 * M_PI * wheel_radius_;
  return (wheel_circumference / encoder_ticks_per_rotation_) * encoder_ticks;
}

int RomiEncoderPlugin::DistanceToEncoderTicks(double distance_m) const {
  double wheel_circumference = 2.0 * M_PI * wheel_radius_;
  return static_cast<int>(std::round((distance_m / wheel_circumference) * encoder_ticks_per_rotation_));
}

void RomiEncoderPlugin::UpdateOdometry(int left_ticks, int right_ticks, double dt) {
  // Skip first update (need previous values)
  if (prev_time_ < 0.0) {
    prev_left_ticks_ = left_ticks;
    prev_right_ticks_ = right_ticks;
    return;
  }
  
  // Calculate change in encoder ticks (delta)
  int delta_left = left_ticks - prev_left_ticks_;
  int delta_right = right_ticks - prev_right_ticks_;
  
  // Convert encoder ticks to distances
  double left_distance = EncoderTicksToDistance(delta_left);
  double right_distance = EncoderTicksToDistance(delta_right);
  
  // Differential drive forward kinematics
  double v = (left_distance + right_distance) / (2.0 * dt);  // Linear velocity
  double omega = (right_distance - left_distance) / (wheel_base_ * dt);  // Angular velocity
  
  // Update pose
  x_ += v * std::cos(yaw_) * dt;
  y_ += v * std::sin(yaw_) * dt;
  yaw_ += omega * dt;
  
  // Normalize yaw to [-pi, pi]
  while (yaw_ > M_PI) yaw_ -= 2.0 * M_PI;
  while (yaw_ < -M_PI) yaw_ += 2.0 * M_PI;
  
  // Store current ticks for next iteration
  prev_left_ticks_ = left_ticks;
  prev_right_ticks_ = right_ticks;
}

void RomiEncoderPlugin::PublishOdometry(const UpdateInfo &_info) {
  msgs::Odometry odom_msg;
  
  // Set header
  odom_msg.mutable_header()->mutable_stamp()->set_sec(
      std::chrono::duration_cast<std::chrono::seconds>(_info.simTime).count());
  odom_msg.mutable_header()->mutable_stamp()->set_nsec(
      std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime).count() % 1000000000);
  odom_msg.mutable_header()->add_data()->set_key("frame_id");
  odom_msg.mutable_header()->mutable_data(0)->add_value("odom");
  
  // Set pose
  msgs::Pose *pose = odom_msg.mutable_pose();
  pose->mutable_position()->set_x(x_);
  pose->mutable_position()->set_y(y_);
  pose->mutable_position()->set_z(0.0);
  
  // Convert yaw to quaternion
  double qw = std::cos(yaw_ / 2.0);
  double qz = std::sin(yaw_ / 2.0);
  pose->mutable_orientation()->set_w(qw);
  pose->mutable_orientation()->set_x(0.0);
  pose->mutable_orientation()->set_y(0.0);
  pose->mutable_orientation()->set_z(qz);
  
  // Set twist (velocities)
  msgs::Twist *twist = odom_msg.mutable_twist();
  
  // Calculate velocities from current joint velocities (more accurate)
  double left_angular_vel = left_joint_velocity_;
  double right_angular_vel = right_joint_velocity_;
  double left_linear_vel = left_angular_vel * wheel_radius_;
  double right_linear_vel = right_angular_vel * wheel_radius_;
  double v = (left_linear_vel + right_linear_vel) / 2.0;
  double omega = (right_linear_vel - left_linear_vel) / wheel_base_;
  
  twist->mutable_linear()->set_x(v);
  twist->mutable_linear()->set_y(0.0);
  twist->mutable_linear()->set_z(0.0);
  twist->mutable_angular()->set_x(0.0);
  twist->mutable_angular()->set_y(0.0);
  twist->mutable_angular()->set_z(omega);
  
  // Publish
  odom_pub_.Publish(odom_msg);
}

}  // namespace gazebo
}  // namespace romi

// Register the plugin
IGNITION_ADD_PLUGIN(romi::gazebo::RomiEncoderPlugin,
                    ignition::gazebo::System,
                    ignition::gazebo::ISystemConfigure,
                    ignition::gazebo::ISystemPreUpdate,
                    ignition::gazebo::ISystemUpdate,
                    ignition::gazebo::ISystemPostUpdate)

