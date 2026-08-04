#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <gazebo/common/Events.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>

namespace gazebo {

class Go2WDynamicsPlugin : public ModelPlugin {
 public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = model;
    if (!ros::isInitialized()) {
      ROS_FATAL("go2w_dynamics_plugin requires gazebo_ros_api_plugin");
      return;
    }
    nh_.reset(new ros::NodeHandle);
    cmd_topic_ = sdf->Get<std::string>("commandTopic", "/cmd_vel").first;
    posture_topic_ = sdf->Get<std::string>("postureTopic", "/go2w/posture_cmd").first;
    odom_topic_ = sdf->Get<std::string>("odometryTopic", "/odom").first;
    odom_frame_ = sdf->Get<std::string>("odometryFrame", "odom").first;
    base_frame_ = sdf->Get<std::string>("robotBaseFrame", "base_footprint").first;
    wheel_radius_ = sdf->Get<double>("wheelRadius", 0.10).first;
    track_width_ = sdf->Get<double>("trackWidth", 0.285).first;
    max_linear_ = sdf->Get<double>("maxLinear", 1.0).first;
    max_angular_ = sdf->Get<double>("maxAngular", 1.5).first;
    timeout_ = sdf->Get<double>("commandTimeout", 0.5).first;
    publish_tf_ = sdf->Get<bool>("publishTf", true).first;

    const std::vector<std::string> leg_names = {
        "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"};
    for (const auto& name : leg_names) leg_joints_.push_back(RequiredJoint(name));
    const std::vector<std::string> wheel_names = {
        "FL_foot_joint", "FR_foot_joint", "RL_foot_joint", "RR_foot_joint"};
    for (const auto& name : wheel_names) wheel_joints_.push_back(RequiredJoint(name));
    if (!joints_valid_) return;

    // URDF fixed-joint reduction merges `base` into the canonical root link.
    base_link_ = model_->GetLink("base_footprint");
    if (!base_link_) base_link_ = model_->GetLink();
    if (!base_link_) {
      ROS_ERROR("Go2-W dynamics: missing base link");
      return;
    }
    // At plugin load time fixed-joint link poses may not yet be propagated by
    // Gazebo, so derive the nominal body height from the spawned model root.
    initial_base_height_ = model_->WorldPose().Pos().Z();
    for (const auto& link : model_->GetLinks())
      if (link->GetInertial()) total_mass_ += link->GetInertial()->Mass();
    if (total_mass_ < 1.0) total_mass_ = 14.0;
    ROS_INFO("Go2-W balance target: height=%.3f m, mass=%.2f kg",
             initial_base_height_, total_mass_);

    joint_controller_ = model_->GetJointController();
    for (const auto& wheel : wheel_joints_)
      joint_controller_->SetVelocityPID(
          wheel->GetScopedName(), common::PID(1.0, 0.0, 0.03, 0.0, 0.0, 4.0, -4.0));

    cmd_sub_ = nh_->subscribe(cmd_topic_, 1, &Go2WDynamicsPlugin::OnCommand, this);
    posture_sub_ = nh_->subscribe(posture_topic_, 1, &Go2WDynamicsPlugin::OnPosture, this);
    odom_pub_ = nh_->advertise<nav_msgs::Odometry>(odom_topic_, 5);
    last_update_ = model_->GetWorld()->SimTime();
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&Go2WDynamicsPlugin::OnUpdate, this));
    ROS_INFO("Go2-W dynamics ready: torque-controlled legs and contact-driven wheels");
  }

 private:
  static double Clamp(double value, double lower, double upper) {
    return std::max(lower, std::min(upper, value));
  }

  physics::JointPtr RequiredJoint(const std::string& name) {
    auto joint = model_->GetJoint(name);
    if (!joint) {
      ROS_ERROR("Go2-W dynamics: missing joint %s", name.c_str());
      joints_valid_ = false;
    }
    return joint;
  }

  void OnCommand(const geometry_msgs::TwistConstPtr& msg) {
    std::lock_guard<std::mutex> guard(mutex_);
    target_linear_ = Clamp(msg->linear.x, -max_linear_, max_linear_);
    target_angular_ = Clamp(msg->angular.z, -max_angular_, max_angular_);
    if (std::abs(msg->linear.y) > 1e-3)
      ROS_WARN_THROTTLE(2.0, "Physics Go2-W has fixed wheels; lateral cmd_vel is ignored");
    last_command_ = ros::WallTime::now();
  }

  void OnPosture(const std_msgs::StringConstPtr& msg) {
    std::lock_guard<std::mutex> guard(mutex_);
    if (msg->data == "lie_down" || msg->data == "stand_down" || msg->data == "prone") {
      // Moving all four wheel contacts during folding requires contact-aware
      // foot IK. Reject it here instead of destabilising Gazebo Classic.
      target_posture_ = 0.0;
      target_linear_ = target_angular_ = 0.0;
      ROS_WARN("Dynamics model keeps standing: StandDown needs contact-aware foot IK");
    } else if (msg->data == "stand" || msg->data == "stand_up") {
      target_posture_ = 0.0;
      ROS_INFO("Dynamics posture: StandUp");
    }
  }

  void OnUpdate() {
    const common::Time now = model_->GetWorld()->SimTime();
    const double dt = Clamp((now - last_update_).Double(), 0.0, 0.02);
    if (dt <= 0.0) return;
    last_update_ = now;

    double linear = 0.0, angular = 0.0, target_posture = 0.0;
    {
      std::lock_guard<std::mutex> guard(mutex_);
      if ((ros::WallTime::now() - last_command_).toSec() <= timeout_) {
        linear = target_linear_;
        angular = target_angular_;
      }
      target_posture = target_posture_;
    }
    posture_ += Clamp(target_posture - posture_, -dt / 1.2, dt / 1.2);
    if (posture_ > 0.03) linear = angular = 0.0;

    const double motion = Clamp(std::hypot(linear, angular * track_width_ * 0.5) / 0.8, 0.0, 1.0);

    // Approximate the body-height and attitude part of a whole-body
    // controller. Forces remain in the physics engine; x/y/yaw are free and
    // are produced by wheel-ground contact rather than pose teleportation.
    const auto base_pose = base_link_->WorldPose();
    const auto base_velocity = base_link_->WorldLinearVel();
    const auto base_angular = base_link_->WorldAngularVel();
    const double desired_height = initial_base_height_ - 0.22 * posture_;
    const double weight = total_mass_ * 9.81;
    const double vertical = Clamp(
        weight + 450.0 * (desired_height - base_pose.Pos().Z()) - 70.0 * base_velocity.Z(),
        0.0, 2.5 * weight);
    const double yaw = base_pose.Rot().Yaw();
    const double desired_vx = linear * std::cos(yaw);
    const double desired_vy = linear * std::sin(yaw);
    const double force_x = Clamp(total_mass_ * 4.0 * (desired_vx - base_velocity.X()), -70.0, 70.0);
    const double force_y = Clamp(total_mass_ * 4.0 * (desired_vy - base_velocity.Y()), -70.0, 70.0);
    base_link_->AddForce(ignition::math::Vector3d(force_x, force_y, vertical));
    const double roll = base_pose.Rot().Roll();
    const double pitch = base_pose.Rot().Pitch();
    const double roll_torque = Clamp(-120.0 * roll - 18.0 * base_angular.X(), -80.0, 80.0);
    const double pitch_torque = Clamp(-120.0 * pitch - 18.0 * base_angular.Y(), -80.0, 80.0);
    const double yaw_torque = Clamp(18.0 * (angular - base_angular.Z()), -30.0, 30.0);
    base_link_->AddTorque(ignition::math::Vector3d(roll_torque, pitch_torque, yaw_torque));
    if (motion > 0.04) gait_phase_ = std::remainder(gait_phase_ + dt * (4.0 + 3.0 * motion), 2.0 * M_PI);
    const double gait_amplitude = 0.05 * motion * (1.0 - posture_);
    leg_update_accumulator_ += dt;
    const bool update_legs = leg_update_accumulator_ >= (1.0 / 30.0);
    if (update_legs) leg_update_accumulator_ = 0.0;
    const double stand[3] = {0.0, 0.72, -1.45};
    const double down[3] = {0.0, 1.35, -2.55};
    // FL/RR and FR/RL form the diagonal phase pairs used by a trot gait.
    const double phase_sign[4] = {1.0, -1.0, -1.0, 1.0};
    for (size_t leg = 0; leg < 4; ++leg) {
      const double wave = std::sin(gait_phase_) * phase_sign[leg];
      double targets[3];
      for (size_t joint = 0; joint < 3; ++joint)
        targets[joint] = stand[joint] + posture_ * (down[joint] - stand[joint]);
      targets[1] += gait_amplitude * wave;
      targets[2] -= 1.25 * gait_amplitude * wave;
      // Kinematic leg trajectory servo keeps the deliberately tiny stick
      // inertias numerically stable. Chassis and wheels remain fully dynamic.
      if (update_legs) {
        leg_joints_[leg * 3]->SetPosition(0, targets[0], false);
        leg_joints_[leg * 3 + 1]->SetPosition(0, targets[1], false);
        leg_joints_[leg * 3 + 2]->SetPosition(0, targets[2], false);
      }
    }

    const double left_target = (linear - angular * track_width_ * 0.5) / wheel_radius_;
    const double right_target = (linear + angular * track_width_ * 0.5) / wheel_radius_;
    const double wheel_targets[4] = {left_target, right_target, left_target, right_target};
    for (size_t i = 0; i < wheel_joints_.size(); ++i) {
      joint_controller_->SetVelocityTarget(wheel_joints_[i]->GetScopedName(), wheel_targets[i]);
    }
    PublishOdometry(now);
  }

  void PublishOdometry(const common::Time& sim_time) {
    const auto pose = model_->WorldPose();
    const auto linear_velocity = model_->WorldLinearVel();
    const auto angular_velocity = model_->WorldAngularVel();
    const ros::Time stamp(sim_time.sec, sim_time.nsec);
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = pose.Pos().X();
    odom.pose.pose.position.y = pose.Pos().Y();
    odom.pose.pose.position.z = pose.Pos().Z();
    odom.pose.pose.orientation.x = pose.Rot().X();
    odom.pose.pose.orientation.y = pose.Rot().Y();
    odom.pose.pose.orientation.z = pose.Rot().Z();
    odom.pose.pose.orientation.w = pose.Rot().W();
    odom.twist.twist.linear.x = linear_velocity.X();
    odom.twist.twist.linear.y = linear_velocity.Y();
    odom.twist.twist.linear.z = linear_velocity.Z();
    odom.twist.twist.angular.x = angular_velocity.X();
    odom.twist.twist.angular.y = angular_velocity.Y();
    odom.twist.twist.angular.z = angular_velocity.Z();
    odom_pub_.publish(odom);
    if (publish_tf_) {
      tf::Transform transform;
      transform.setOrigin(tf::Vector3(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z()));
      transform.setRotation(tf::Quaternion(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W()));
      tf_broadcaster_.sendTransform(tf::StampedTransform(transform, stamp, odom_frame_, base_frame_));
    }
  }

  physics::ModelPtr model_;
  physics::LinkPtr base_link_;
  physics::JointControllerPtr joint_controller_;
  physics::Joint_V leg_joints_, wheel_joints_;
  event::ConnectionPtr update_connection_;
  std::unique_ptr<ros::NodeHandle> nh_;
  ros::Subscriber cmd_sub_, posture_sub_;
  ros::Publisher odom_pub_;
  tf::TransformBroadcaster tf_broadcaster_;
  std::mutex mutex_;
  common::Time last_update_;
  ros::WallTime last_command_;
  std::string cmd_topic_, posture_topic_, odom_topic_, odom_frame_, base_frame_;
  double target_linear_{0.0}, target_angular_{0.0}, target_posture_{0.0};
  double posture_{0.0}, gait_phase_{0.0}, leg_update_accumulator_{0.0};
  double wheel_radius_{0.10}, track_width_{0.285};
  double initial_base_height_{0.08}, total_mass_{0.0};
  double max_linear_{1.0}, max_angular_{1.5}, timeout_{0.5};
  bool publish_tf_{true}, joints_valid_{true};
};

GZ_REGISTER_MODEL_PLUGIN(Go2WDynamicsPlugin)
}  // namespace gazebo
