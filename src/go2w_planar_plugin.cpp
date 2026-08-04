#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

namespace gazebo {

class Go2WPlanarPlugin : public ModelPlugin {
 public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = model;
    if (!ros::isInitialized()) {
      ROS_FATAL("go2w_planar_plugin requires gazebo_ros_api_plugin");
      return;
    }
    nh_.reset(new ros::NodeHandle(sdf->Get<std::string>("robotNamespace", "").first));
    cmd_topic_ = sdf->Get<std::string>("commandTopic", "cmd_vel").first;
    odom_topic_ = sdf->Get<std::string>("odometryTopic", "odom").first;
    odom_frame_ = sdf->Get<std::string>("odometryFrame", "odom").first;
    base_frame_ = sdf->Get<std::string>("robotBaseFrame", "base_footprint").first;
    max_linear_ = sdf->Get<double>("maxLinear", 1.2).first;
    max_angular_ = sdf->Get<double>("maxAngular", 1.8).first;
    accel_ = sdf->Get<double>("acceleration", 1.8).first;
    angular_accel_ = sdf->Get<double>("angularAcceleration", 3.0).first;
    timeout_ = sdf->Get<double>("commandTimeout", 0.5).first;
    publish_tf_ = sdf->Get<bool>("publishTf", true).first;

    const std::vector<std::string> leg_names = {
        "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"};
    for (const auto& name : leg_names) leg_joints_.push_back(model_->GetJoint(name));
    const std::vector<std::string> wheel_names = {
        "FL_foot_joint", "FR_foot_joint", "RL_foot_joint", "RR_foot_joint"};
    for (const auto& name : wheel_names) wheel_joints_.push_back(model_->GetJoint(name));

    initial_pose_ = model_->WorldPose();
    pose_ = initial_pose_;
    cmd_sub_ = nh_->subscribe(cmd_topic_, 1, &Go2WPlanarPlugin::OnCommand, this);
    odom_pub_ = nh_->advertise<nav_msgs::Odometry>(odom_topic_, 5);
    last_update_ = model_->GetWorld()->SimTime();
    last_command_ = ros::Time(0);
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&Go2WPlanarPlugin::OnUpdate, this));
    ROS_INFO("Go2-W planar plugin ready: %s -> %s", cmd_topic_.c_str(), odom_topic_.c_str());
  }

 private:
  static double Clamp(double value, double lower, double upper) {
    return std::max(lower, std::min(upper, value));
  }
  static double Approach(double current, double target, double amount) {
    return current + Clamp(target - current, -amount, amount);
  }

  void OnCommand(const geometry_msgs::TwistConstPtr& msg) {
    std::lock_guard<std::mutex> guard(mutex_);
    target_linear_ = Clamp(msg->linear.x, -max_linear_, max_linear_);
    target_angular_ = Clamp(msg->angular.z, -max_angular_, max_angular_);
    last_command_ = ros::Time::now();
  }

  void OnUpdate() {
    const common::Time now = model_->GetWorld()->SimTime();
    const double dt = Clamp((now - last_update_).Double(), 0.0, 0.1);
    if (dt <= 0.0) return;
    last_update_ = now;
    double target_linear = 0.0, target_angular = 0.0;
    {
      std::lock_guard<std::mutex> guard(mutex_);
      if ((ros::Time::now() - last_command_).toSec() <= timeout_) {
        target_linear = target_linear_;
        target_angular = target_angular_;
      }
    }
    linear_ = Approach(linear_, target_linear, accel_ * dt);
    angular_ = Approach(angular_, target_angular, angular_accel_ * dt);

    double yaw = pose_.Rot().Yaw() + angular_ * dt;
    pose_.Pos().X() += linear_ * std::cos(yaw) * dt;
    pose_.Pos().Y() += linear_ * std::sin(yaw) * dt;
    pose_.Rot() = ignition::math::Quaterniond(0.0, 0.0, yaw);
    model_->SetWorldPose(pose_, true, false);

    const double stance[12] = {0.0, 0.72, -1.45, 0.0, 0.72, -1.45,
                               0.0, 0.72, -1.45, 0.0, 0.72, -1.45};
    for (size_t i = 0; i < leg_joints_.size(); ++i)
      if (leg_joints_[i]) leg_joints_[i]->SetPosition(0, stance[i], true);
    const double left = -(linear_ - angular_ * 0.24) / 0.10;
    const double right = -(linear_ + angular_ * 0.24) / 0.10;
    const double wheel_speed[4] = {left, right, left, right};
    for (size_t i = 0; i < wheel_joints_.size(); ++i) {
      wheel_angle_[i] = std::remainder(wheel_angle_[i] + wheel_speed[i] * dt, 2.0 * M_PI);
      if (wheel_joints_[i]) wheel_joints_[i]->SetPosition(0, wheel_angle_[i], true);
    }
    PublishOdometry(now);
  }

  void PublishOdometry(const common::Time& sim_time) {
    const ros::Time stamp(sim_time.sec, sim_time.nsec);
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = pose_.Pos().X();
    odom.pose.pose.position.y = pose_.Pos().Y();
    odom.pose.pose.position.z = 0.0;
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, pose_.Rot().Yaw());
    tf::quaternionTFToMsg(q, odom.pose.pose.orientation);
    odom.twist.twist.linear.x = linear_;
    odom.twist.twist.angular.z = angular_;
    odom_pub_.publish(odom);
    if (publish_tf_) {
      tf::Transform transform(q, tf::Vector3(pose_.Pos().X(), pose_.Pos().Y(), 0.0));
      tf_broadcaster_.sendTransform(tf::StampedTransform(transform, stamp, odom_frame_, base_frame_));
    }
  }

  physics::ModelPtr model_;
  physics::Joint_V leg_joints_, wheel_joints_;
  event::ConnectionPtr update_connection_;
  std::unique_ptr<ros::NodeHandle> nh_;
  ros::Subscriber cmd_sub_;
  ros::Publisher odom_pub_;
  tf::TransformBroadcaster tf_broadcaster_;
  std::mutex mutex_;
  ignition::math::Pose3d initial_pose_, pose_;
  common::Time last_update_;
  ros::Time last_command_;
  std::string cmd_topic_, odom_topic_, odom_frame_, base_frame_;
  double target_linear_{0.0}, target_angular_{0.0}, linear_{0.0}, angular_{0.0};
  double max_linear_{1.2}, max_angular_{1.8}, accel_{1.8}, angular_accel_{3.0}, timeout_{0.5};
  double wheel_angle_[4]{0.0, 0.0, 0.0, 0.0};
  bool publish_tf_{true};
};

GZ_REGISTER_MODEL_PLUGIN(Go2WPlanarPlugin)
}  // namespace gazebo
