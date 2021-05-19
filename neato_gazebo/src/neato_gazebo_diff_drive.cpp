// Copyright 2021 Emerson Knapp
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <string>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/Joint.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/World.hh"

#include "gazebo_ros/conversions/builtin_interfaces.hpp"
#include "gazebo_ros/conversions/geometry_msgs.hpp"
#include "gazebo_ros/node.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "neato_msgs/msg/neato_wheel_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcpputils/thread_safety_annotations.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace neato_gazebo
{


class NeatoGazeboDiffDrive : public gazebo::ModelPlugin
{
public:
  NeatoGazeboDiffDrive() {}
  virtual ~NeatoGazeboDiffDrive() {}

protected:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;
  void Reset() override;

private:
  /// Callback to be called at every simulation iteration.
  void OnUpdate(const gazebo::common::UpdateInfo & info);

  /// Callback when a velocity command is received.
  void OnCmd(const neato_msgs::msg::NeatoWheelCommand::SharedPtr msg);

  /// Update odometry according to ground truth from simulation world
  void UpdateOdometry();

  /// Publish odometry message
  void PublishOdometryMsg(const gazebo::common::Time & sim_time);

  /// Publish trasforms for the wheels
  void PublishWheelTf(const gazebo::common::Time & sim_time, gazebo::physics::JointPtr joint);

  /// Publish odometry transforms
  void PublishOdometryTf(const gazebo::common::Time & sim_time);

  /// Update wheel velocities according to latest target velocities.
  void UpdateWheelJoints(double seconds_since_last_update);

private:
  struct Pose2D
  {
    double x;
    double y;
    double theta;
  };

  // configuration
  double wheel_diameter_;
  double max_wheel_torque_;
  double odom_covariance_[3];

  std::string robot_base_frame_;
  std::string odometry_frame_;

  double update_period_;
  bool publish_wheel_tf_;
  bool publish_odom_tf_;

  // Gazebo assets
  gazebo::physics::ModelPtr model_;
  gazebo::event::ConnectionPtr update_connection_;
  gazebo::physics::JointPtr left_wheel_joint_;
  gazebo::physics::JointPtr right_wheel_joint_;

  // ROS assets
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<neato_msgs::msg::NeatoWheelCommand>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  nav_msgs::msg::Odometry odom_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

  // dynamic tracking variables
  gazebo::common::Time last_update_time_;
  std::mutex cmd_mutex_;
  neato_msgs::msg::NeatoWheelCommand current_cmd_ RCPPUTILS_TSA_GUARDED_BY(cmd_mutex_);
  double last_sent_left_wheel_speed_;
  double last_sent_right_wheel_speed_;

};

void NeatoGazeboDiffDrive::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  model_ = model;
  ros_node_ = gazebo_ros::Node::Get(sdf);

  // Initialize general configuration
  wheel_diameter_ = sdf->Get<double>("wheel_diameter", 0.070).first;
  max_wheel_torque_ = sdf->Get<double>("max_wheel_torque", 5.0).first;
  odom_covariance_[0] = sdf->Get<double>("covariance_x", 0.00001).first;
  odom_covariance_[1] = sdf->Get<double>("covariance_y", 0.00001).first;
  odom_covariance_[2] = sdf->Get<double>("covariance_yaw", 0.001).first;

  odometry_frame_ = sdf->Get<std::string>("odometry_frame", "odom").first;
  robot_base_frame_ = sdf->Get<std::string>("robot_base_frame", "base_footprint").first;

  {
    auto update_rate = sdf->Get<double>("update_rate", 100.0).first;
    if (update_rate > 0.0) {
      update_period_ = 1.0 / update_rate;
    } else {
      update_period_ = 0.0;
    }
  }
  publish_wheel_tf_ = sdf->Get<bool>("publish_wheel_tf", false).first;
  publish_odom_tf_ = sdf->Get<bool>("publish_odom_tf", false).first;

  // Initialize Gazebo assets
  {
    auto joint_elem = sdf->GetElement("left_joint");
    auto joint_name = joint_elem->Get<std::string>();
    auto joint = model->GetJoint(joint_name);
    if (!joint) {
      RCLCPP_ERROR(
        ros_node_->get_logger(),
        "Joint [%s] not found, plugin will not work.", joint_name.c_str());
      ros_node_.reset();
      return;
    }
    left_wheel_joint_ = joint;
  }
  {
    auto joint_elem = sdf->GetElement("right_joint");
    auto joint_name = joint_elem->Get<std::string>();
    auto joint = model->GetJoint(joint_name);
    if (!joint) {
      RCLCPP_ERROR(
        ros_node_->get_logger(),
        "Joint [%s] not found, plugin will not work.", joint_name.c_str());
      ros_node_.reset();
      return;
    }
    right_wheel_joint_ = joint;
  }
  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&NeatoGazeboDiffDrive::OnUpdate, this, std::placeholders::_1));

  // Initialize ROS assets
  const auto & qos = ros_node_->get_qos();
  {
    // Comand subscription
    auto cmd_topic = sdf->Get<std::string>("command_topic", "neato_cmd").first;
    cmd_sub_ = ros_node_->create_subscription<neato_msgs::msg::NeatoWheelCommand>(
      cmd_topic,
      qos.get_subscription_qos(cmd_topic, rclcpp::QoS(1)),
      std::bind(&NeatoGazeboDiffDrive::OnCmd, this, std::placeholders::_1));
    RCLCPP_INFO(ros_node_->get_logger(), "Subscribed to command_topic [%s]", cmd_topic.c_str());
  }
  {
    // Odometry publisher
    bool publish_odom = sdf->Get<bool>("publish_odom", false).first;
    if (publish_odom) {
      auto odom_topic = sdf->Get<std::string>("odometry_topic", "odom").first;
      odom_pub_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>(
        odom_topic, qos.get_publisher_qos(odom_topic, rclcpp::QoS(1)));
      RCLCPP_INFO(ros_node_->get_logger(), "Publishing odometry to topic [%s]", odom_topic.c_str());
    } else {
      RCLCPP_INFO(ros_node_->get_logger(), "Not publishing odometry");
    }
  }
  transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(ros_node_);

  // Initialize tracking variables
  Reset();
}

void NeatoGazeboDiffDrive::Reset()
{
  last_update_time_ = left_wheel_joint_->GetWorld()->SimTime();
  left_wheel_joint_->SetParam("fmax", 0, max_wheel_torque_);
  right_wheel_joint_->SetParam("fmax", 0, max_wheel_torque_);
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    current_cmd_.left_dist = 0;
    current_cmd_.right_dist = 0;
    current_cmd_.speed = 0;
    current_cmd_.accel = 0;
  }
}

void NeatoGazeboDiffDrive::OnUpdate(const gazebo::common::UpdateInfo & info)
{
  double seconds_since_last_update = (info.simTime - last_update_time_).Double();
  if (seconds_since_last_update < update_period_) {
    return;
  }

  UpdateOdometry();
  if (odom_pub_) {
    PublishOdometryMsg(info.simTime);
  }
  if (publish_wheel_tf_) {
    PublishWheelTf(info.simTime, left_wheel_joint_);
    PublishWheelTf(info.simTime, right_wheel_joint_);
  }
  if (publish_odom_tf_) {
    PublishOdometryTf(info.simTime);
  }
  UpdateWheelJoints(seconds_since_last_update);

  last_update_time_ = info.simTime;
}

void NeatoGazeboDiffDrive::OnCmd(const neato_msgs::msg::NeatoWheelCommand::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cmd_mutex_);
  current_cmd_ = *msg;
  if (current_cmd_.speed < 0.0) {
    current_cmd_.speed = 0.0;
  }
  if (current_cmd_.accel <= 0.0) {
    current_cmd_.accel = current_cmd_.speed;
  }
  RCLCPP_INFO(ros_node_->get_logger(), "left_dist=%f, right_dist=%f, speed=%f, accel=%f",
              current_cmd_.left_dist, current_cmd_.right_dist, current_cmd_.speed, current_cmd_.accel);
}

void NeatoGazeboDiffDrive::UpdateOdometry()
{
  auto pose = model_->WorldPose();
  odom_.pose.pose.position = gazebo_ros::Convert<geometry_msgs::msg::Point>(pose.Pos());
  odom_.pose.pose.orientation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose.Rot());

  odom_.twist.twist.angular.z = model_->WorldAngularVel().Z();

  auto linear = model_->WorldLinearVel();
  float yaw = pose.Rot().Yaw();
  odom_.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
  odom_.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();
}

void NeatoGazeboDiffDrive::PublishOdometryMsg(const gazebo::common::Time & sim_time)
{
  odom_.pose.covariance[0] = odom_covariance_[0];
  odom_.pose.covariance[7] = odom_covariance_[1];
  odom_.pose.covariance[14] = 1000000000000.0;
  odom_.pose.covariance[21] = 1000000000000.0;
  odom_.pose.covariance[28] = 1000000000000.0;
  odom_.pose.covariance[35] = odom_covariance_[2];

  odom_.twist.covariance[0] = odom_covariance_[0];
  odom_.twist.covariance[7] = odom_covariance_[1];
  odom_.twist.covariance[14] = 1000000000000.0;
  odom_.twist.covariance[21] = 1000000000000.0;
  odom_.twist.covariance[28] = 1000000000000.0;
  odom_.twist.covariance[35] = odom_covariance_[2];

  odom_.header.frame_id = odometry_frame_;
  odom_.child_frame_id = robot_base_frame_;
  odom_.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(sim_time);

  odom_pub_->publish(odom_);
}

void NeatoGazeboDiffDrive::PublishWheelTf(
  const gazebo::common::Time & sim_time, gazebo::physics::JointPtr joint)
{
  auto pose_wheel = joint->GetChild()->RelativePose();

  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(sim_time);
  msg.header.frame_id = joint->GetParent()->GetName();
  msg.child_frame_id = joint->GetChild()->GetName();
  msg.transform.translation = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(pose_wheel.Pos());
  msg.transform.rotation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose_wheel.Rot());

  transform_broadcaster_->sendTransform(msg);
}

void NeatoGazeboDiffDrive::PublishOdometryTf(const gazebo::common::Time & sim_time)
{
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(sim_time);
  msg.header.frame_id = odometry_frame_;
  msg.child_frame_id = robot_base_frame_;
  msg.transform.translation =
    gazebo_ros::Convert<geometry_msgs::msg::Vector3>(odom_.pose.pose.position);
  msg.transform.rotation = odom_.pose.pose.orientation;

  transform_broadcaster_->sendTransform(msg);
}

void NeatoGazeboDiffDrive::UpdateWheelJoints(double seconds_since_last_update)
{
  // TODO(neato) check if this logic is actually doing something similar to the robot firmware
  // this should be similar to the robot->motors logic

  const double wheel_radius = wheel_diameter_ / 2.0;

  const double speed_close_enough = 0.0001;
  const double dist_close_enough = 0.0001;
  const double speed_too_fast = 0.280;
  double left_dist_remaining_meters;
  double right_dist_remaining_meters;

  float left_dist, right_dist;

  left_dist = current_cmd_.left_dist / 1000.0f;
  right_dist = current_cmd_.right_dist / 1000.0f;

  const float left_speed = left_wheel_joint_->GetVelocity(0) * wheel_radius;
  const float right_speed = right_wheel_joint_->GetVelocity(0) * wheel_radius;

  float left_target_speed = current_cmd_.speed / 1000.0f;
  float right_target_speed = current_cmd_.speed / 1000.0f;

  float left_accel = current_cmd_.accel / 1000.0f;
  float right_accel = current_cmd_.accel / 1000.0f;

  // left_speed *= (left_dist > 0.0f) ? 1.0f : -1.0f;
  // right_speed *= (right_dist > 0.0f) ? 1.0f : -1.0f;

  // if (fabs(left_dist) >= 0.001f || fabs(right_dist) >= 0.001f) {
  //   RCLCPP_INFO(ros_node_->get_logger(), "left_dist=%f, right_dist=%f\n", left_dist, right_dist);
  // }

  // if (fabs(left_speed) >= speed_close_enough || fabs(right_speed) >= speed_close_enough) {
  //   RCLCPP_INFO(ros_node_->get_logger(), "left_speed=%f, right_speed=%f\n", left_speed, right_speed);
  // }

  {
    const double left_distance_traveled = left_speed * seconds_since_last_update;
    const double right_distance_traveled = right_speed * seconds_since_last_update;

    std::lock_guard<std::mutex> lock(cmd_mutex_);
    if (current_cmd_.left_dist >= 0.0) {
      current_cmd_.left_dist = std::max(0.0, current_cmd_.left_dist - left_distance_traveled * 1000.0);
    } else {
      current_cmd_.left_dist = std::min(0.0, current_cmd_.left_dist - left_distance_traveled * 1000.0);
    }
    left_dist_remaining_meters = current_cmd_.left_dist / 1000.0;
    if (current_cmd_.right_dist >= 0.0) {
      current_cmd_.right_dist = std::max(0.0, current_cmd_.right_dist - right_distance_traveled * 1000.0);
    } else {
      current_cmd_.right_dist = std::min(0.0, current_cmd_.right_dist - right_distance_traveled * 1000.0);
    }
    right_dist_remaining_meters = current_cmd_.right_dist / 1000.0;
    

    if (left_dist != right_dist) {
      double dFrac = fabs(right_dist / left_dist);

      if (dFrac < 1.0) {
        // leftDistance is larger
        right_target_speed *= dFrac;
        right_accel *= dFrac;
      } else {
        left_target_speed /= dFrac;
        left_accel /= dFrac;
      }
    }
  }

  // if (fabs(target_speed_meters_s) >= 0.001f) {
  //   RCLCPP_INFO(ros_node_->get_logger(), "target_speed_meters_s=%f", target_speed_meters_s);
  // }
  if (fabs(left_dist_remaining_meters) >= dist_close_enough ||
      fabs(right_dist_remaining_meters) >= dist_close_enough) {
    RCLCPP_INFO(ros_node_->get_logger(), "lw_remaining=%f, rw_remaining=%f\n",
    left_dist_remaining_meters, right_dist_remaining_meters);
  }

  if (fabs(left_dist_remaining_meters) < dist_close_enough) {
    // Distance accomplished, stop running abruptly..?
    // RCLCPP_INFO(ros_node_->get_logger(), "Approaching destination on left wheel\n");
    left_wheel_joint_->SetParam("vel", 0, 0.0);
  } else if (fabs(left_target_speed - fabs(left_speed)) < speed_close_enough) {
    // Max speed accomplished, stay there
    // RCLCPP_INFO(ros_node_->get_logger(), "Approaching target speed %f on left wheel\n", target_speed_meters_s);
    left_wheel_joint_->SetParam("vel", 0, static_cast<double>(left_speed));
  } else {
    if (left_target_speed >= fabs(left_speed)) {
      if (left_dist_remaining_meters > 0.0) {
        // RCLCPP_INFO(ros_node_->get_logger(), "Increasing speed by %f to match target speed %f on left wheel\n", accel_meters_s_s * seconds_since_last_update, target_speed_meters_s);
        last_sent_left_wheel_speed_ += left_accel * seconds_since_last_update;
      } else {
        // RCLCPP_INFO(ros_node_->get_logger(), "Decreasing speed by %f to match target speed %f on left wheel\n", accel_meters_s_s * seconds_since_last_update, target_speed_meters_s);
        last_sent_left_wheel_speed_ -= left_accel * seconds_since_last_update;
      }
    } else {
      if (left_dist_remaining_meters > 0.0) {
        // RCLCPP_INFO(ros_node_->get_logger(), "Decreasing speed by %f to match target speed %f on left wheel\n", accel_meters_s_s * seconds_since_last_update, target_speed_meters_s);
        last_sent_left_wheel_speed_ -= left_accel * seconds_since_last_update;
      } else {
        // RCLCPP_INFO(ros_node_->get_logger(), "Increasing speed by %f to match target speed %f on left wheel\n", accel_meters_s_s * seconds_since_last_update, target_speed_meters_s);
        last_sent_left_wheel_speed_ += left_accel * seconds_since_last_update;
      }
    }
    if (last_sent_left_wheel_speed_ > speed_too_fast) {
      last_sent_left_wheel_speed_ = speed_too_fast;
    } else if (last_sent_left_wheel_speed_ < -speed_too_fast) {
      last_sent_left_wheel_speed_ = -speed_too_fast;
    }
    left_wheel_joint_->SetParam(
      "vel", 0, last_sent_left_wheel_speed_ / wheel_radius);
  }

  if (fabs(right_dist_remaining_meters) < dist_close_enough) {
    // Distance accomplished, stop running abruptly..?
    // RCLCPP_INFO(ros_node_->get_logger(), "Approaching destination on right wheel\n");
    right_wheel_joint_->SetParam("vel", 0, 0.0);
  } else if (fabs(right_target_speed - fabs(right_speed)) < speed_close_enough) {
    // RCLCPP_INFO(ros_node_->get_logger(), "Approaching target speed %f on right wheel\n", target_speed_meters_s);
    right_wheel_joint_->SetParam("vel", 0, static_cast<double>(right_speed));
  } else {
    if (right_target_speed >= fabs(right_speed)) {
      if (right_dist_remaining_meters > 0.0) {
        // RCLCPP_INFO(ros_node_->get_logger(), "Increasing speed by %f to match target speed %f on right wheel\n", accel_meters_s_s * seconds_since_last_update, target_speed_meters_s);
        last_sent_right_wheel_speed_ += right_accel * seconds_since_last_update;
      } else {
	    // RCLCPP_INFO(ros_node_->get_logger(), "Decreasing speed by %f to match target speed %f on right wheel\n", accel_meters_s_s * seconds_since_last_update, target_speed_meters_s);
        last_sent_right_wheel_speed_ -= right_accel * seconds_since_last_update;
      }
    } else {
      if (right_dist_remaining_meters > 0.0) {
        // RCLCPP_INFO(ros_node_->get_logger(), "Decreasing speed by %f to match target speed %f on right wheel\n", accel_meters_s_s * seconds_since_last_update, target_speed_meters_s);
        last_sent_right_wheel_speed_ -= right_accel * seconds_since_last_update;
      } else {
	    // RCLCPP_INFO(ros_node_->get_logger(), "Increasing speed by %f to match target speed %f on right wheel\n", accel_meters_s_s * seconds_since_last_update, target_speed_meters_s);
        last_sent_right_wheel_speed_ += right_accel * seconds_since_last_update;
      }
    }
    if (last_sent_right_wheel_speed_ > speed_too_fast) {
      last_sent_right_wheel_speed_ = speed_too_fast;
    } else if (last_sent_right_wheel_speed_ < -speed_too_fast) {
      last_sent_right_wheel_speed_ = -speed_too_fast;
    }
    right_wheel_joint_->SetParam(
      "vel", 0, last_sent_right_wheel_speed_ / wheel_radius);
  }

  double lw_speed = left_wheel_joint_->GetParam("vel", 0);
  double rw_speed = right_wheel_joint_->GetParam("vel", 0);
  if (fabs(lw_speed) >= speed_close_enough || fabs(rw_speed) >= speed_close_enough) {
    RCLCPP_INFO(ros_node_->get_logger(), "lw_speed=%f, rw_speed=%f\n", lw_speed, rw_speed);
  }
}

GZ_REGISTER_MODEL_PLUGIN(NeatoGazeboDiffDrive)
}  // namespace neato_gazebo
