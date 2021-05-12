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
#include "gazebo/physics/Joint.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/World.hh"

#include "gazebo_ros/node.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "neato_msgs/msg/neato_wheel_command.hpp"
#include "rclcpp/rclcpp.hpp"
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
  /// \param[in] info Updated simulation info.
  void OnUpdate(const gazebo::common::UpdateInfo & info);

  /// Callback when a velocity command is received.
  /// \param[in] msg command message.
  void OnCmd(const neato_msgs::msg::NeatoWheelCommand::SharedPtr msg);

private:
  struct Pose2D
  {
    double x;
    double y;
    double theta;
  };

  // configuration
  double wheel_diameter_;
  double max_wheel_accel_;
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
  std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

  // dynamic tracking variables
  gazebo::common::Time last_update_time_;
  gazebo::common::Time last_encoder_update_;
  Pose2D pose_encoder_;
};

void NeatoGazeboDiffDrive::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  model_ = model;
  ros_node_ = gazebo_ros::Node::Get(sdf);

  // Initialize general configuration
  wheel_diameter_ = sdf->Get<double>("wheel_diameter", 0.070).first;
  max_wheel_accel_ = sdf->Get<double>("max_wheel_acceleration", 0.0).first;
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
  last_update_time_ = model->GetWorld()->SimTime();
}

void NeatoGazeboDiffDrive::Reset()
{
  last_update_time_ = left_wheel_joint_->GetWorld()->SimTime();
  left_wheel_joint_->SetParam("fmax", 0, max_wheel_torque_);
  right_wheel_joint_->SetParam("fmax", 0, max_wheel_torque_);
  pose_encoder_.x = 0;
  pose_encoder_.y = 0;
  pose_encoder_.theta = 0;
  // TODO(neato)
  // target_x_ = 0;
  // target_rot_ = 0;
}

void NeatoGazeboDiffDrive::OnUpdate(const gazebo::common::UpdateInfo & info)
{
  // TODO(neato)
}

void NeatoGazeboDiffDrive::OnCmd(const neato_msgs::msg::NeatoWheelCommand::SharedPtr msg)
{
  // TODO(neato)
}

GZ_REGISTER_MODEL_PLUGIN(NeatoGazeboDiffDrive)
}  // namespace neato_gazebo
