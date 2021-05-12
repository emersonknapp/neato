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

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/Joint.hh"
#include "gazebo/physics/Model.hh"

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
  NeatoGazeboDiffDrive();
  virtual ~NeatoGazeboDiffDrive();

protected:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;
  void Reset() override;

private:
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<neato_msgs::msg::NeatoWheelCommand>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  gazebo::event::ConnectionPtr update_connection_;
  double wheel_diameter_;
  gazebo::physics::JointPtr left_wheel_joint_;
  gazebo::physics::JointPtr right_wheel_joint_;
  gazebo::physics::ModelPtr model_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
  std::string odometry_frame_;
};


NeatoGazeboDiffDrive::NeatoGazeboDiffDrive()
{}

NeatoGazeboDiffDrive::~NeatoGazeboDiffDrive()
{}

void NeatoGazeboDiffDrive::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{

}

void NeatoGazeboDiffDrive::Reset()
{

}

}  // namespace neato_gazebo
