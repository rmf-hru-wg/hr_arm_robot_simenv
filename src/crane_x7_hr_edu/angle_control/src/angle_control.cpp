// Copyright 2022 RT Corporation
//
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

// Reference:
// https://github.com/ros-planning/moveit2_tutorials/blob
// /a547cf49ff7d1fe16a93dfe020c6027bcb035b51/doc/move_group_interface
// /src/move_group_interface_tutorial.cpp

#include <chrono>

#include "angles/angles.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "angle_control_interfaces/msg/angle_control.hpp"

using namespace std::chrono_literals;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

// static const rclcpp::Logger LOGGER = rclcpp::get_logger("joint_values");

class AngleSubscriber : public rclcpp::Node
{
public:
  AngleSubscriber(
    rclcpp::Node::SharedPtr move_group_arm_node,
    rclcpp::Node::SharedPtr move_group_gripper_node)
  : Node("angle_subscriber_node")
  {
    using namespace std::placeholders;
    move_group_arm_ = std::make_shared<MoveGroupInterface>(move_group_arm_node, "arm");
    move_group_arm_ -> setMaxVelocityScalingFactor(0.7);
    move_group_arm_ -> setMaxAccelerationScalingFactor(0.7);

    move_group_gripper_ = std::make_shared<MoveGroupInterface>(move_group_gripper_node, "gripper");
    move_group_gripper_ -> setMaxVelocityScalingFactor(1.0);
    move_group_gripper_ -> setMaxAccelerationScalingFactor(1.0);

    move_group_arm_ -> setNamedTarget("home");
    move_group_arm_ -> move();

    subscription_ = this->create_subscription<angle_control_interfaces::msg::AngleControl>(
      "angle_control",
      10,
      std::bind(&AngleSubscriber::angle_control_callback, this, std::placeholders::_1)
    );
  }

private:
  void angle_control_callback(const angle_control_interfaces::msg::AngleControl & msg) const
  {
    // move to given angle
    auto joint_values = move_group_arm_ -> getCurrentJointValues();
    auto gripper_joint_values = move_group_gripper_ -> getCurrentJointValues();

    double target_joint_values[7] = {
      angles::from_degrees(msg.joint0),
      angles::from_degrees(msg.joint1),
      angles::from_degrees(msg.joint2),
      angles::from_degrees(msg.joint3),
      angles::from_degrees(msg.joint4),
      angles::from_degrees(msg.joint5),
      angles::from_degrees(msg.joint6),
    };

    for(int i=0; i<6; i++){
      joint_values[i] = target_joint_values[i];
    }
    gripper_joint_values[0] = angles::from_degrees(msg.grip);

    move_group_arm_ -> setJointValueTarget(joint_values);
    move_group_arm_ -> move();
    move_group_gripper_ -> setJointValueTarget(gripper_joint_values);
    move_group_gripper_ -> move();
  }
  
  std::shared_ptr<MoveGroupInterface> move_group_arm_;
  std::shared_ptr<MoveGroupInterface> move_group_gripper_;
  rclcpp::Subscription<angle_control_interfaces::msg::AngleControl>::SharedPtr subscription_;

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_arm_node = rclcpp::Node::make_shared("move_group_arm_node", node_options);
  auto move_group_gripper_node = rclcpp::Node::make_shared("move_group_gripper_node", node_options);
  // // For current state monitor
  // rclcpp::executors::SingleThreadedExecutor executor;
  // executor.add_node(move_group_node);
  // executor.add_node(move_group_gripper_node);
  // std::thread([&executor]() {executor.spin();}).detach();

  // MoveGroupInterface move_group_arm(move_group_node, "arm");
  // // 駆動速度を調整する
  // move_group_arm.setMaxVelocityScalingFactor(0.8);  // Set 0.0 ~ 1.0
  // move_group_arm.setMaxAccelerationScalingFactor(0.8);  // Set 0.0 ~ 1.0

  // MoveGroupInterface move_group_gripper(move_group_gripper_node, "gripper");
  // move_group_gripper.setMaxVelocityScalingFactor(1.0);  // Set 0.0 ~ 1.0
  // move_group_gripper.setMaxAccelerationScalingFactor(1.0);  // Set 0.0 ~ 1.0

  // // SRDFに定義されている"vertical"の姿勢にする
  // // すべてのジョイントの目標角度が0度になる
  // move_group_arm.setNamedTarget("vertical");
  // move_group_arm.move();

  rclcpp::executors::MultiThreadedExecutor exec;
  auto angle_subscriber_node = std::make_shared<AngleSubscriber>(
    move_group_arm_node,
    move_group_gripper_node);
  exec.add_node(angle_subscriber_node);
  exec.add_node(move_group_arm_node);
  exec.add_node(move_group_gripper_node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
