// Copyright 2023 RT Corporation
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
// https://www.opencv-srf.com/2010/09/object-detection-using-color-seperation.html

#ifndef COMPOSITION__NECK_JT_CONTROL_HPP_
#define COMPOSITION__NECK_JT_CONTROL_HPP_

#include <cmath>
#include <iostream>
#include <iomanip>
#include <memory>
#include "angles/angles.h"

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
using std::placeholders::_1;
using namespace std::chrono_literals;

namespace sciurus17_examples
{

class NeckJtControl : public rclcpp::Node
{
public:
  explicit NeckJtControl(const rclcpp::NodeOptions & options);

private:
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr client_ptr_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr angles_subscription_;
  void angles_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
};

}  // namespace sciurus17_examples

#endif // COMPOSITION__NECK_JT_CONTROL_HPP_