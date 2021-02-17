// Copyright 2021 Intelligent Robotics Lab
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

 #include <string>

#include "test_ci/TestNode.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace test_ci
{

using std::placeholders::_1;

TestNode::TestNode()
: Node("test_ci")
{
  sub_ = create_subscription<std_msgs::msg::String>("/message", 10, std::bind(&TestNode::string_callback, this, _1));
}

std::string
TestNode::get_last_msg() const
{
  return last_msg_;
}

void
TestNode::string_callback(const std_msgs::msg::String::ConstSharedPtr msg)
{
  last_msg_ = msg->data;
}


}  // namespace test_ci
