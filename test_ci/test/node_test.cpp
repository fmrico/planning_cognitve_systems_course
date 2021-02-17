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

#include <memory>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "test_ci/TestNode.hpp"

TEST(test_node, subscription)
{
  auto node = std::make_shared<test_ci::TestNode>();

  auto test_node = rclcpp::Node::make_shared("test_node");
  auto test_pub = test_node->create_publisher<std_msgs::msg::String>(
    "/message", 10);


  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(node);
  exe.add_node(test_node);

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });

  ASSERT_EQ("", node->get_last_msg());

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  std_msgs::msg::String msg;
  msg.data = "hello";
  test_pub->publish(msg);

  {
    rclcpp::Rate rate(10);
    auto start = test_node->now();
    while ((test_node->now() - start).seconds() < 0.5) {
      rate.sleep();
    }
  }

  ASSERT_EQ("hello____", node->get_last_msg());

  finish = true;
  t.join();
}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
