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

#include "gtest/gtest.h"

#include "blackboard/BlackBoard.hpp"

TEST(blackboard, add_get_entry)
{
  auto blackboard = blackboard::BlackBoard::make_shared();

  auto entry_1 = blackboard::Entry<bool>::make_shared(true);

  auto entry_base = entry_1->to_base();
  auto entry_2 = blackboard::Entry<std::string>::make_shared("Hi!!");

  blackboard->add_entry("my_entry_1", entry_1->to_base());
  blackboard->add_entry("my_entry_2", entry_2->to_base());

  auto entry_1_got = blackboard::as<bool>(blackboard->get_entry("my_entry_1"));
  auto entry_2_got = blackboard::as<std::string>(blackboard->get_entry("my_entry_2"));

  ASSERT_TRUE(entry_1_got->data_);
  ASSERT_EQ(entry_2_got->data_, "Hi!!");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
