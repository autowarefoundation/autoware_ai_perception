/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Authors: Kenji Miyake
 */

#include <gtest/gtest.h>

#include <ros/ros.h>

class TestSuite : public ::testing::Test
{
public:
  TestSuite() = default;
  ~TestSuite() = default;
};

bool isNodeFound()
{
  std::vector<std::string> node_names;
  while (true)
  {
    // Get node names
    ros::master::getNodes(node_names);

    // Wait for ndt_matching node
    const auto itr = std::find(std::begin(node_names), std::end(node_names), "/ndt_matching");

    // Break if found
    if (itr != std::end(node_names))
    {
      break;
    }
  }

  return true;
}

TEST_F(TestSuite, launch_ndt_matching)
{
  ASSERT_TRUE(isNodeFound());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "TestNode");
  return RUN_ALL_TESTS();
}
