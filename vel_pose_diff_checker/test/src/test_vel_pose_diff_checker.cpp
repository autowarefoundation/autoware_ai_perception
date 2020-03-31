/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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
 */

#include <gtest/gtest.h>

#include <vector>
#include <memory>

#include <ros/ros.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include "vel_pose_diff_checker/vel_pose_diff_checker_core.h"

class VelPoseDiffCheckerTestSuite : public ::testing::Test
{
  public:
    VelPoseDiffCheckerTestSuite()
      : nh_()
      , private_nh_("~")
    {
    }
    ~VelPoseDiffCheckerTestSuite()
    {
    }
    uint8_t getErrorLevel()
    {
      return obj_ptr_->getErrorLevelWithHealthChecker(obj_ptr_->checkDiff());
    }
    std::unique_ptr<VelPoseDiffChecker> obj_ptr_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

  protected:
    virtual void SetUp()
    {
      nh_.setParam("/health_checker/vel_pose_diff_checker_diff_position", "default");
      obj_ptr_ = std::make_unique<VelPoseDiffChecker>(nh_, private_nh_);
    };
    virtual void TearDown()
    {
      obj_ptr_.reset();
    };
};

TEST_F(VelPoseDiffCheckerTestSuite, checkDiff)
{
  ros::Publisher pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("/current_pose", 1);
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.frame_id = "map";
  pose_msg.pose.position.x = 0.0;
  pose_msg.pose.position.y = 0.0;
  pose_msg.pose.position.z = 0.0;
  pose_msg.pose.orientation.x = 0.0;
  pose_msg.pose.orientation.y = 0.0;
  pose_msg.pose.orientation.z = 0.0;
  pose_msg.pose.orientation.w = 1.0;

  ros::Publisher twist_pub = nh_.advertise<geometry_msgs::TwistStamped>("/current_velocity", 1);
  geometry_msgs::TwistStamped twist_msg;
  twist_msg.header.frame_id = "base_link";
  twist_msg.twist.linear.x = 0.0;
  twist_msg.twist.linear.y = 0.0;
  twist_msg.twist.linear.z = 0.0;
  twist_msg.twist.angular.x = 0.0;
  twist_msg.twist.angular.y = 0.0;
  twist_msg.twist.angular.z = 0.0;

  constexpr double dt = 0.1;
  constexpr double linear_x = 1.0;
  constexpr double noise = 1.0;

  // TEST No Difference
  for (size_t i = 0; i < 10; ++i)
  {
    const ros::Time ros_time_now = ros::Time::now();

    pose_msg.header.stamp = ros_time_now;
    pose_msg.pose.position.x += linear_x*dt;
    pose_pub.publish(pose_msg);

    twist_msg.header.stamp = ros_time_now;
    twist_msg.twist.linear.x = linear_x;
    twist_pub.publish(twist_msg);

    ros::spinOnce();
    ros::Duration(dt).sleep();
  }
  ASSERT_EQ(getErrorLevel(), autoware_system_msgs::DiagnosticStatus::OK);

  // TEST Detect Difference
  for (size_t i = 0; i < 5; ++i)
  {
    const ros::Time ros_time_now = ros::Time::now();

    pose_msg.header.stamp = ros_time_now;
    pose_msg.pose.position.x += linear_x*dt + noise;
    pose_pub.publish(pose_msg);

    twist_msg.header.stamp = ros_time_now;
    twist_msg.twist.linear.x = linear_x;
    twist_pub.publish(twist_msg);

    ros::spinOnce();
    ros::Duration(dt).sleep();
  }
  ASSERT_EQ(getErrorLevel(), autoware_system_msgs::DiagnosticStatus::ERROR);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "VelPoseDiffCheckerTestSuite");

  return RUN_ALL_TESTS();
}
