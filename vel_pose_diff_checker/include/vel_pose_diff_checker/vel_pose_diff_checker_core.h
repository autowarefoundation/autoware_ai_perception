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

#ifndef VEL_POSE_DIFF_CHECKER_VEL_POSE_DIFF_CHECKER_CORE_H
#define VEL_POSE_DIFF_CHECKER_VEL_POSE_DIFF_CHECKER_CORE_H

#include <string>
#include <deque>
#include <utility>

#include <ros/ros.h>
#include <tf/tf.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <autoware_health_checker/health_checker/health_checker.h>

#include "vel_pose_diff_checker/value_time_queue.h"

class VelPoseDiffChecker
{
  friend class VelPoseDiffCheckerTestSuite;
  struct CheckValues
  {
    double diff_pose_topic_time = 0.0;
    double diff_twist_topic_time = 0.0;
    double diff_position = 0.0;
    double diff_position_median = 0.0;
    double diff_angle = 0.0;
    double diff_angle_median = 0.0;
  };

  struct CheckValueThresholds
  {
    std::string key;
    double warn_value;
    double error_value;
    double fatal_value;
    std::string error_description;
  };

public:
  VelPoseDiffChecker(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);

private:
  void callbackPose(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void callbackTwist(const geometry_msgs::TwistStamped::ConstPtr& msg);
  void callbackTimer(const ros::TimerEvent& e);
  CheckValues checkDiff();
  uint8_t getErrorLevelWithHealthChecker(const CheckValues& check_values);
  void publishCheckValues(const CheckValues& check_values);
  std::pair<tf::Transform, ros::Time> calcDeltaTfPoseAndTime(
    const boost::shared_ptr<const geometry_msgs::PoseStamped>& current_pose_ptr,
    const ros::Time& ros_time_now, const ros::Time& ros_time_past);
  std::deque<boost::shared_ptr<const geometry_msgs::TwistStamped>>::iterator
    searchCurrentTwistPtrItr(const ros::Time& pose_time_current);
  tf::Transform calcDeltaTfOdom(
    const std::deque<boost::shared_ptr<const geometry_msgs::TwistStamped>>::iterator& current_twist_ptr_it,
    const ros::Time& pose_time_past);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Subscriber pose_sub_;
  ros::Subscriber twist_sub_;
  ros::Timer timer_;

  ros::Publisher diff_position_pub_;
  ros::Publisher diff_position_median_pub_;
  ros::Publisher diff_angle_rad_pub_;
  ros::Publisher diff_angle_rad_median_pub_;
  ros::Publisher diff_angle_deg_pub_;
  ros::Publisher diff_angle_deg_median_pub_;

  autoware_health_checker::HealthChecker health_checker_;

  ValueTimeQueue dist_time_queue_;
  ValueTimeQueue angle_time_queue_;
  std::deque<boost::shared_ptr<const geometry_msgs::PoseStamped> > pose_ptr_queue_;
  std::deque<boost::shared_ptr<const geometry_msgs::TwistStamped> > twist_ptr_queue_;

  double loop_rate_hz_;
  double comparison_window_size_sec_;
  double moving_median_window_size_sec_;

  CheckValueThresholds diff_pose_topic_time_thresholds_;
  CheckValueThresholds diff_twist_topic_time_thresholds_;
  CheckValueThresholds diff_position_thresholds_;
  CheckValueThresholds diff_position_median_thresholds_;
  CheckValueThresholds diff_angle_thresholds_;
  CheckValueThresholds diff_angle_median_thresholds_;
};

#endif  // VEL_POSE_DIFF_CHECKER_VEL_POSE_DIFF_CHECKER_CORE_H
