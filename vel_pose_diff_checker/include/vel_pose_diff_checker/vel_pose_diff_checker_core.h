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

#include <deque>

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include "vel_pose_diff_checker/value_time_queue.h"


class VelPoseDiffChecker
{
  friend class VelPoseDiffCheckerTestSuite;

  public:
    enum class Status { ERROR = 0, OK = 1, INVALID = 2 };

    VelPoseDiffChecker(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
    void run();

  private:
    void callbackPose(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void callbackTwist(const geometry_msgs::TwistStamped::ConstPtr& msg);
    VelPoseDiffChecker::Status checkDiff();

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber pose_sub_;
    ros::Subscriber twist_sub_;

    ros::Publisher diff_position_pub_;
    ros::Publisher diff_position_median_pub_;
    ros::Publisher diff_angle_rad_pub_;
    ros::Publisher diff_angle_rad_median_pub_;
    ros::Publisher diff_angle_deg_pub_;
    ros::Publisher diff_angle_deg_median_pub_;
    ros::Publisher remote_cmd_pub_;

    ValueTimeQueue dist_time_queue_;
    ValueTimeQueue angle_time_queue_;
    std::deque< boost::shared_ptr<const geometry_msgs::PoseStamped> > pose_ptr_queue_;
    std::deque< boost::shared_ptr<const geometry_msgs::TwistStamped> > twist_ptr_queue_;

    double loop_rate_hz_;
    double comparison_window_size_sec_;
    double topic_timeout_sec_;
    double moving_median_window_size_sec_;

    double diff_position_threshold_meter_;
    double diff_position_median_threshold_meter_;
    double diff_angle_threshold_rad_;
    double diff_angle_median_threshold_rad_;

    bool enable_emergency_to_twist_gate_;
};

#endif  // VEL_POSE_DIFF_CHECKER_VEL_POSE_DIFF_CHECKER_CORE_H
