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

#include "vel_pose_diff_checker/vel_pose_diff_checker_core.h"

#include <iostream>
#include <deque>
#include <algorithm>
#include <utility>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <autoware_msgs/RemoteCmd.h>
#include <autoware_system_msgs/DiagnosticStatus.h>

#include <amathutils_lib/amathutils.hpp>

// referenced by lane_select
tf::Point convertPointIntoRelativeCoordinate(const geometry_msgs::Point& input_point, const geometry_msgs::Pose& pose)
{
  tf::Transform inverse;
  tf::poseMsgToTF(pose, inverse);
  tf::Transform transform = inverse.inverse();

  tf::Point p;
  pointMsgToTF(input_point, p);
  tf::Point tf_p = transform * p;
  return tf_p;
}

VelPoseDiffChecker::VelPoseDiffChecker(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
  : nh_(nh)
  , private_nh_(private_nh)
  , health_checker_(nh, private_nh)
  , loop_rate_hz_(10.0)
  , comparison_window_size_sec_(1.0)
  , moving_median_window_size_sec_(2.0)
  , diff_pose_topic_time_thresholds_
    {
      "vel_pose_diff_checker_diff_pose_topic_time", 0.24, 0.3, DBL_MAX,
      "[vel_pose_diff_checker] pose topic is timeout"
    }
  , diff_twist_topic_time_thresholds_
    {
      "vel_pose_diff_checker_diff_twist_topic_time", 0.24, 0.3, DBL_MAX,
      "[vel_pose_diff_checker] twist topic is timeout"
    }
  , diff_position_thresholds_
    {
      "vel_pose_diff_checker_diff_position", 0.8, 1.0, DBL_MAX,
      "[vel_pose_diff_checker] detect position difference"
    }
  , diff_position_median_thresholds_
    {
      "vel_pose_diff_checker_diff_position_median", 0.4, 0.5, DBL_MAX,
      "[vel_pose_diff_checker] detect position_median difference"
    }
  , diff_angle_thresholds_
    {
      "vel_pose_diff_checker_diff_angle", 0.08, 0.1, DBL_MAX,
      "[vel_pose_diff_checker] detect angle difference"
    }
  , diff_angle_median_thresholds_
    {
      "vel_pose_diff_checker_diff_angle_median", 0.04, 0.05, DBL_MAX,
      "[vel_pose_diff_checker] detect angle_median difference"
    }
{
  private_nh_.getParam("loop_rate_hz", loop_rate_hz_);
  private_nh_.getParam("comparison_window_size_sec", comparison_window_size_sec_);

  private_nh_.getParam("moving_median_window_size_sec", moving_median_window_size_sec_);
  dist_time_queue_.setWindowSizeSec(moving_median_window_size_sec_);
  angle_time_queue_.setWindowSizeSec(moving_median_window_size_sec_);

  private_nh_.getParam("topic_timeout_sec", diff_pose_topic_time_thresholds_.error_value);
  private_nh_.getParam("topic_timeout_sec", diff_twist_topic_time_thresholds_.error_value);
  private_nh_.getParam("diff_position_threshold_meter", diff_position_thresholds_.error_value);
  private_nh_.getParam("diff_position_median_threshold_meter", diff_position_median_thresholds_.error_value);
  private_nh_.getParam("diff_angle_threshold_rad", diff_angle_thresholds_.error_value);
  private_nh_.getParam("diff_angle_median_threshold_rad", diff_angle_median_thresholds_.error_value);

  constexpr double error_to_warn_value_ratio = 0.8;
  std::vector<CheckValueThresholds*> thresholds
  {
    &diff_pose_topic_time_thresholds_, &diff_twist_topic_time_thresholds_,
    &diff_position_thresholds_,        &diff_position_median_thresholds_,
    &diff_angle_thresholds_,           &diff_angle_median_thresholds_
  };

  for (auto* th : thresholds)
  {
    th->warn_value = th->error_value * error_to_warn_value_ratio;
  }

  pose_sub_ = nh_.subscribe("current_pose", 10, &VelPoseDiffChecker::callbackPose, this,
                            ros::TransportHints().tcpNoDelay(true));
  twist_sub_ = nh_.subscribe("current_velocity", 10, &VelPoseDiffChecker::callbackTwist, this,
                             ros::TransportHints().tcpNoDelay(true));
  timer_ = nh_.createTimer(ros::Rate(loop_rate_hz_), &VelPoseDiffChecker::callbackTimer, this);

  // for visualize
  diff_position_pub_ = private_nh_.advertise<std_msgs::Float32>("diff_position", 10);
  diff_position_median_pub_ = private_nh_.advertise<std_msgs::Float32>("diff_position_median", 10);
  diff_angle_rad_pub_ = private_nh_.advertise<std_msgs::Float32>("diff_angle_rad", 10);
  diff_angle_rad_median_pub_ = private_nh_.advertise<std_msgs::Float32>("diff_angle_rad_median", 10);
  diff_angle_deg_pub_ = private_nh_.advertise<std_msgs::Float32>("diff_angle_deg", 10);
  diff_angle_deg_median_pub_ = private_nh_.advertise<std_msgs::Float32>("diff_angle_deg_median", 10);

  health_checker_.ENABLE();
}

void VelPoseDiffChecker::callbackTimer(const ros::TimerEvent& e)
{
  const CheckValues check_values = checkDiff();
  const auto error_level = getErrorLevelWithHealthChecker(check_values);
  if (error_level >= autoware_system_msgs::DiagnosticStatus::ERROR)
  {
    ROS_WARN("Difference Detection!!!!!");
    ROS_WARN("diff_pose_topic_time %lf[sec], diff_twist_topic_time %lf[sec], diff_pos %lf[m], diff_pos_median "
             "%lf[m], diff_ang %lf[rad], diff_ang_median %lf[rad]"  // NOLINT
             ,
             check_values.diff_pose_topic_time, check_values.diff_twist_topic_time, check_values.diff_position,
             check_values.diff_position_median, check_values.diff_angle, check_values.diff_angle_median);
  }
  else
  {
    ROS_DEBUG("diff_pose_topic_time %lf[sec], diff_twist_topic_time %lf[sec], diff_pos %lf[m], diff_pos_median "
              "%lf[m], diff_ang %lf[rad], diff_ang_median %lf[rad]"  // NOLINT
              ,
              check_values.diff_pose_topic_time, check_values.diff_twist_topic_time, check_values.diff_position,
              check_values.diff_position_median, check_values.diff_angle, check_values.diff_angle_median);
  }

  publishCheckValues(check_values);
}

void VelPoseDiffChecker::callbackPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  pose_ptr_queue_.push_back(msg);

  while (!pose_ptr_queue_.empty())
  {
    // for replay rosbag
    if (pose_ptr_queue_.front()->header.stamp > msg->header.stamp)
    {
      pose_ptr_queue_.pop_front();
    }
    else if (pose_ptr_queue_.front()->header.stamp <
             msg->header.stamp - ros::Duration(comparison_window_size_sec_ * 2.0))
    {
      pose_ptr_queue_.pop_front();
    }
    else
    {
      break;
    }
  }
}

void VelPoseDiffChecker::callbackTwist(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  twist_ptr_queue_.push_back(msg);

  while (!twist_ptr_queue_.empty())
  {
    // for replay rosbag
    if (twist_ptr_queue_.front()->header.stamp > msg->header.stamp)
    {
      twist_ptr_queue_.pop_front();
    }
    else if (twist_ptr_queue_.front()->header.stamp <
             msg->header.stamp - ros::Duration(comparison_window_size_sec_ * 2.0))
    {
      twist_ptr_queue_.pop_front();
    }
    else
    {
      break;
    }
  }
}

std::pair<tf::Transform, ros::Time> VelPoseDiffChecker::calcDeltaTfPoseAndTime(
  const boost::shared_ptr<const geometry_msgs::PoseStamped>& current_pose_ptr,
  const ros::Time& ros_time_now, const ros::Time& ros_time_past)
{
  tf::Transform delta_tf_pose(tf::Quaternion(0.0, 0.0, 0.0, 1.0), tf::Vector3(0.0, 0.0, 0.0));
  // NOTE: ros_time_past-ros::Duration(0.05) -> To stabilize the acquisition time
  const auto past_pose_ptr_it =
    std::lower_bound(std::begin(pose_ptr_queue_), std::end(pose_ptr_queue_), ros_time_past - ros::Duration(0.05),
      [](const boost::shared_ptr<const geometry_msgs::PoseStamped>& x_ptr, ros::Time t)
    {
      return x_ptr->header.stamp < t;
    });  // NOLINT
  const auto past_pose_ptr =
      past_pose_ptr_it == pose_ptr_queue_.end() ? *(pose_ptr_queue_.end() - 1) : *past_pose_ptr_it;

  delta_tf_pose.setOrigin(convertPointIntoRelativeCoordinate(current_pose_ptr->pose.position, past_pose_ptr->pose));
  tf::Quaternion current_pose_orientation, past_pose_orientation;
  tf::quaternionMsgToTF(current_pose_ptr->pose.orientation, current_pose_orientation);
  tf::quaternionMsgToTF(past_pose_ptr->pose.orientation, past_pose_orientation);
  delta_tf_pose.setRotation(past_pose_orientation.inverse() * current_pose_orientation);
  std::pair<tf::Transform, ros::Time> pair = std::make_pair(delta_tf_pose, past_pose_ptr->header.stamp);
  return pair;
}

std::deque<boost::shared_ptr<const geometry_msgs::TwistStamped>>::iterator
  VelPoseDiffChecker::searchCurrentTwistPtrItr(const ros::Time& pose_time_current)
{
  auto current_twist_ptr_it =
    std::lower_bound(std::begin(twist_ptr_queue_), std::end(twist_ptr_queue_), pose_time_current,
    [](const boost::shared_ptr<const geometry_msgs::TwistStamped>& x_ptr, ros::Time t)
    {
      return x_ptr->header.stamp < t;
    });  // NOLINT
  current_twist_ptr_it =
    current_twist_ptr_it == twist_ptr_queue_.end() ? (twist_ptr_queue_.end() - 1) : current_twist_ptr_it;
  return current_twist_ptr_it;
}

tf::Transform VelPoseDiffChecker::calcDeltaTfOdom(
  const std::deque<boost::shared_ptr<const geometry_msgs::TwistStamped>>::iterator& current_twist_ptr_it,
  const ros::Time& pose_time_past)
{
  tf::Transform delta_tf_odom(tf::Quaternion(0.0, 0.0, 0.0, 1.0), tf::Vector3(0.0, 0.0, 0.0));
  auto past_twist_ptr_it = std::lower_bound(
    std::begin(twist_ptr_queue_), std::end(twist_ptr_queue_), pose_time_past,
    [](const boost::shared_ptr<const geometry_msgs::TwistStamped>& x_ptr, ros::Time t)
    {
      return x_ptr->header.stamp < t;
    });  // NOLINT
  past_twist_ptr_it = past_twist_ptr_it == twist_ptr_queue_.end() ? twist_ptr_queue_.end() - 1 : past_twist_ptr_it;

  ros::Time prev_time = (*past_twist_ptr_it)->header.stamp;
  double x = 0.0, y = 0.0, yaw = 0.0;
  for (auto twist_ptr_it = past_twist_ptr_it + 1; twist_ptr_it != current_twist_ptr_it + 1; ++twist_ptr_it)
  {
    const double dt = ((*twist_ptr_it)->header.stamp - prev_time).toSec();
    const double dis = (*twist_ptr_it)->twist.linear.x * dt;
    yaw += (*twist_ptr_it)->twist.angular.z * dt;
    x += dis * std::cos(yaw);
    y += dis * std::sin(yaw);
    prev_time = (*twist_ptr_it)->header.stamp;
  }

  delta_tf_odom.setOrigin(tf::Vector3(x, y, 0.0));
  tf::Quaternion tf_quat;
  tf_quat.setRPY(0.0, 0.0, yaw);
  delta_tf_odom.setRotation(tf_quat);
  return delta_tf_odom;
}

VelPoseDiffChecker::CheckValues VelPoseDiffChecker::checkDiff()
{
  const auto ros_time_now = ros::Time::now();
  const auto ros_time_past = ros_time_now - ros::Duration(comparison_window_size_sec_);

  auto pose_time_current = ros_time_now;
  auto pose_time_past = ros_time_past;

  tf::Transform delta_tf_pose(tf::Quaternion(0.0, 0.0, 0.0, 1.0), tf::Vector3(0.0, 0.0, 0.0));
  double diff_pose_topic_time = 0;

  if (pose_ptr_queue_.empty())
  {
    ROS_WARN_STREAM_THROTTLE(1, "Pose is not subscribed yet");
  }
  else
  {
    const auto current_pose_ptr = pose_ptr_queue_.back();
    const auto delta_pose_and_time = calcDeltaTfPoseAndTime(current_pose_ptr, ros_time_now, ros_time_past);
    delta_tf_pose = delta_pose_and_time.first;
    const auto& past_pose_stamp = delta_pose_and_time.second;

    diff_pose_topic_time = std::abs((ros_time_now - current_pose_ptr->header.stamp).toSec());
    if (diff_pose_topic_time > diff_pose_topic_time_thresholds_.error_value)
    {
      ROS_WARN("current_pose.header.stamp is late. ros_time_now is %lf. current_pose.header.stamp is %lf",
               ros_time_now.toSec(), current_pose_ptr->header.stamp.toSec());
    }
    else
    {
      pose_time_current = current_pose_ptr->header.stamp;
      pose_time_past = past_pose_stamp;
    }
  }

  tf::Transform delta_tf_odom(tf::Quaternion(0.0, 0.0, 0.0, 1.0), tf::Vector3(0.0, 0.0, 0.0));
  double diff_twist_topic_time = 0;

  if (twist_ptr_queue_.empty())
  {
    ROS_WARN_STREAM_THROTTLE(1, "Twist is not subscribed yet");
  }
  else
  {
    auto current_twist_ptr_it = searchCurrentTwistPtrItr(pose_time_current);

    diff_twist_topic_time = std::abs((ros_time_now - (*current_twist_ptr_it)->header.stamp).toSec());
    if (diff_twist_topic_time > diff_twist_topic_time_thresholds_.error_value)
    {
      ROS_WARN("current_twist.header.stamp is late. ros_time_now is %lf. current_twist.header.stamp is %lf",
               ros_time_now.toSec(), (*current_twist_ptr_it)->header.stamp.toSec());
    }
    delta_tf_odom = calcDeltaTfOdom(current_twist_ptr_it, pose_time_past);
  }

  CheckValues check_values;
  check_values.diff_pose_topic_time = diff_pose_topic_time;
  check_values.diff_twist_topic_time = diff_twist_topic_time;

  check_values.diff_position = (delta_tf_pose.getOrigin() - delta_tf_odom.getOrigin()).length();
  check_values.diff_angle = (delta_tf_odom.getRotation().inverse() * delta_tf_pose.getRotation()).getAngle();

  dist_time_queue_.addValueTime(check_values.diff_position, ros_time_now);
  check_values.diff_position_median = dist_time_queue_.getMedianValue();

  angle_time_queue_.addValueTime(check_values.diff_angle, ros_time_now);
  check_values.diff_angle_median = angle_time_queue_.getMedianValue();

  return check_values;
}

uint8_t VelPoseDiffChecker::getErrorLevelWithHealthChecker(const CheckValues& check_values)
{
  const auto health_check_max_value_func = [this](const double check_value, const CheckValueThresholds& thresholds)
  {
    return health_checker_.CHECK_MAX_VALUE(thresholds.key, check_value, thresholds.warn_value, thresholds.error_value,
                                           thresholds.fatal_value, thresholds.error_description);
  };

  health_checker_.NODE_ACTIVATE();
  const auto a_error_level =
      health_check_max_value_func(check_values.diff_pose_topic_time, diff_pose_topic_time_thresholds_);
  const auto b_error_level =
      health_check_max_value_func(check_values.diff_twist_topic_time, diff_twist_topic_time_thresholds_);
  const auto c_error_level = health_check_max_value_func(check_values.diff_position, diff_position_thresholds_);
  const auto d_error_level =
      health_check_max_value_func(check_values.diff_position_median, diff_position_median_thresholds_);
  const auto e_error_level = health_check_max_value_func(check_values.diff_angle, diff_angle_thresholds_);
  const auto f_error_level = health_check_max_value_func(check_values.diff_angle_median, diff_angle_median_thresholds_);

  return std::max({ a_error_level, b_error_level, c_error_level,     // NOLINT
                    d_error_level, e_error_level, f_error_level });  // NOLINT
}

void VelPoseDiffChecker::publishCheckValues(const CheckValues& check_values)
{
  std_msgs::Float32 diff_position_msg;
  diff_position_msg.data = check_values.diff_position;
  diff_position_pub_.publish(diff_position_msg);

  std_msgs::Float32 diff_position_median_msg;
  diff_position_median_msg.data = check_values.diff_position_median;
  diff_position_median_pub_.publish(diff_position_median_msg);

  std_msgs::Float32 diff_angle_rad_msg;
  diff_angle_rad_msg.data = check_values.diff_angle;
  diff_angle_rad_pub_.publish(diff_angle_rad_msg);

  std_msgs::Float32 diff_angle_rad_median_msg;
  diff_angle_rad_median_msg.data = check_values.diff_angle_median;
  diff_angle_rad_median_pub_.publish(diff_angle_rad_median_msg);

  std_msgs::Float32 diff_angle_deg_msg;
  diff_angle_deg_msg.data = amathutils::rad2deg(check_values.diff_angle);
  diff_angle_deg_pub_.publish(diff_angle_deg_msg);

  std_msgs::Float32 diff_angle_deg_median_msg;
  diff_angle_deg_median_msg.data = amathutils::rad2deg(check_values.diff_angle_median);
  diff_angle_deg_median_pub_.publish(diff_angle_deg_median_msg);
}
