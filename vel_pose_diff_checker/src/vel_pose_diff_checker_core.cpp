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

#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <autoware_msgs/RemoteCmd.h>

#include <amathutils_lib/amathutils.hpp>

// referenced by lane_select
tf::Point convertPointIntoRelativeCoordinate(const geometry_msgs::Point &input_point,
                                                        const geometry_msgs::Pose &pose)
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
  , loop_rate_hz_(10.0)
  , comparison_window_size_sec_(1.0)
  , topic_timeout_sec_(0.3)
  , moving_median_window_size_sec_(2.0)
  , diff_position_threshold_meter_(1.0)
  , diff_position_median_threshold_meter_(0.5)
  , diff_angle_threshold_rad_(0.1)
  , diff_angle_median_threshold_rad_(0.05)
  , enable_emergency_to_twist_gate_(true)
{
  private_nh_.getParam("loop_rate_hz", loop_rate_hz_);
  private_nh_.getParam("comparison_window_size_sec", comparison_window_size_sec_);
  private_nh_.getParam("topic_timeout_sec", topic_timeout_sec_);

  private_nh_.getParam("moving_median_window_size_sec", moving_median_window_size_sec_);
  dist_time_queue_.setWindowSizeSec(moving_median_window_size_sec_);
  angle_time_queue_.setWindowSizeSec(moving_median_window_size_sec_);

  private_nh_.getParam("diff_position_threshold_meter", diff_position_threshold_meter_);
  private_nh_.getParam("diff_position_median_threshold_meter", diff_position_median_threshold_meter_);
  private_nh_.getParam("diff_angle_threshold_rad", diff_angle_threshold_rad_);
  private_nh_.getParam("diff_angle_median_threshold_rad", diff_angle_median_threshold_rad_);

  private_nh_.getParam("enable_emergency_to_twist_gate", enable_emergency_to_twist_gate_);

  pose_sub_ = nh_.subscribe("current_pose", 10, &VelPoseDiffChecker::callbackPose, this,
                            ros::TransportHints().tcpNoDelay(true));
  twist_sub_ = nh_.subscribe("current_velocity", 10, &VelPoseDiffChecker::callbackTwist, this,
                            ros::TransportHints().tcpNoDelay(true));

  // for visualize
  diff_position_pub_ = private_nh_.advertise<std_msgs::Float32>("diff_position", 10);
  diff_position_median_pub_ = private_nh_.advertise<std_msgs::Float32>("diff_position_median", 10);
  diff_angle_rad_pub_ = private_nh_.advertise<std_msgs::Float32>("diff_angle_rad", 10);
  diff_angle_rad_median_pub_ = private_nh_.advertise<std_msgs::Float32>("diff_angle_rad_median", 10);
  diff_angle_deg_pub_ = private_nh_.advertise<std_msgs::Float32>("diff_angle_deg", 10);
  diff_angle_deg_median_pub_ = private_nh_.advertise<std_msgs::Float32>("diff_angle_deg_median", 10);

  // to twist_gate
  remote_cmd_pub_ = nh_.advertise<autoware_msgs::RemoteCmd>("remote_cmd", 10);
}

void VelPoseDiffChecker::run()
{
  ros::Rate rate(loop_rate_hz_);

  while (ros::ok())
  {
    ros::spinOnce();

    VelPoseDiffChecker::Status status = checkDiff();

    if (enable_emergency_to_twist_gate_ && status == VelPoseDiffChecker::Status::ERROR)
    {
      ROS_WARN("Publish Emergency Flag!!!");
      autoware_msgs::RemoteCmd remote_cmd;
      remote_cmd.vehicle_cmd.emergency = true;
      remote_cmd_pub_.publish(remote_cmd);
    }

    rate.sleep();
  }
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
    else if (pose_ptr_queue_.front()->header.stamp < msg->header.stamp-ros::Duration(comparison_window_size_sec_*2.0))
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
    else if (twist_ptr_queue_.front()->header.stamp < msg->header.stamp-ros::Duration(comparison_window_size_sec_*2.0))
    {
      twist_ptr_queue_.pop_front();
    }
    else
    {
      break;
    }
  }
}

VelPoseDiffChecker::Status VelPoseDiffChecker::checkDiff()
{
  const auto ros_time_now = ros::Time::now();
  if (ros_time_now.toSec() == 0.0)
  {
    ROS_WARN_STREAM_THROTTLE(1, "ros_time_now is not yet valid");
    return VelPoseDiffChecker::Status::INVALID;
  }

  const auto ros_time_past = ros_time_now-ros::Duration(comparison_window_size_sec_);

  auto pose_time_current = ros_time_now;
  auto pose_time_past = ros_time_past;

  bool is_topic_timeout = false;

  tf::Transform delta_tf_pose(tf::Quaternion(0.0, 0.0, 0.0, 1.0), tf::Vector3(0.0, 0.0, 0.0));

  if (pose_ptr_queue_.empty())
  {
    ROS_WARN_STREAM_THROTTLE(1, "Pose is not subscribed yet");
  }
  else
  {
    const auto current_pose_ptr = pose_ptr_queue_.back();

    // NOTE: ros_time_past-ros::Duration(0.05) -> To stabilize the acquisition time
    const auto past_pose_ptr_it =
      std::lower_bound(std::begin(pose_ptr_queue_), std::end(pose_ptr_queue_), ros_time_past-ros::Duration(0.05),
      [](const boost::shared_ptr<const geometry_msgs::PoseStamped> &x_ptr, ros::Time t)
      {
        return x_ptr->header.stamp < t;
      });  // NOLINT
    const auto past_pose_ptr = past_pose_ptr_it == pose_ptr_queue_.end() ? *(pose_ptr_queue_.end()-1)
                                                                         : *past_pose_ptr_it;

    delta_tf_pose.setOrigin(convertPointIntoRelativeCoordinate(current_pose_ptr->pose.position, past_pose_ptr->pose));
    tf::Quaternion current_pose_orientation, past_pose_orientation;
    tf::quaternionMsgToTF(current_pose_ptr->pose.orientation, current_pose_orientation);
    tf::quaternionMsgToTF(past_pose_ptr->pose.orientation, past_pose_orientation);
    delta_tf_pose.setRotation(past_pose_orientation.inverse() * current_pose_orientation);

    if (std::fabs((ros_time_now-current_pose_ptr->header.stamp).toSec()) > topic_timeout_sec_)
    {
      ROS_WARN("current_pose.header.stamp is late. ros_time_now is %lf. current_pose.header.stamp is %lf",
                ros_time_now.toSec(), current_pose_ptr->header.stamp.toSec());
      is_topic_timeout = true;
    }
    else
    {
      pose_time_current = current_pose_ptr->header.stamp;
      pose_time_past = past_pose_ptr->header.stamp;
    }
  }

  tf::Transform delta_tf_odom(tf::Quaternion(0.0, 0.0, 0.0, 1.0), tf::Vector3(0.0, 0.0, 0.0));

  if (twist_ptr_queue_.empty())
  {
    ROS_WARN_STREAM_THROTTLE(1, "Twist is not subscribed yet");
  }
  else
  {
    auto current_twist_ptr_it =
      std::lower_bound(std::begin(twist_ptr_queue_), std::end(twist_ptr_queue_), pose_time_current,
      [](const boost::shared_ptr<const geometry_msgs::TwistStamped> &x_ptr, ros::Time t)
      {
        return x_ptr->header.stamp < t;
      });  //NOLINT
    current_twist_ptr_it = current_twist_ptr_it == twist_ptr_queue_.end() ? (twist_ptr_queue_.end()-1)
                                                                          : current_twist_ptr_it;

    auto past_twist_ptr_it =
      std::lower_bound(std::begin(twist_ptr_queue_), std::end(twist_ptr_queue_), pose_time_past,
      [](const boost::shared_ptr<const geometry_msgs::TwistStamped> &x_ptr, ros::Time t)
      {
        return x_ptr->header.stamp < t;
      });  //NOLINT
    past_twist_ptr_it = past_twist_ptr_it == twist_ptr_queue_.end() ? twist_ptr_queue_.end()-1
                                                                    : past_twist_ptr_it;

    if (std::fabs((ros_time_now-(*current_twist_ptr_it)->header.stamp).toSec()) > topic_timeout_sec_)
    {
      ROS_WARN("current_twist.header.stamp is late. ros_time_now is %lf. current_twist.header.stamp is %lf",
                ros_time_now.toSec(), (*current_twist_ptr_it)->header.stamp.toSec());
      is_topic_timeout = true;
    }

    ros::Time prev_time = (*past_twist_ptr_it)->header.stamp;
    double x = 0.0, y = 0.0, yaw = 0.0;
    for (auto twist_ptr_it = past_twist_ptr_it+1; twist_ptr_it != current_twist_ptr_it+1; ++twist_ptr_it)
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
  }

  const double diff_position = (delta_tf_pose.getOrigin()-delta_tf_odom.getOrigin()).length();
  const double diff_angle = (delta_tf_odom.getRotation().inverse() * delta_tf_pose.getRotation()).getAngle();

  dist_time_queue_.addValueTime(diff_position, ros_time_now);
  const double diff_position_median = dist_time_queue_.getMedianValue();

  angle_time_queue_.addValueTime(diff_angle, ros_time_now);
  const double diff_angle_median = angle_time_queue_.getMedianValue();

  std_msgs::Float32 diff_position_msg;
  diff_position_msg.data = diff_position;
  diff_position_pub_.publish(diff_position_msg);

  std_msgs::Float32 diff_position_median_msg;
  diff_position_median_msg.data = diff_position_median;
  diff_position_median_pub_.publish(diff_position_median_msg);

  std_msgs::Float32 diff_angle_rad_msg;
  diff_angle_rad_msg.data = diff_angle;
  diff_angle_rad_pub_.publish(diff_angle_rad_msg);

  std_msgs::Float32 diff_angle_rad_median_msg;
  diff_angle_rad_median_msg.data = diff_angle_median;
  diff_angle_rad_median_pub_.publish(diff_angle_rad_median_msg);

  std_msgs::Float32 diff_angle_deg_msg;
  diff_angle_deg_msg.data = amathutils::rad2deg(diff_angle);
  diff_angle_deg_pub_.publish(diff_angle_deg_msg);

  std_msgs::Float32 diff_angle_deg_median_msg;
  diff_angle_deg_median_msg.data = amathutils::rad2deg(diff_angle_median);
  diff_angle_deg_median_pub_.publish(diff_angle_deg_median_msg);

  if ( is_topic_timeout
    || diff_position > diff_position_threshold_meter_
    || diff_position_median > diff_position_median_threshold_meter_
    || diff_angle > diff_angle_threshold_rad_
    || diff_angle_median > diff_angle_median_threshold_rad_ )
  {
    ROS_WARN("Difference Detection!!!!!");
    ROS_WARN("is_topic_timeout %d, diff_pos %lf[m], diff_pos_median %lf[m], diff_ang %lf[rad], diff_ang_median %lf[rad]"
            , is_topic_timeout, diff_position, diff_position_median, diff_angle, diff_angle_median);
    return VelPoseDiffChecker::Status::ERROR;
  }

  ROS_DEBUG("is_topic_timeout %d, diff_pos %lf[m], diff_pos_median %lf[m], diff_ang %lf[rad], diff_ang_median %lf[rad]"
           , is_topic_timeout, diff_position, diff_position_median, diff_angle, diff_angle_median);
  return VelPoseDiffChecker::Status::OK;
}
