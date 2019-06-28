/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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

#ifndef CAN_ODOMETRY_CORE_H
#define CAN_ODOMETRY_CORE_H

// ROS includes
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>

// User Defined Includes
#include "autoware_can_msgs/CANInfo.h"
#include "autoware_msgs/VehicleStatus.h"
#include "can_vehicle_info.h"

namespace autoware_connector
{
struct Odometry
{
  double x;
  double y;
  double th;
  ros::Time stamp;

  Odometry(const ros::Time& time)
  {
    x = 0.0;
    y = 0.0;
    th = 0.0;
    stamp = time;
  }

  void updateOdometry(const double vx, const double vth, const ros::Time& cur_time)
  {
    if (stamp.sec == 0 && stamp.nsec == 0)
    {
      stamp = cur_time;
    }
    double dt = (cur_time - stamp).toSec();
    double delta_x = (vx * cos(th)) * dt;
    double delta_y = (vx * sin(th)) * dt;
    double delta_th = vth * dt;

    ROS_INFO("dt : %f delta (x y th) : (%f %f %f %f)", dt, delta_x, delta_y, delta_th);

    x += delta_x;
    y += delta_y;
    th += delta_th;
    stamp = cur_time;
  }
};

class CanOdometryNode
{
public:
  CanOdometryNode();
  ~CanOdometryNode();

  void run();

private:
  // handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // publisher
  ros::Publisher pub1_;

  // subscriber
  ros::Subscriber sub1_;

  // variables
  VehicleInfo v_info_;
  Odometry odom_;

  // callbacks
  void callbackFromVehicleStatus(const autoware_msgs::VehicleStatusConstPtr& msg);

  // initializer
  void initForROS();

  // functions
  void publishOdometry(const autoware_msgs::VehicleStatusConstPtr& msg);
};
}
#endif  // CAN_ODOMETRY_CORE_H
