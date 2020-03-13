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

#ifndef CAN_STATUS_TRANSLATOR_CORE_H
#define CAN_STATUS_TRANSLATOR_CORE_H

// ROS includes
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>

// User Defined Includes
#include "autoware_can_msgs/CANInfo.h"
#include "autoware_msgs/VehicleStatus.h"
#include "autoware_msgs/Gear.h"
#include "can_vehicle_info.h"

namespace autoware_connector
{
class CanStatusTranslatorNode
{
  enum class GearShift
  {
    Drive = 16,
    Neutral = 32,
    Reverse = 64,
    Parking = 128,
  };

public:
  CanStatusTranslatorNode();
  ~CanStatusTranslatorNode();

  void run();

private:
  // handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // publisher
  ros::Publisher pub1_, pub2_, pub3_;

  // subscriber
  ros::Subscriber sub1_, sub2_;

  // variables
  VehicleInfo v_info_;

  // callbacks
  void callbackFromCANInfo(const autoware_can_msgs::CANInfoConstPtr& msg);
  void callbackFromVehicleStatus(const autoware_msgs::VehicleStatusConstPtr& msg);

  // initializer
  void initForROS();

  // functions
  void publishVelocity(const autoware_msgs::VehicleStatusConstPtr& msg);
  void publishVelocityViz(const autoware_msgs::VehicleStatusConstPtr& msg);
  void publishVehicleStatus(const autoware_can_msgs::CANInfoConstPtr& msg);
};
}
#endif  // CAN_STATUS_TRANSLATOR_CORE_H
