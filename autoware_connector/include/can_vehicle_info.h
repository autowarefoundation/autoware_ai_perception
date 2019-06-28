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

#ifndef CAN_VEHICLE_INFO_H
#define CAN_VEHICLE_INFO_H

namespace autoware_connector
{
inline double kmph2mps(double velocity_kmph)
{
  return (velocity_kmph * 1000) / (60 * 60);
}

inline double mps2kmph(double velocity_mps)
{
  return (velocity_mps * 60 * 60) / 1000;
}

// convert degree to radian
inline double deg2rad(double deg)
{
  return deg * M_PI / 180;
}

// convert degree to radian
inline double rad2deg(double rad)
{
  return rad * 180 / M_PI;
}
struct VehicleInfo
{
  bool is_stored;
  double wheel_base;
  double minimum_turning_radius;
  double maximum_steering_wheel_angle_deg;

  VehicleInfo()
  {
    is_stored = false;
    wheel_base = 0.0;
    minimum_turning_radius = 0.0;
    maximum_steering_wheel_angle_deg = 0.0;
  }
  double convertSteeringAngleToAngularVelocity(const double cur_vel_mps, const double cur_angle_rad)  // rad/s
  {
    return is_stored ? tan(cur_angle_rad) * cur_vel_mps / wheel_base : 0;
  }
  double getCurrentSteeringAngle(const double steering_wheel_angle_rad)  // steering wheel [rad] -> steering [rad]
  {
    return is_stored ?
               steering_wheel_angle_rad * getMaximumSteeringWheelAngle() / deg2rad(maximum_steering_wheel_angle_deg) :
               0;
  }
  double getMaximumSteeringWheelAngle()  // radian
  {
    return is_stored ? asin(wheel_base / minimum_turning_radius) : 0;
  }
};
}
#endif  // CAN_VEHICLE_INFO_H
