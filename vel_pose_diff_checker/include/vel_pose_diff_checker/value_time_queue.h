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

#ifndef VEL_POSE_DIFF_CHECKER_VALUE_TIME_QUEUE_H
#define VEL_POSE_DIFF_CHECKER_VALUE_TIME_QUEUE_H

#include <deque>

#include <ros/ros.h>

class ValueTimeQueue
{
  struct ValueTime
  {
    ValueTime(const double a_value, const ros::Time& a_stamp) : value(a_value), stamp(a_stamp)
    {
    }

    double value;
    ros::Time stamp;
  };

public:
  ValueTimeQueue();
  explicit ValueTimeQueue(const double window_size_sec);
  void addValueTime(const double value, const ros::Time& a_stamp);
  double getMedianValue() const;
  void setWindowSizeSec(const double time_window_size_sec);

private:
  double window_size_sec_;
  std::deque<ValueTime> queue_;
};

#endif  // VEL_POSE_DIFF_CHECKER_VALUE_TIME_QUEUE_H
