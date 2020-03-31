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

#include "vel_pose_diff_checker/value_time_queue.h"

#include <iostream>
#include <deque>
#include <algorithm>

ValueTimeQueue::ValueTimeQueue() : window_size_sec_(1.0)
{
}

ValueTimeQueue::ValueTimeQueue(const double window_size_sec) : window_size_sec_(window_size_sec)
{
}

void ValueTimeQueue::addValueTime(const double value, const ros::Time& a_stamp)
{
  const auto ros_time_now = ros::Time::now();
  queue_.push_back(ValueTime(value, a_stamp));
  while (!queue_.empty())
  {
    // for replay rosbag
    if (queue_.front().stamp > ros_time_now)
    {
      queue_.pop_front();
    }
    else if (queue_.front().stamp < ros_time_now - ros::Duration(window_size_sec_))
    {
      queue_.pop_front();
    }
    else
    {
      break;
    }
  }
}

double ValueTimeQueue::getMedianValue() const
{
  std::deque<ValueTime> queue_tmp = queue_;
  const size_t median_index = queue_tmp.size() / 2;
  std::nth_element(std::begin(queue_tmp), std::begin(queue_tmp) + median_index, std::end(queue_tmp),
                   [](const ValueTime& lhs, const ValueTime& rhs) { return lhs.value < rhs.value; });  // NOLINT

  return queue_tmp.at(median_index).value;
}

void ValueTimeQueue::setWindowSizeSec(const double time_window_size_sec)
{
  window_size_sec_ = time_window_size_sec;
}
