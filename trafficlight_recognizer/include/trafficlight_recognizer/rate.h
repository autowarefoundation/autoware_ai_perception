/*
 * Copyright 2015 sujiwo
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *    http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef TRAFFICLIGHT_RECOGNIZER_RATE_H
#define TRAFFICLIGHT_RECOGNIZER_RATE_H

#include <unistd.h>
#include <string>
#include <iostream>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace pt = boost::posix_time;

class Rate
{
public:
  inline explicit Rate(int hz)
  {
    assert(hz >= 1);
    float microsec = 1e6 / static_cast<float>(hz);
    sleeptime = pt::microseconds(microsec);
    lastUpdate = pt::microsec_clock::local_time();
  }

  inline void sleep()
  {
    pt::ptime curtime = pt::microsec_clock::local_time();
    pt::time_duration delay = curtime - lastUpdate;
    if (delay < sleeptime)
    {
      pt::time_duration realSleepTime = sleeptime - delay;
      usleep(realSleepTime.total_microseconds());
    }
    lastUpdate = pt::microsec_clock::local_time();
  }

private:
  pt::ptime lastUpdate;
  pt::time_duration sleeptime;
};

#endif  // TRAFFICLIGHT_RECOGNIZER_RATE_H
