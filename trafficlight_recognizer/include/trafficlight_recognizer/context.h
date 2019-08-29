/*
 * Copyright 2019 Autoware Foundation
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

#ifndef TRAFFICLIGHT_RECOGNIZER_CONTEXT_H
#define TRAFFICLIGHT_RECOGNIZER_CONTEXT_H

#include <vector>
#include <opencv2/core/core.hpp>
#include <autoware_msgs/Signals.h>

enum LightState
{
  GREEN,
  YELLOW,
  RED,
  UNDEFINED
};

class Context
{
public:
  Context() {}

  Context(cv::Point aRedCenter, cv::Point aYellowCenter, cv::Point aGreenCenter, int aLampRadius, cv::Point aTopLeft,
          cv::Point aBotRight);

  static void SetContexts(std::vector<Context>* out_contexts, const autoware_msgs::Signals::ConstPtr& in_extracted_pos,
                          const int in_image_height, const int in_image_width);

  cv::Point redCenter;
  cv::Point yellowCenter;
  cv::Point greenCenter;
  cv::Point3d redCenter3d;
  cv::Point3d yellowCenter3d;
  cv::Point3d greenCenter3d;
  int lampRadius;
  cv::Point topLeft;
  cv::Point botRight;
  LightState lightState = UNDEFINED;
  LightState newCandidateLightState = UNDEFINED;
  int signalID;
  int stateJudgeCount;
  bool leftTurnSignal;
  bool rightTurnSignal;
  int closestLaneId;

private:
  static bool CompareContext(const Context in_context_a, const Context in_context_b);
};

#endif  // TRAFFICLIGHT_RECOGNIZER_CONTEXT_H
