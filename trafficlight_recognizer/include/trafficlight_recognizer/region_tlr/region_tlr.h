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

#ifndef TRAFFICLIGHT_RECOGNIZER_REGION_TLR_REGION_TLR_H
#define TRAFFICLIGHT_RECOGNIZER_REGION_TLR_REGION_TLR_H

/* External includes */
#include <cmath>

/* Internal includes */
#include "trafficlight_recognizer/region_tlr/traffic_light_detector.h"

#define MINIMAM_RADIUS 3
#define ROI_MARGINE 25

static inline bool IsNearlyZero(double x)
{
  double abs_x = fabs(x);
  int scale = 100;
  return (abs_x < DBL_MIN * scale);
}

struct valueSet
{
  double upper;
  double lower;
};

struct hsvSet
{
  valueSet Hue;
  valueSet Sat;
  valueSet Val;
};

struct thresholdSet
{
  hsvSet Red;
  hsvSet Yellow;
  hsvSet Green;
};

// #define SHOW_DEBUG_INFO
#endif  // TRAFFICLIGHT_RECOGNIZER_REGION_TLR_REGION_TLR_H
