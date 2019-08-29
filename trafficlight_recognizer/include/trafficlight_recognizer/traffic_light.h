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

#ifndef TRAFFICLIGHT_RECOGNIZER_TRAFFIC_LIGHT_H
#define TRAFFICLIGHT_RECOGNIZER_TRAFFIC_LIGHT_H

/* Extra includes */
#include "autoware_msgs/Signals.h"

#define MAIN_WINDOW_NAME "Main"
#define SETTINGS_WINDOW_NAME "Settings"

#define TLR_GREEN_SIGNAL_STR "green signal"
#define TLR_RED_SIGNAL_STR "red signal"
#define TLR_UNKNOWN_SIGNAL_STR ""
#define TRAFFIC_LIGHT_RED 0
#define TRAFFIC_LIGHT_GREEN 1
#define TRAFFIC_LIGHT_UNKNOWN 2

#endif  // TRAFFICLIGHT_RECOGNIZER_TRAFFIC_LIGHT_H
