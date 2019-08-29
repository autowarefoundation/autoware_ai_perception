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

#ifndef TRAFFICLIGHT_RECOGNIZER_TLR_TUNER_TUNER_BODY_H
#define TRAFFICLIGHT_RECOGNIZER_TLR_TUNER_TUNER_BODY_H

#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#define DEFAULT_SAT_LOWER ((double)0.37 * 255)
#define DEFAULT_SAT_UPPER (255)
#define DEFAULT_VAL_LOWER (90)
#define DEFAULT_VAL_UPPER (255)

typedef struct
{
  int center;
  int range;
}
value_set;

typedef struct
{
  value_set hue;
  value_set sat;
  value_set val;
  bool isUpdated;
}
thresholds_set;

class TunerBody
{
private:
  static cv::Point Clicked_point;
  static int Signal_color;
  int H_slider_val;
  int S_slider_val;
  int V_slider_val;
  static cv::Mat src_img;
  cv::Mat base;
  static cv::Mat mask;
  static std::string windowName;
  static thresholds_set Red_set;
  static thresholds_set Yellow_set;
  static thresholds_set Green_set;
  static thresholds_set* Selected_set;
  static bool updateImage;

public:
  enum signal_state
  {
    GREEN = 0,
    YELLOW = 1,
    RED = 2,
  };

  TunerBody();
  ~TunerBody();
  void launch(void);
  static void setColor(signal_state state);
  static void setClickedPoint(cv::Point pt);
  static void saveResult(std::string fileName);
  static void openSetting(std::string fileName);
  static void setUpdateImage(void);
  static void image_raw_callBack(const sensor_msgs::Image& image_msg);
};

#endif  // TRAFFICLIGHT_RECOGNIZER_TLR_TUNER_TUNER_BODY_H
