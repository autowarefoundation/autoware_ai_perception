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

#ifndef TRAFFICLIGHT_RECOGNIZER_REGION_TLR_MXNET_MXNET_TRAFFIC_LIGHT_RECOGNIZER_H
#define TRAFFICLIGHT_RECOGNIZER_REGION_TLR_MXNET_MXNET_TRAFFIC_LIGHT_RECOGNIZER_H

#include <vector>
#include <string>

#include "mxnet-cpp/MxNetCpp.h"
#include <mxnet/c_predict_api.h>
#include <opencv2/opencv.hpp>

#include "trafficlight_recognizer/context.h"

class MxNetTrafficLightRecognizer
{
  enum NetworkResults
  {
    Green,
    Yellow,
    Red,
    None
  };
  int width_;
  int height_;
  int num_channels_;
  PredictorHandle prediction_handle_;

  void PreProcessImage(const cv::Mat& in_image, mx_float* out_image_data, const int in_channels,
                       const cv::Size in_resize_size);

public:
  MxNetTrafficLightRecognizer();
  ~MxNetTrafficLightRecognizer();
  void Init(const char* in_network_definition_buffer, const char* in_pretrained_model_buffer,
            int in_pretrained_model_length, const bool in_use_gpu, const unsigned int in_gpu_id);

  LightState RecognizeLightState(const cv::Mat& in_image, double in_score_threshold);
};

#endif  // TRAFFICLIGHT_RECOGNIZER_REGION_TLR_MXNET_MXNET_TRAFFIC_LIGHT_RECOGNIZER_H
