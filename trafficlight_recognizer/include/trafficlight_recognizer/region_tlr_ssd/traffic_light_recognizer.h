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

#ifndef TRAFFICLIGHT_RECOGNIZER_REGION_TLR_SSD_TRAFFIC_LIGHT_RECOGNIZER_H
#define TRAFFICLIGHT_RECOGNIZER_REGION_TLR_SSD_TRAFFIC_LIGHT_RECOGNIZER_H

#include <vector>
#include <string>

#include <caffe/caffe.hpp>
#include <opencv2/opencv.hpp>

#include "trafficlight_recognizer/context.h"

class TrafficLightRecognizer
{
public:
  TrafficLightRecognizer();
  ~TrafficLightRecognizer();
  void Init(const std::string& network_definition_file_name, const std::string& pretrained_model_file_name,
            const bool use_gpu, const unsigned int gpu_id);

  LightState RecognizeLightState(const cv::Mat& image);

private:
  void WrapInputLayer(std::vector<cv::Mat>* input_channels);
  void Preprocess(const cv::Mat& image, std::vector<cv::Mat>* input_channels);

  boost::shared_ptr<caffe::Net<float> > network_;
  int num_channels_;
  cv::Size input_geometry_;
  cv::Scalar kPixelMean_;
};

#endif  // TRAFFICLIGHT_RECOGNIZER_REGION_TLR_SSD_TRAFFIC_LIGHT_RECOGNIZER_H
