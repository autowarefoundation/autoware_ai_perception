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

#ifndef TRAFFICLIGHT_RECOGNIZER_ROI_EXTRACTOR_ROI_EXTRACTOR_H
#define TRAFFICLIGHT_RECOGNIZER_ROI_EXTRACTOR_ROI_EXTRACTOR_H

#include <string>

#include <ros/ros.h>
#include <autoware_msgs/Signals.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>

class ROIExtractor
{
public:
  explicit ROIExtractor(int minimum_height, double similarity_threshold)
    : k_minimum_height_(minimum_height)
    , k_similarity_threshold_(similarity_threshold)
    , previous_saved_frame_(cv::Mat()) {}

  ~ROIExtractor() {}

  // Callback functions to obtain images and signal position
  void ImageRawCallback(const sensor_msgs::Image& image);
  void ROISignalCallback(const autoware_msgs::Signals::ConstPtr& extracted_pos);

  // Utility function to create directory which roi images will be saved
  void CreateTargetDirectory(std::string base_name);

private:
  // Utility function to count the number of files contained in the specified directory
  int CountFileNum(std::string directory_name);

  // Utility function to create directory tree
  void MakeDirectoryTree(const std::string& target, const std::string& base, const mode_t& mode);

  // The function to calculate similarity of two image
  double CalculateSimilarity(const cv::Mat& image1, const cv::Mat& image2);

  // Directory path that extracted ROI images will be saved
  std::string target_directory_;

  // One subscribed frame image
  cv::Mat frame_;

  // Time stamp value of subscribed value
  ros::Time frame_timestamp_;
  ros::Time previous_timestamp_;

  // The number of files contained in the target directory
  int file_count_;

  // The minimum height threshold of ROI image that will be saved
  const int k_minimum_height_;

  // The threshold of the level of similarity
  const double k_similarity_threshold_;

  // The image saved last time
  cv::Mat previous_saved_frame_;
};

#endif  // TRAFFICLIGHT_RECOGNIZER_ROI_EXTRACTOR_ROI_EXTRACTOR_H
