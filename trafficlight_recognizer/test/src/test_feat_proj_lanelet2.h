/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
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
 *
 * Authors: Simon Thompson
 * 
 */
#ifndef TEST_SRC_TEST_FEAT_PROJ_LANELET2_H  // NOLINT
#define TEST_SRC_TEST_FEAT_PROJ_LANELET2_H

#include <gtest/gtest.h>
#include <ros/connection_manager.h>
#include <ros/ros.h>

#include <algorithm>

#include <trafficlight_recognizer/feat_proj_lanelet2/feat_proj_lanelet2_core.h>

namespace trafficlight_recognizer
{
class FeatProjLanelet2TestClass
{
public:
  FeatProjLanelet2TestClass()
  {
  }

  FeatProjLanelet2* fpll2;

  ros::NodeHandle rosnode;

  bool inRange(lanelet::BasicPoint2d p, lanelet::BasicPoint2d cam, double max_r)
  {
    return fpll2->inRange(p, cam, max_r);
  }

  Eigen::Vector3f transform(Eigen::Vector3f p, tf::StampedTransform tf)
  {
    return (fpll2->transform(p, tf));
  }

  void setCameraInfo(double cx, double cy, double fx, double fy, int iw, int ih)
  {
    fpll2->cx_ = cx;
    fpll2->cy_ = cy;
    fpll2->image_width_ = iw;
    fpll2->image_height_ = ih;
    fpll2->fx_ = fx;
    fpll2->fy_ = fy;
  }

  void setCameraToMapTransform(tf::StampedTransform tf)
  {
    fpll2->camera_to_map_tf_ = tf;
  }

  bool project2(const Eigen::Vector3f& pt, int* u, int* v)
  {
    return (fpll2->project2(pt, u, v, false));
  }

  bool inView(lanelet::BasicPoint2d p, lanelet::BasicPoint2d cam, double heading, double max_a, double max_r)
  {
    return fpll2->inView(p, cam, heading, max_a, max_r);
  }
};

}  // namespace trafficlight_recognizer

#endif  // TEST_SRC_TEST_FEAT_PROJ_LANELET2_H NOLINT
