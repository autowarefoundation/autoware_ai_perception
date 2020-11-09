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

#ifndef TRAFFICLIGHT_RECOGNIZER_FEAT_PROJ_LANELET2_FEAT_PROJ_LANELET2_CORE_H
#define TRAFFICLIGHT_RECOGNIZER_FEAT_PROJ_LANELET2_FEAT_PROJ_LANELET2_CORE_H

#include <ros/ros.h>
#include <tf/tf.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraphContainer.h>

#include <lanelet2_extension/regulatory_elements/autoware_traffic_light.h>
#include <lanelet2_extension/utility/query.h>

#include <autoware_lanelet2_msgs/MapBin.h>
#include <autoware_msgs/AdjustXY.h>
#include <autoware_msgs/LaneArray.h>
#include <autoware_msgs/Signals.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>
#include <vector>

static constexpr double DEFAULT_SIGNAL_LAMP_RADIUS = 0.3;

namespace trafficlight_recognizer
{
class FeatProjLanelet2
{
  friend class FeatProjLanelet2TestClass;

public:
  FeatProjLanelet2();
  void init();
  void run();

private:
  ros::NodeHandle rosnode_;
  ros::NodeHandle private_nh_;

  ros::Subscriber bin_map_sub_;
  ros::Subscriber camera_info_subscriber_;
  ros::Subscriber adjustXY_subscriber_;
  ros::Subscriber waypoint_subscriber_;

  ros::Publisher roi_sign_pub_;
  ros::Publisher visible_traffic_light_triangle_pub_;

  std::string camera_frame_;

  bool viz_tl_signs_ = true;

  bool use_path_info_ = false;

  bool loaded_lanelet_map_ = false;
  lanelet::LaneletMapPtr lanelet_map_;
  lanelet::routing::RoutingGraphPtr routing_graph_ptr_;

  int adjust_proj_x_ = 0;
  int adjust_proj_y_ = 0;
  float near_plane_ = 1.0;
  float far_plane_ = 200.0;

  autoware_msgs::LaneArray waypoints_;

  Eigen::Vector3f position_;
  Eigen::Quaternionf orientation_;
  float fx_;
  float fy_;
  float image_width_;
  float image_height_;
  float cx_;
  float cy_;

  tf::StampedTransform camera_to_map_tf_;
  tf::StampedTransform map_to_camera_tf_;

  void adjustXYCallback(const autoware_msgs::AdjustXY& config_msg);
  void cameraInfoCallback(const sensor_msgs::CameraInfo& camInfoMsg);
  void binMapCallback(const autoware_lanelet2_msgs::MapBin& msg);
  void waypointsCallback(const autoware_msgs::Lane::ConstPtr& waypoints);

  void getTransform(const std::string from_frame, const std::string to_frame, Eigen::Quaternionf* ori,
                    Eigen::Vector3f* pos, tf::StampedTransform* tf);
  Eigen::Vector3f transform(const Eigen::Vector3f& psrc, const tf::StampedTransform& tfsource);  // NOLINT
  bool project2(const Eigen::Vector3f& pt, int* u, int* v, const bool useOpenGLCoord = false);
  double getAbsoluteDiff2Angles(const double x, const double y, const double c);
  double normalise(const double value, const double start, const double end);
  bool inRange(const lanelet::BasicPoint2d& p, const lanelet::BasicPoint2d& cam, const double max_r);
  bool inView(const lanelet::BasicPoint2d& p, const lanelet::BasicPoint2d& cam, double heading, const double max_a,
              const double max_r);
  bool isAttributeValue(const lanelet::ConstPoint3d& p, const std::string& attr_str, const std::string& value_str);
  void trafficLightVisibilityCheck(const std::vector<lanelet::AutowareTrafficLightConstPtr>& aw_tl_reg_elems,
                                   std::vector<lanelet::AutowareTrafficLightConstPtr>* visible_aw_tl);
  void findVisibleTrafficLightFromLanes(std::vector<lanelet::AutowareTrafficLightConstPtr>* visible_aw_tl);
  void findSignalsInCameraFrame(const std::vector<lanelet::AutowareTrafficLightConstPtr>& visible_aw_tl,
                                autoware_msgs::Signals* signalsInFrame);
  lanelet::ConstLineString3d createDummyLightBulbString(const lanelet::ConstLineString3d& base_string);
};
}  // namespace trafficlight_recognizer

#endif  // TRAFFICLIGHT_RECOGNIZER_FEAT_PROJ_LANELET2_FEAT_PROJ_LANELET2_CORE_H
