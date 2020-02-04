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

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/utility/utilities.h>
#include <lanelet2_extension/visualization/visualization.h>

#include <autoware_lanelet2_msgs/MapBin.h>
#include <autoware_msgs/AdjustXY.h>
#include <autoware_msgs/Signals.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <string>
#include <vector>

#include <trafficlight_recognizer/feat_proj_lanelet2/feat_proj_lanelet2_core.h>

using lanelet::utils::getId;

namespace trafficlight_recognizer
{
/* Callback function to shift projection result */
void FeatProjLanelet2::adjustXYCallback(const autoware_msgs::AdjustXY& config_msg)
{
  adjust_proj_x_ = config_msg.x;
  adjust_proj_y_ = config_msg.y;
}

/**
 * [FeatProjLanelet2::cameraInfoCallback callback function for camera info]
 */
void FeatProjLanelet2::cameraInfoCallback(const sensor_msgs::CameraInfo& camInfoMsg)
{
  fx_ = static_cast<float>(camInfoMsg.P[0]);
  fy_ = static_cast<float>(camInfoMsg.P[5]);
  image_width_ = camInfoMsg.width;
  image_height_ = camInfoMsg.height;
  cx_ = static_cast<float>(camInfoMsg.P[2]);
  cy_ = static_cast<float>(camInfoMsg.P[6]);
}

// @brief Callback function for lanelet map
void FeatProjLanelet2::binMapCallback(const autoware_lanelet2_msgs::MapBin& msg)
{
  lanelet_map_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(msg, lanelet_map_);
  loaded_lanelet_map_ = true;
}

void FeatProjLanelet2::waypointsCallback(const autoware_msgs::Lane::ConstPtr& waypoints)
{
  if (!use_path_info_)
  {
    return;
  }
  waypoints_.lanes.clear();
  waypoints_.lanes.push_back(std::move(*waypoints));
  int waypoint_id = 0;

  // In case gid(global id) in waypoint is not filled, overwrite it.
  // get max gid in first loop and overwrite invalid gid in second loop
  for (auto& lane : waypoints_.lanes)
  {
    for (auto& wp : lane.waypoints)
    {
      if (wp.gid > waypoint_id)
      {
        waypoint_id = wp.gid;
      }
    }
  }

  for (auto& lane : waypoints_.lanes)
  {
    for (auto& wp : lane.waypoints)
    {
      // overwrite invalid gid
      if (wp.gid == 0)
      {
        wp.gid = waypoint_id++;
      }
    }
  }
}

// @brief get transformation between given frames
void FeatProjLanelet2::getTransform(const std::string from_frame, const std::string to_frame, Eigen::Quaternionf* ori,
                                    Eigen::Vector3f* pos, tf::StampedTransform* tf)
{
  if (ori == nullptr || pos == nullptr || tf == nullptr)
  {
    ROS_ERROR_STREAM(__FUNCTION__ << ": ori, pos, or tf is null pointer!");
    return;
  }

  static tf::TransformListener listener;

  ros::Time now = ros::Time();
  listener.waitForTransform(from_frame, to_frame, now, ros::Duration(10.0));
  listener.lookupTransform(from_frame, to_frame, now, *tf);

  tf::Vector3& p = tf->getOrigin();
  tf::Quaternion o = tf->getRotation();

  pos->x() = p.x();
  pos->y() = p.y();
  pos->z() = p.z();
  ori->w() = o.w();
  ori->x() = o.x();
  ori->y() = o.y();
  ori->z() = o.z();
}

// @brief transform 3d point
Eigen::Vector3f FeatProjLanelet2::transform(const Eigen::Vector3f& psrc, const tf::StampedTransform& tfsource)
{
  tf::Vector3 pt3(psrc.x(), psrc.y(), psrc.z());
  tf::Vector3 pt3s = tfsource * pt3;
  Eigen::Vector3f tf_v(pt3s.x(), pt3s.y(), pt3s.z());
  return tf_v;
}

// @brief project point in map frame into 2D camera image
bool FeatProjLanelet2::project2(const Eigen::Vector3f& pt, int* u, int* v, const bool useOpenGLCoord)
{
  if (u == nullptr || v == nullptr)
  {
    ROS_ERROR_STREAM(__FUNCTION__ << ": u or v is null pointer!");
    return false;
  }
  float nearPlane = 1.0;
  float farPlane = 200.0;

  Eigen::Vector3f transformed_pt = transform(pt, camera_to_map_tf_);

  float _u = transformed_pt.x() * fx_ / transformed_pt.z() + cx_;
  float _v = transformed_pt.y() * fy_ / transformed_pt.z() + cy_;

  *u = static_cast<int>(_u);
  *v = static_cast<int>(_v);
  if (*u < 0 || image_width_ < *u || *v < 0 || image_height_ < *v || transformed_pt.z() < nearPlane ||
      farPlane < transformed_pt.z())
  {
    *u = -1, *v = -1;
    return false;
  }

  if (useOpenGLCoord)
  {
    *v = image_height_ - *v;
  }

  return true;
}

// @brief get absolute difference between two angles.
double FeatProjLanelet2::getAbsoluteDiff2Angles(const double x, const double y, const double c)
{
  // c can be PI or 180;
  return c - std::abs(std::fmod(std::abs(x - y), 2 * c) - c);
}

// @brief normalize angle "value" between "start" and "end"
double FeatProjLanelet2::normalise(const double value, const double start, const double end)
{
  const double width = end - start;
  const double offsetValue = value - start;  // value relative to 0

  return (offsetValue - (std::floor(offsetValue / width) * width)) + start;
  // + start to reset back to start of original range
}

// @brief check if point p is within range
bool FeatProjLanelet2::inRange(const lanelet::BasicPoint2d& p, const lanelet::BasicPoint2d& cam, const double max_r)
{
  double d = lanelet::geometry::distance(p, cam);
  return (d < max_r);
}

// @brief check if point p is within distance max_r and within angle (-max_a, max_a)
bool FeatProjLanelet2::inView(const lanelet::BasicPoint2d& p, const lanelet::BasicPoint2d& cam, const double heading,
                              const double max_a, const double max_r)
{
  double d = lanelet::geometry::distance(p, cam);
  double a = getAbsoluteDiff2Angles(heading, std::atan2(p.y() - cam.y(), p.x() - cam.x()), M_PI);

  return (d < max_r && a < max_a);
}

// @brief return true if point has specified tag
bool FeatProjLanelet2::isAttributeValue(const lanelet::ConstPoint3d& p, const std::string& attr_str,
                                        const std::string& value_str)
{
  if (!p.hasAttribute(attr_str))
    return false;

  lanelet::Attribute attr = p.attribute(attr_str);
  if (attr.value().compare(value_str) == 0)
    return true;

  return false;
}

// @brief find visible traffic lights
void FeatProjLanelet2::trafficLightVisibilityCheck(
    const std::vector<lanelet::AutowareTrafficLightConstPtr>& aw_tl_reg_elems,
    std::vector<lanelet::AutowareTrafficLightConstPtr>* visible_aw_tl)
{
  if (visible_aw_tl == nullptr)
  {
    ROS_ERROR_STREAM(__FUNCTION__ << ": visible_aw_tl is null pointer!");
    return;
  }

  lanelet::BasicPoint2d camera_position_2d(position_.x(), position_.y());
  double cam_yaw = tf::getYaw(map_to_camera_tf_.getRotation()) + M_PI / 2;

  // for each traffic light in map check if in range and in view angle of camera
  for (auto tli = aw_tl_reg_elems.begin(); tli != aw_tl_reg_elems.end(); tli++)
  {
    lanelet::AutowareTrafficLightConstPtr tl = *tli;

    lanelet::LineString3d ls;

    auto lights = tl->trafficLights();
    for (auto lsp : lights)
    {
      if (lsp.isLineString())  // traffic ligths must be linestrings
      {
        lanelet::ConstLineString3d ls = static_cast<lanelet::ConstLineString3d>(lsp);

        lanelet::BasicPoint2d tl_base_0 = lanelet::utils::to2D(ls.front()).basicPoint();
        lanelet::BasicPoint2d tl_base_1 = lanelet::utils::to2D(ls.back()).basicPoint();

        double max_r = 200.0;

        if (inRange(tl_base_0, camera_position_2d, max_r) && inRange(tl_base_1, camera_position_2d, max_r))
        {
          double dx = tl_base_1.x() - tl_base_0.x();
          double dy = tl_base_1.y() - tl_base_0.y();
          double nx = -dy;  // 90 rotation for cos sin = 1's and 0's -> normal is -dy, dx
          double ny = dx;
          double dir = normalise(std::atan2(ny, nx), -M_PI, M_PI);

          double diff = getAbsoluteDiff2Angles(dir, cam_yaw, M_PI);

          // traffic light must be facing to the vehicle
          if (std::abs(diff) < 50.0 / 180.0 * M_PI)
          {
            // testing range twice (above inRange) for each base point at the moment
            if (inView(tl_base_0, camera_position_2d, cam_yaw, M_PI, max_r) &&
                inView(tl_base_1, camera_position_2d, cam_yaw, M_PI, max_r))
            {
              visible_aw_tl->push_back(tl);
            }
          }
        }
      }
    }
  }
}

void FeatProjLanelet2::findVisibleTrafficLightFromLanes(
    std::vector<lanelet::AutowareTrafficLightConstPtr>* visible_aw_tl)
{
  if (visible_aw_tl == nullptr)
  {
    ROS_ERROR_STREAM(__FUNCTION__ << ": visible_aw_tl is null pointer!");
    return;
  }

  std::map<int, lanelet::Id> waypoint2laneletid;
  std::set<lanelet::Id> already_added;
  lanelet::ConstLanelets relevant_lanelets;

  // find lanelets that matches with waypoint
  lanelet::utils::matchWaypointAndLanelet(lanelet_map_, routing_graph_ptr_, waypoints_, &waypoint2laneletid);
  for (const auto& wp2llt : waypoint2laneletid)
  {
    if (already_added.find(wp2llt.second) == already_added.end())
    {
      lanelet::ConstLanelet lanelet = lanelet_map_->laneletLayer.get(wp2llt.second);
      relevant_lanelets.push_back(lanelet);
      already_added.insert(wp2llt.second);
    }
  }

  *visible_aw_tl = lanelet::utils::query::autowareTrafficLights(relevant_lanelets);
}

// @brief create dummy light_bulbs linestring from traffic_light linestring
lanelet::ConstLineString3d FeatProjLanelet2::createDummyLightBulbString(const lanelet::ConstLineString3d& base_string)
{
  lanelet::LineString3d bounding_box(base_string.id());

  if (!base_string.empty())
  {
    double height = base_string.attributeOr("height", 0.0);

    lanelet::Point3d p0(getId(), base_string.front().basicPoint());
    p0.attributes()["radius"] = 0.1;
    p0.attributes()["color"] = "red";
    bounding_box.push_back(p0);

    lanelet::Point3d p1(getId(), base_string.front().basicPoint());
    p1.z() += height;
    p1.attributes()["radius"] = 0.1;
    p1.attributes()["color"] = "green";
    bounding_box.push_back(p1);

    lanelet::Point3d p2(getId(), base_string.back().basicPoint());
    p2.z() += height;
    p2.attributes()["radius"] = 0.1;
    p2.attributes()["color"] = "yellow";
    bounding_box.push_back(p2);

    lanelet::Point3d p3(getId(), base_string.back().basicPoint());
    p3.attributes()["radius"] = 0.1;
    p3.attributes()["color"] = "red";
    bounding_box.push_back(p3);
  }
  return static_cast<lanelet::ConstLineString3d>(bounding_box);
}

// find signals in camera frame and create Signals message
void FeatProjLanelet2::findSignalsInCameraFrame(const std::vector<lanelet::AutowareTrafficLightConstPtr>& visible_aw_tl,
                                                autoware_msgs::Signals* signal_in_frame)
{
  if (signal_in_frame == nullptr)
  {
    ROS_ERROR_STREAM(__FUNCTION__ << ": signal_in_frame is null pointer!");
    return;
  }
  for (auto tli = visible_aw_tl.begin(); tli != visible_aw_tl.end(); tli++)
  {
    lanelet::AutowareTrafficLightConstPtr tl = *tli;
    lanelet::ConstLineStrings3d lights;

    if (!tl->lightBulbs().empty())
    {
      lights = tl->lightBulbs();
    }
    else
    {
      // if there is no light bulb strings, create dummy light_bulb string from base linestring.
      // The dummy lightbulbs will be created at four corners of traffic light.
      // This allows context library to create appropriate bounding box for some recognizers region_tlr_mxnet
      for (const auto& ls_or_poly : tl->trafficLights())
      {
        if (ls_or_poly.isLineString())
        {
          lanelet::ConstLineString3d base_string = static_cast<lanelet::ConstLineString3d>(ls_or_poly);
          lanelet::ConstLineString3d bounding_box = createDummyLightBulbString(base_string);
          lights.push_back(bounding_box);
        }
      }
    }

    for (const auto& ls : lights)
    {
      for (const auto& p : ls)
      {
        double signal_radius = p.attributeOr("radius", DEFAULT_SIGNAL_LAMP_RADIUS);
        Eigen::Vector3f signal_center(p.x(), p.y(), p.z());
        Eigen::Vector3f signal_centerx(p.x(), p.y(), p.z() + signal_radius);  /// ????
        int u, v;

        if (project2(signal_center, &u, &v, false) == true)
        {
          int radius, ux, vx;
          project2(signal_centerx, &ux, &vx, false);
          radius = static_cast<int>((Eigen::Vector2f(ux - u, vx - v)).norm());

          autoware_msgs::ExtractedPosition sign;
          sign.signalId = p.id();
          sign.u = u + adjust_proj_x_;
          sign.v = v + adjust_proj_y_;

          sign.radius = radius;
          sign.x = signal_center.x(), sign.y = signal_center.y(), sign.z = signal_center.z();
          sign.hang = 0.0;  // angle [0, 360] degrees: not used in setContexts for region_tlr
          sign.type = 0;  // type should be int for color/form of bulb 1: RED, 2: GREEN 3: YELLOW // to do left arrorws
                          // etc 21, 22, 23
          if (isAttributeValue(p, "color", "red"))
            sign.type = 1;
          else if (isAttributeValue(p, "color", "yellow"))
            sign.type = 3;
          else if (isAttributeValue(p, "color", "green"))
            sign.type = 2;

          // only left arrow defined in trafficlight detection contexts
          if (p.hasAttribute("arrow"))
          {
            if (isAttributeValue(p, "arrow", "left"))
            {
              sign.type += 20;
            }
          }

          sign.linkId = ls.id();  // this should be lane id? -> traffic light reg elem id ok because unique to multiple
                                  // physical traffic lights
          sign.plId = ls.id();
          signal_in_frame->Signals.push_back(sign);
        }
      }  // for each point p
    }    // for each linestring ls
  }
}

FeatProjLanelet2::FeatProjLanelet2() : private_nh_("~")
{
  viz_tl_signs_ = true;
  loaded_lanelet_map_ = false;
}

void FeatProjLanelet2::init()
{
  visible_traffic_light_triangle_pub_ =
      rosnode_.advertise<visualization_msgs::MarkerArray>("visible_traffic_lights_triangle", 1);

  bin_map_sub_ = rosnode_.subscribe("lanelet_map_bin", 10, &FeatProjLanelet2::binMapCallback, this);

  while (ros::ok() && !loaded_lanelet_map_)
  {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  private_nh_.param<std::string>("camera_frame", camera_frame_, "camera");

  // subcribe to camera info & image etc
  camera_info_subscriber_ = rosnode_.subscribe("camera_info", 100, &FeatProjLanelet2::cameraInfoCallback, this);
  adjustXY_subscriber_ = rosnode_.subscribe("config/adjust_xy", 100, &FeatProjLanelet2::adjustXYCallback, this);

  // publisher to pub regions of interest: ie areas in image where traffic lights should be
  roi_sign_pub_ = rosnode_.advertise<autoware_msgs::Signals>("roi_signal", 100);

  private_nh_.param<bool>("use_path_info", use_path_info_, false);

  waypoint_subscriber_ = rosnode_.subscribe("final_waypoints", 1, &FeatProjLanelet2::waypointsCallback, this);

  if (use_path_info_)
  {
    // location Germany is used as default value for Lanelet. We may want to create Locations::Autoware to avoid
    // confusion.
    lanelet::traffic_rules::TrafficRulesPtr trafficRules = lanelet::traffic_rules::TrafficRulesFactory::create(
        lanelet::Locations::Germany, lanelet::Participants::Vehicle);
    routing_graph_ptr_ = lanelet::routing::RoutingGraph::build(*lanelet_map_, *trafficRules);
  }
}

void FeatProjLanelet2::run()
{
  ros::Rate loop_rate(10);
  Eigen::Vector3f pos;
  Eigen::Quaternionf ori;
  Eigen::Vector3f prev_position(0, 0, 0);
  Eigen::Quaternionf prev_orientation(0, 0, 0, 0);

  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_);
  std::vector<lanelet::AutowareTrafficLightConstPtr> aw_tl_reg_elems =
      lanelet::utils::query::autowareTrafficLights(all_lanelets);

  std_msgs::ColorRGBA cl;
  cl.r = 0.9;
  cl.g = 0.7;
  cl.b = 0.3;
  cl.a = 0.9;

  std::vector<lanelet::AutowareTrafficLightConstPtr> visible_aw_tl;

  while (ros::ok())
  {
    ros::spinOnce();

    try
    {
      getTransform(camera_frame_, "map", &orientation_, &position_, &camera_to_map_tf_);
      getTransform("map", camera_frame_, &orientation_, &position_, &map_to_camera_tf_);
    }
    catch (tf::TransformException& exc)
    {
      ROS_WARN_STREAM(exc.what());
    }

    if (prev_orientation.vec() != orientation_.vec() && prev_position != position_)
    {
      visible_aw_tl.clear();

      if (use_path_info_)
      {
        // match waypoints and lanelets, then find relevant traffic lights to matched lanelet
        findVisibleTrafficLightFromLanes(&visible_aw_tl);
      }
      else
      {
        // check if traffic light regulatory elements are potentially in camera view field
        // Future: should pre select candidates by lanelets on current waypoint path
        trafficLightVisibilityCheck(aw_tl_reg_elems, &visible_aw_tl);
      }

      // int tl_count = 0;
      autoware_msgs::Signals signal_in_frame;

      // project light bulbs into camera frame - create roi signal
      findSignalsInCameraFrame(visible_aw_tl, &signal_in_frame);

      // publish signals detected in camera frame
      signal_in_frame.header.stamp = ros::Time::now();
      roi_sign_pub_.publish(signal_in_frame);

      if (viz_tl_signs_)
      {
        if (visible_aw_tl.size() > 0)
        {
          std::vector<lanelet::TrafficLightConstPtr> visible_tl(visible_aw_tl.begin(), visible_aw_tl.end());
          visible_traffic_light_triangle_pub_.publish(
              lanelet::visualization::trafficLightsAsTriangleMarkerArray(visible_tl, cl, ros::Duration(1.0), 1.2));
        }
      }
    }

    prev_orientation = orientation_;
    prev_position = position_;

    // ros loop stuff
    loop_rate.sleep();
  }
}
}  // namespace trafficlight_recognizer
