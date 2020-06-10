/*
 * Copyright 2017-2020 Autoware Foundation. All rights reserved.
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
 ********************
 *  v1.0: amc-nu (abrahammonrroy@yahoo.com)
 */
#ifndef POINTS_PREPROCESSOR_RAY_GROUND_FILTER_RAY_GROUND_FILTER_H
#define POINTS_PREPROCESSOR_RAY_GROUND_FILTER_RAY_GROUND_FILTER_H

#include <iostream>
#include <algorithm>
#include <vector>
#include <memory>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <velodyne_pointcloud/point_types.h>
#include "autoware_config_msgs/ConfigRayGroundFilter.h"

#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

// headers in Autoware Health Checker
#include <autoware_health_checker/health_checker/health_checker.h>

#define USE_ATAN_APPROXIMATION


class RayGroundFilter
{
private:
  std::shared_ptr<autoware_health_checker::HealthChecker> health_checker_ptr_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber points_node_sub_;
  ros::Subscriber config_node_sub_;
  ros::Publisher groundless_points_pub_;
  ros::Publisher ground_points_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string input_point_topic_;
  std::string base_frame_;

  double general_max_slope_;            // degrees
  double local_max_slope_;              // degrees
  double radial_divider_angle_;         // distance in rads between dividers
  double concentric_divider_distance_;  // distance in meters between concentric divisions
  double min_height_threshold_;         // minimum height threshold regardless the slope, useful for close points
  double clipping_height_;              // the points higher than this will be removed from the input cloud.
  double min_point_distance_;           // minimum distance from the origin to consider a point as valid
  double reclass_distance_threshold_;   // distance between points at which re classification will occur

  size_t radial_dividers_num_;
  size_t concentric_dividers_num_;


  struct PointRH
  {
    float height;
    float radius;  // cylindric coords on XY Plane
    void* original_data_pointer;

    PointRH(float height, float radius, void* original_data_pointer)
        : height(height), radius(radius), original_data_pointer(original_data_pointer)
    {}
  };
  typedef std::vector<PointRH> PointCloudRH;

  void update_config_params(const autoware_config_msgs::ConfigRayGroundFilter::ConstPtr& param);

  /*!
   * Output transformed PointCloud from in_cloud_ptr->header.frame_id to in_target_frame
   * @param in_target_frame Coordinate system to perform transform
   * @param in_cloud_ptr PointCloud to perform transform
   * @param out_cloud_ptr Resulting transformed PointCloud
   * @retval true transform successed
   * @retval false transform faild
   */
  bool TransformPointCloud(const std::string& in_target_frame, const sensor_msgs::PointCloud2::ConstPtr& in_cloud_ptr,
                           const sensor_msgs::PointCloud2::Ptr& out_cloud_ptr);

  /*!
   * Extract the points pointed by in_selector from in_radial_ordered_clouds to copy them in out_no_ground_ptrs
   * @param pub The ROS publisher on which to output the point cloud
   * @param in_sensor_cloud The input point cloud from which to select the points to publish
   * @param in_selector The pointers to the input cloud's binary blob. No checks are done so be carefull
   */
  void publish(ros::Publisher pub,
                const sensor_msgs::PointCloud2ConstPtr in_sensor_cloud,
                const std::vector<void*>& in_selector);

  /*!
   * Extract the points pointed by in_selector from in_radial_ordered_clouds to copy them in out_no_ground_ptrs
   * @param in_origin_cloud The original cloud from which we want to copy the points
   * @param in_selector The pointers to the input cloud's binary blob. No checks are done so be carefull
   * @param out_filtered_msg Returns a cloud comprised of the selected points from the origin cloud
   */
  void filterROSMsg(const sensor_msgs::PointCloud2ConstPtr in_origin_cloud,
                     const std::vector<void*>& in_selector,
                     const sensor_msgs::PointCloud2::Ptr out_filtered_msg);

  /*!
   * Classifies Points in the PointCoud as Ground and Not Ground
   * @param in_radial_ordered_clouds Vector of an Ordered PointsCloud ordered by radial distance from the origin
   * @param in_point_count Total number of lidar point. This is used to reserve the output's vector memory
   * @param out_ground_indices Returns the indices of the points classified as ground in the original PointCloud
   * @param out_no_ground_indices Returns the indices of the points classified as not ground in the original PointCloud
   */
  void ClassifyPointCloud(const std::vector<PointCloudRH>& in_radial_ordered_clouds,
                          const size_t in_point_count,
                          std::vector<void*>* out_ground_ptrs,
                          std::vector<void*>* out_no_ground_ptrs);

  /*!
   * Convert the sensor_msgs::PointCloud2 into PointCloudRH and filter out the points too high or too close
   * @param in_transformed_cloud Input Point Cloud to be organized in radial segments
   * @param in_clip_height Maximum allowed height in the cloud
   * @param in_min_distance Minimum valid distance, points closer than this will be removed.
   * @param out_radial_ordered_clouds Vector of Points Clouds, each element will contain the points ordered
   * @param out_no_ground_ptrs Returns the pointers to the points filtered out as no ground
   */
  void ConvertAndTrim(const sensor_msgs::PointCloud2::Ptr in_transformed_cloud,
                      const double in_clip_height,
                      double in_min_distance,
                      std::vector<PointCloudRH>* out_radial_ordered_clouds,
                      std::vector<void*>* out_no_ground_ptrs);

  void CloudCallback(const sensor_msgs::PointCloud2ConstPtr& in_sensor_cloud);

  friend class RayGroundFilter_callback_Test;

public:
  RayGroundFilter();
  void Run();
};

#endif  // POINTS_PREPROCESSOR_RAY_GROUND_FILTER_RAY_GROUND_FILTER_H
