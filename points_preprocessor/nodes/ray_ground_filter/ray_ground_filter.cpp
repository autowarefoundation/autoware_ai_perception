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
#include <iostream>
#include <algorithm>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <pcl_ros/transforms.h>
#include <velodyne_pointcloud/point_types.h>

#include "autoware_config_msgs/ConfigRayGroundFilter.h"
#include "points_preprocessor/ray_ground_filter/ray_ground_filter.h"
#include "points_preprocessor/ray_ground_filter/atan2_utils.h"

void RayGroundFilter::update_config_params(const autoware_config_msgs::ConfigRayGroundFilter::ConstPtr& param)
{
  general_max_slope_ = param->general_max_slope;
  local_max_slope_ = param->local_max_slope;
  radial_divider_angle_ = param->radial_divider_angle;
  concentric_divider_distance_ = param->concentric_divider_distance;
  min_height_threshold_ = param->min_height_threshold;
  clipping_height_ = param->clipping_height;
  min_point_distance_ = param->min_point_distance;
  reclass_distance_threshold_ = param->reclass_distance_threshold;
  radial_dividers_num_ = ceil(360.0 / radial_divider_angle_);
}

/*!
 * Output transformed PointCloud from in_cloud_ptr->header.frame_id to in_target_frame
 * @param in_target_frame Coordinate system to perform transform
 * @param in_cloud_ptr PointCloud to perform transform
 * @param out_cloud_ptr Resulting transformed PointCloud
 * @retval true transform successed
 * @retval false transform faild
 */
bool RayGroundFilter::TransformPointCloud(const std::string& in_target_frame,
                                          const sensor_msgs::PointCloud2::ConstPtr& in_cloud_ptr,
                                          const sensor_msgs::PointCloud2::Ptr& out_cloud_ptr)
{
  if (in_target_frame == in_cloud_ptr->header.frame_id)
  {
    *out_cloud_ptr = *in_cloud_ptr;
    return true;
  }

  geometry_msgs::TransformStamped transform_stamped;
  try
  {
    transform_stamped = tf_buffer_.lookupTransform(in_target_frame, in_cloud_ptr->header.frame_id,
                                                   in_cloud_ptr->header.stamp, ros::Duration(1.0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return false;
  }
  // tf2::doTransform(*in_cloud_ptr, *out_cloud_ptr, transform_stamped);
  Eigen::Matrix4f mat = tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
  pcl_ros::transformPointCloud(mat, *in_cloud_ptr, *out_cloud_ptr);
  out_cloud_ptr->header.frame_id = in_target_frame;
  return true;
}

/*!
 * Extract the points pointed by in_selector from in_radial_ordered_clouds to copy them in out_no_ground_ptrs
 * @param pub The ROS publisher on which to output the point cloud
 * @param in_sensor_cloud The input point cloud from which to select the points to publish
 * @param in_selector The pointers to the input cloud's binary blob. No checks are done so be carefull
 */
void RayGroundFilter::publish(ros::Publisher pub,
                               const sensor_msgs::PointCloud2ConstPtr in_sensor_cloud,
                               const std::vector<void*>& in_selector)
{
  sensor_msgs::PointCloud2::Ptr output_cloud(new sensor_msgs::PointCloud2);
  filterROSMsg(in_sensor_cloud, in_selector, output_cloud);
  pub.publish(*output_cloud);
}

/*!
 * Extract the points pointed by in_selector from in_radial_ordered_clouds to copy them in out_no_ground_ptrs
 * @param in_origin_cloud The original cloud from which we want to copy the points
 * @param in_selector The pointers to the input cloud's binary blob. No checks are done so be carefull
 * @param out_filtered_msg Returns a cloud comprised of the selected points from the origin cloud
 */
void RayGroundFilter::filterROSMsg(const sensor_msgs::PointCloud2ConstPtr in_origin_cloud,
                   const std::vector<void*>& in_selector,
                   const sensor_msgs::PointCloud2::Ptr out_filtered_msg)
{
  size_t point_size = in_origin_cloud->row_step/in_origin_cloud->width;  // in Byte

  // TODO(yoan picchi) I fear this may do a lot of cache miss because it is sorted in the radius
  // and no longer sorted in the original pointer. One thing to try is that, given
  // that we know the value possibles, we can make a rather large vector and insert
  // all the point in, then move things around to remove the "blank" space. This
  // would be a linear sort to allow cache prediction to work better. To be tested.

  size_t data_size = point_size * in_selector.size();
  out_filtered_msg->data.resize(data_size);  // TODO(yoan picchi) a fair amount of time (5-10%) is wasted on this resize

  size_t offset = 0;
  for ( auto it = in_selector.cbegin(); it != in_selector.cend(); it++ )
  {
    memcpy(out_filtered_msg->data.data()+offset, *it, point_size);
    offset += point_size;
  }

  out_filtered_msg->width  = (uint32_t) in_selector.size();
  out_filtered_msg->height = 1;

  out_filtered_msg->fields            = in_origin_cloud->fields;
  out_filtered_msg->header.frame_id   = base_frame_;
  out_filtered_msg->header.stamp      = in_origin_cloud->header.stamp;
  out_filtered_msg->point_step        = in_origin_cloud->point_step;
  out_filtered_msg->row_step          = point_size * in_selector.size();
  out_filtered_msg->is_dense          = in_origin_cloud->is_dense
                                        && in_origin_cloud->data.size() == in_selector.size();
}

/*!
 * Classifies Points in the PointCoud as Ground and Not Ground
 * @param in_radial_ordered_clouds Vector of an Ordered PointsCloud ordered by radial distance from the origin
 * @param in_point_count Total number of lidar point. This is used to reserve the output's vector memory
 * @param out_ground_ptrs Returns the original adress of the points classified as ground in the original PointCloud
 * @param out_no_ground_ptrs Returns the original adress of the points classified as not ground in the original PointCloud
 */
void RayGroundFilter::ClassifyPointCloud(const std::vector<PointCloudRH>& in_radial_ordered_clouds,
                                         const size_t in_point_count,
                                         std::vector<void*>* out_ground_ptrs,
                                         std::vector<void*>* out_no_ground_ptrs)
{
  double expected_ground_no_ground_ratio = 0.1;
  out_ground_ptrs->reserve(in_point_count * expected_ground_no_ground_ratio);
  out_no_ground_ptrs->reserve(in_point_count);

  const float local_slope_ratio = tan(DEG2RAD(local_max_slope_));
  const float general_slope_ratio = tan(DEG2RAD(general_max_slope_));
  for (size_t i = 0; i < in_radial_ordered_clouds.size(); i++)  // sweep through each radial division
  {
    float prev_radius = 0.f;
    float prev_height = 0.f;
    bool prev_ground = false;
    bool current_ground = false;
    for (size_t j = 0; j < in_radial_ordered_clouds[i].size(); j++)  // loop through each point in the radial div
    {
      float points_distance = in_radial_ordered_clouds[i][j].radius - prev_radius;
      float height_threshold = local_slope_ratio * points_distance;
      float current_height = in_radial_ordered_clouds[i][j].height;
      float general_height_threshold = general_slope_ratio * in_radial_ordered_clouds[i][j].radius;

      // for points which are very close causing the height threshold to be tiny, set a minimum value
      if (points_distance > concentric_divider_distance_ && height_threshold < min_height_threshold_)
      {
        height_threshold = min_height_threshold_;
      }

      // check current point height against the LOCAL threshold (previous point)
      if (current_height <= (prev_height + height_threshold) && current_height >= (prev_height - height_threshold))
      {
        // Check again using general geometry (radius from origin) if previous points wasn't ground
        if (!prev_ground)
        {
          if (current_height <= general_height_threshold && current_height >= -general_height_threshold)
          {
            current_ground = true;
          }
          else
          {
            current_ground = false;
          }
        }
        else
        {
          current_ground = true;
        }
      }
      else
      {
        // check if previous point is too far from previous one, if so classify again
        if (points_distance > reclass_distance_threshold_ &&
            (current_height <= height_threshold && current_height >= -height_threshold))
        {
          current_ground = true;
        }
        else
        {
          current_ground = false;
        }
      }

      if (current_ground)
      {
        out_ground_ptrs->push_back(in_radial_ordered_clouds[i][j].original_data_pointer);
        prev_ground = true;
      }
      else
      {
        out_no_ground_ptrs->push_back(in_radial_ordered_clouds[i][j].original_data_pointer);
        prev_ground = false;
      }

      prev_radius = in_radial_ordered_clouds[i][j].radius;
      prev_height = in_radial_ordered_clouds[i][j].height;
    }
  }
}

float ReverseFloat(float inFloat)  // Swap endianness
{
  float retVal;
  char *floatToConvert = reinterpret_cast<char*>(& inFloat);
  char *returnFloat = reinterpret_cast<char*>(& retVal);

  // swap the bytes into a temporary buffer
  returnFloat[0] = floatToConvert[3];
  returnFloat[1] = floatToConvert[2];
  returnFloat[2] = floatToConvert[1];
  returnFloat[3] = floatToConvert[0];

  return retVal;
}

bool is_big_endian(void)
{
  union
  {
    uint32_t i;
    char c[4];
  } bint = {0x01020304};

  return bint.c[0] == 1;
}

/*!
 * Convert the sensor_msgs::PointCloud2 into PointCloudRH and filter out the points too high or too close
 * @param in_transformed_cloud Input Point Cloud to be organized in radial segments
 * @param in_clip_height Maximum allowed height in the cloud
 * @param in_min_distance Minimum valid distance, points closer than this will be removed.
 * @param out_radial_ordered_clouds Vector of Points Clouds, each element will contain the points ordered
 * @param out_no_ground_ptrs Returns the pointers to the points filtered out as no ground
 */
void RayGroundFilter::ConvertAndTrim(const sensor_msgs::PointCloud2::Ptr in_transformed_cloud,
                      const double in_clip_height,
                      double in_min_distance,
                      std::vector<PointCloudRH>* out_radial_ordered_clouds,
                      std::vector<void*>* out_no_ground_ptrs)
{
  // --- Clarify some of the values used to access the binary blob
  size_t point_size = in_transformed_cloud->row_step/in_transformed_cloud->width;  // in Byte
  size_t cloud_count = in_transformed_cloud->width*in_transformed_cloud->height;

  const uint offset_not_set = ~0;
  uint x_offset = offset_not_set;  // in Byte from the point's start
  uint y_offset = offset_not_set;  // in Byte from the point's start
  uint z_offset = offset_not_set;  // in Byte from the point's start

  if (in_transformed_cloud->fields.size() < 3)
  {
    ROS_ERROR_STREAM_THROTTLE(10, "Failed to decode the pointcloud message : not enough fields found : "
        << in_transformed_cloud->fields.size() << " (needs at least 3 : x,y,z)");
    return;
  }

  for ( uint i = 0; i < in_transformed_cloud->fields.size(); i++ )
  {
    sensor_msgs::PointField field = in_transformed_cloud->fields[i];
    if ("x" == field.name)
    {
      x_offset = field.offset;
    }
    else if ("y" == field.name)
    {
      y_offset = field.offset;
    }
    else if ("z" == field.name)
    {
      z_offset = field.offset;
    }
  }

  if (offset_not_set == x_offset || offset_not_set == y_offset || offset_not_set == z_offset)
  {
    ROS_ERROR_STREAM_THROTTLE(10, "Failed to decode the pointcloud message : bad coordinate field name");
    return;
  }
  // ---

  out_radial_ordered_clouds->resize(radial_dividers_num_);

  const int mean_ray_count = cloud_count/radial_dividers_num_;
  // In theory reserving more than the average memory would reduce even more the number of realloc
  // but it would also make the reserving takes longer. One or two times the average are pretty
  // much identical in term of speedup. Three seems a bit worse.
  const int reserve_count = mean_ray_count;
  for (auto it = out_radial_ordered_clouds->begin(); it != out_radial_ordered_clouds->end(); it++)
  {
    it->reserve(reserve_count);
  }

  for ( size_t i = 0; i < cloud_count; i++ )
  {
    // --- access the binary blob fields
    uint8_t* point_start_ptr = reinterpret_cast<uint8_t*>(in_transformed_cloud->data.data()) + (i*point_size);
    float x = *(reinterpret_cast<float*>(point_start_ptr+x_offset));
    float y = *(reinterpret_cast<float*>(point_start_ptr+y_offset));
    float z = *(reinterpret_cast<float*>(point_start_ptr+z_offset));
    if (is_big_endian() != in_transformed_cloud->is_bigendian)
    {
      x = ReverseFloat(x);
      y = ReverseFloat(y);
      z = ReverseFloat(z);
    }
    // ---

    if (z > in_clip_height)
    {
      out_no_ground_ptrs->push_back(point_start_ptr);
      continue;
    }
    auto radius = static_cast<float>(sqrt(x*x + y*y));
    if (radius < in_min_distance)
    {
      out_no_ground_ptrs->push_back(point_start_ptr);
      continue;
    }
#ifdef USE_ATAN_APPROXIMATION
    auto theta = static_cast<float>(fast_atan2(y, x) * 180 / M_PI);
#else
    auto theta = static_cast<float>(atan2(y, x) * 180 / M_PI);
#endif  // USE_ATAN_APPROXIMATION
    if (theta < 0)
    {
      theta += 360;
    }
    else if (theta >= 360)
    {
      theta -= 360;
    }

    // radial_divider_angle_ is computed so that
    // 360 / radial_divider_angle_ = radial_dividers_num_
    // Even though theta < 360, rounding error may make it so that
    // theta / radial_divider_angle_ >= radial_dividers_num_
    // which gives a radial_div one past the end. The modulo is here to fix
    // this rare case, wrapping the bad radial_div back to the first one.
    auto radial_div = (size_t)floor(theta / radial_divider_angle_) % radial_dividers_num_;
    out_radial_ordered_clouds->at(radial_div).emplace_back(z, radius, point_start_ptr);
  }  // end for

  // order radial points on each division
  auto strick_weak_radius_ordering = [](const PointRH& a, const PointRH& b)
  {
    if (a.radius < b.radius)
    {
      return true;
    }
    if (a.radius > b.radius)
    {
      return false;
    }
    // then the radius are equals. We add a secondary condition to keep the sort stable
    return a.original_data_pointer < b.original_data_pointer;
  };
  for (size_t i = 0; i < radial_dividers_num_; i++)
  {
    std::sort(out_radial_ordered_clouds->at(i).begin(),
              out_radial_ordered_clouds->at(i).end(),
              strick_weak_radius_ordering);
  }
}

void RayGroundFilter::CloudCallback(const sensor_msgs::PointCloud2ConstPtr& in_sensor_cloud)
{
  health_checker_ptr_->NODE_ACTIVATE();
  health_checker_ptr_->CHECK_RATE("topic_rate_points_raw_slow", 8, 5, 1, "topic points_raw subscribe rate slow.");

  sensor_msgs::PointCloud2::Ptr trans_sensor_cloud(new sensor_msgs::PointCloud2);
  const bool succeeded = TransformPointCloud(base_frame_, in_sensor_cloud, trans_sensor_cloud);
  if (!succeeded)
  {
    ROS_ERROR_STREAM_THROTTLE(10, "Failed transform from " << base_frame_ << " to "
                                                           << in_sensor_cloud->header.frame_id);
    return;
  }

  std::vector<PointCloudRH> radial_ordered_clouds;
  std::vector<void*> ground_ptrs, no_ground_ptrs;
  ConvertAndTrim(trans_sensor_cloud, clipping_height_, min_point_distance_, &radial_ordered_clouds, &no_ground_ptrs);
  const size_t point_count = in_sensor_cloud->width*in_sensor_cloud->height;

  ClassifyPointCloud(radial_ordered_clouds, point_count, &ground_ptrs, &no_ground_ptrs);

  publish(ground_points_pub_, in_sensor_cloud, ground_ptrs);
  publish(groundless_points_pub_, in_sensor_cloud, no_ground_ptrs);
}

RayGroundFilter::RayGroundFilter() : nh_(), pnh_("~"), tf_listener_(tf_buffer_)

{
  health_checker_ptr_ = std::make_shared<autoware_health_checker::HealthChecker>(nh_, pnh_);
  health_checker_ptr_->ENABLE();
}

void RayGroundFilter::Run()
{
  // Model   |   Horizontal   |   Vertical   | FOV(Vertical)    degrees / rads
  // ----------------------------------------------------------
  // HDL-64  |0.08-0.35(0.32) |     0.4      |  -24.9 <=x<=2.0   (26.9  / 0.47)
  // HDL-32  |     0.1-0.4    |     1.33     |  -30.67<=x<=10.67 (41.33 / 0.72)
  // VLP-16  |     0.1-0.4    |     2.0      |  -15.0<=x<=15.0   (30    / 0.52)
  // VLP-16HD|     0.1-0.4    |     1.33     |  -10.0<=x<=10.0   (20    / 0.35)
  ROS_INFO("Initializing Ground Filter, please wait...");
  pnh_.param<std::string>("input_point_topic", input_point_topic_, "points_raw");
  ROS_INFO("Input point_topic: %s", input_point_topic_.c_str());

  pnh_.param<std::string>("base_frame", base_frame_, "base_link");
  ROS_INFO("base_frame: %s", base_frame_.c_str());

  pnh_.param("general_max_slope", general_max_slope_, 3.0);
  ROS_INFO("general_max_slope[deg]: %f", general_max_slope_);

  pnh_.param("local_max_slope", local_max_slope_, 5.0);
  ROS_INFO("local_max_slope[deg]: %f", local_max_slope_);

  pnh_.param("radial_divider_angle", radial_divider_angle_, 0.1);  // 0.1 degree default
  ROS_INFO("radial_divider_angle[deg]: %f", radial_divider_angle_);
  pnh_.param("concentric_divider_distance", concentric_divider_distance_, 0.0);  // 0.0 meters default
  ROS_INFO("concentric_divider_distance[meters]: %f", concentric_divider_distance_);
  pnh_.param("min_height_threshold", min_height_threshold_, 0.05);  // 0.05 meters default
  ROS_INFO("min_height_threshold[meters]: %f", min_height_threshold_);
  pnh_.param("clipping_height", clipping_height_, 2.0);  // 2.0 meters default above the car
  ROS_INFO("clipping_height[meters]: %f", clipping_height_);
  pnh_.param("min_point_distance", min_point_distance_, 1.85);  // 1.85 meters default
  ROS_INFO("min_point_distance[meters]: %f", min_point_distance_);
  pnh_.param("reclass_distance_threshold", reclass_distance_threshold_, 0.2);  // 0.2 meters default
  ROS_INFO("reclass_distance_threshold[meters]: %f", reclass_distance_threshold_);


  radial_dividers_num_ = ceil(360.0 / radial_divider_angle_);
  ROS_INFO("Radial Divisions: %d", (int)radial_dividers_num_);

  std::string no_ground_topic, ground_topic;
  pnh_.param<std::string>("no_ground_point_topic", no_ground_topic, "points_no_ground");
  ROS_INFO("No Ground Output Point Cloud no_ground_point_topic: %s", no_ground_topic.c_str());
  pnh_.param<std::string>("ground_point_topic", ground_topic, "points_ground");
  ROS_INFO("Only Ground Output Point Cloud ground_topic: %s", ground_topic.c_str());

  ROS_INFO("Subscribing to... %s", input_point_topic_.c_str());
  points_node_sub_ = nh_.subscribe(input_point_topic_, 1, &RayGroundFilter::CloudCallback, this);

  config_node_sub_ =
      nh_.subscribe("config/ray_ground_filter", 1, &RayGroundFilter::update_config_params, this);

  groundless_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(no_ground_topic, 2);
  ground_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(ground_topic, 2);

  ROS_INFO("Ready");

  ros::spin();
}
