/*
 * Copyright 2015-2020 Autoware Foundation. All rights reserved.
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
 */

#include <cmath>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointField.h>

#include "points_preprocessor/ray_ground_filter/ray_ground_filter.h"
#include "sensor_msg_serialization.h"
#include "sensor_msg_comparison.h"

TEST(RayGroundFilter, callback)
{
  char arg0[] = "test_points_preprocessor";
  char* argv[] = {&arg0[0], NULL};
  int argc = 1;
  ros::init(argc, argv, "test_raygroundfilter_callback");
  std::string package_path = ros::package::getPath("points_preprocessor");
  std::string test_data_path = package_path + "/test/data";

  RayGroundFilter rgfilter;
  rgfilter.general_max_slope_ = 5.0;
  rgfilter.local_max_slope_ = 8.0;
  rgfilter.concentric_divider_distance_ = 0.0;
  rgfilter.min_height_threshold_ = 0.5;
  rgfilter.clipping_height_ = 2.0;
  rgfilter.min_point_distance_ = 1.85;
  rgfilter.reclass_distance_threshold_ = 0.2;
  rgfilter.radial_divider_angle_ = 0.08;
  rgfilter.radial_dividers_num_ = ceil(360.0 / rgfilter.radial_divider_angle_);

  // 0.2% of mismatch accepted. This is because of the floating points error possibly being different
  //from the the error on the machine on which the expected data was created. This would lead to a different
  //sort radius, a different sort order or a different verdict on being in a cone or not. All those errors may
  //misclassify a point. The ray ground filter algorithm is based upon heuristics, so having a few points
  //misclassified is acceptable.
  const float error_margin = 0.002;  // 1 out of 500

  sensor_msgs::PointCloud2::Ptr input_cloud(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr expected_trans_sensor_cloud(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr expected_ground_output_cloud(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr expected_no_ground_output_cloud(new sensor_msgs::PointCloud2);
  ASSERT_TRUE(unserialize_sensor_msg(input_cloud,
                                    test_data_path+"/input_structured.txt",
                                    test_data_path+"/input_blob.bin"))
                                    << "cannot find the input sensor_msg files";
  ASSERT_TRUE(unserialize_sensor_msg(expected_trans_sensor_cloud,
                                    test_data_path+"/transformed_structured.txt",
                                    test_data_path+"/transformed_blob.bin"))
                                    << "cannot find the transformed sensor_msg files";
  ASSERT_TRUE(unserialize_sensor_msg(expected_ground_output_cloud,
                                    test_data_path+"/output_ground_structured.txt",
                                    test_data_path+"/output_ground_blob.bin"))
                                    << "cannot find the ground output sensor_msg files";
  ASSERT_TRUE(unserialize_sensor_msg(expected_no_ground_output_cloud,
                                    test_data_path+"/output_no_ground_structured.txt",
                                    test_data_path+"/output_no_ground_blob.bin"))
                                    << "cannot find the no_ground output sensor_msg files";
  const int expected_ground_points = expected_ground_output_cloud->width*expected_ground_output_cloud->height;
  const int expected_no_ground_points = expected_no_ground_output_cloud->width*expected_no_ground_output_cloud->height;

  const uint error_allowed = error_margin * (expected_ground_points+expected_no_ground_points);

  //apply the callback's functions
  sensor_msgs::PointCloud2::Ptr trans_sensor_cloud(new sensor_msgs::PointCloud2);
  const bool succeeded = rgfilter.TransformPointCloud("base_link", input_cloud, trans_sensor_cloud);
  EXPECT_TRUE(succeeded) << "cannot transform";
  EXPECT_TRUE(equals(*trans_sensor_cloud, *expected_trans_sensor_cloud)) << "we don't get the expected transform";

  std::vector<RayGroundFilter::PointCloudRH> radial_ordered_clouds;
  std::vector<void*> ground_ptrs, no_ground_ptrs;
  rgfilter.ConvertAndTrim(expected_trans_sensor_cloud, rgfilter.clipping_height_, rgfilter.min_point_distance_, &radial_ordered_clouds, &no_ground_ptrs);
  const size_t point_count = input_cloud->width*input_cloud->height;

  rgfilter.ClassifyPointCloud(radial_ordered_clouds, point_count, &ground_ptrs, &no_ground_ptrs);
  EXPECT_LE(static_cast<uint>(std::abs(static_cast<int>(ground_ptrs.size())-expected_ground_points)), error_allowed)
    << "Wrong number of ground point";
  EXPECT_LE(static_cast<uint>(std::abs(static_cast<int>(no_ground_ptrs.size())-expected_no_ground_points)), error_allowed)
    << "Wrong number of non_ground point";

  sensor_msgs::PointCloud2::Ptr produced_ground_output_cloud(new sensor_msgs::PointCloud2);
  sensor_msgs::PointCloud2::Ptr produced_no_ground_output_cloud(new sensor_msgs::PointCloud2);
  rgfilter.filterROSMsg(input_cloud, ground_ptrs, produced_ground_output_cloud);
  rgfilter.filterROSMsg(input_cloud, no_ground_ptrs, produced_no_ground_output_cloud);

  produced_ground_output_cloud->header = expected_ground_output_cloud->header;
  produced_no_ground_output_cloud->header = expected_no_ground_output_cloud->header;

  bool ground_output_is_as_expected = equals(*produced_ground_output_cloud, *expected_ground_output_cloud);
  bool no_ground_output_is_as_expected = equals(*produced_no_ground_output_cloud, *expected_no_ground_output_cloud);

  if ( !ground_output_is_as_expected || !no_ground_output_is_as_expected )
  {
    uint false_ground_count;
    uint false_no_ground_count;
    bool all_legit = count_data_mismatch(*expected_ground_output_cloud,
                                          *expected_no_ground_output_cloud,
                                          *produced_ground_output_cloud,
                                          *produced_no_ground_output_cloud,
                                          &false_ground_count,
                                          &false_no_ground_count);

    uint error_count = false_ground_count + false_no_ground_count;
    if (!ground_output_is_as_expected)
    {
      if (error_count > error_allowed)
      {
        std::string diff = find_mismatches(*expected_ground_output_cloud, *produced_ground_output_cloud,true);
        EXPECT_TRUE(ground_output_is_as_expected) << "ground points output doesn't match the expected result :" << std::endl
                                                  << diff << std::endl
                                                  << false_ground_count << " false ground found" << std::endl;
      }
    }

    if (!no_ground_output_is_as_expected || !all_legit)
    {
      if (error_count > error_allowed)
      {
        std::string diff = find_mismatches(*expected_no_ground_output_cloud, *produced_no_ground_output_cloud,true);
        EXPECT_TRUE(no_ground_output_is_as_expected) << "no_ground points output doesn't match the expected result :" << std::endl
                                                    << diff << std::endl
                                                    << false_no_ground_count << " false no_ground found" << std::endl;
      }
    }
  }
}
