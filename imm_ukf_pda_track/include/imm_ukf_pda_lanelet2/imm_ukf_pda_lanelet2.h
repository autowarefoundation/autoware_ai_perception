/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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

#ifndef IMM_UKF_PDA_LANELET2_IMM_UKF_PDA_LANELET2_H
#define IMM_UKF_PDA_LANELET2_IMM_UKF_PDA_LANELET2_H

#include <string>
#include <vector>

#include <ros/package.h>
#include <ros/ros.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_listener.h>

#include <vector_map/vector_map.h>

#include <autoware_msgs/DetectedObject.h>
#include <autoware_msgs/DetectedObjectArray.h>

#include <autoware_lanelet2_msgs/MapBin.h>
#include <lanelet2_core/LaneletMap.h>

#include <lanelet2_extension/projection/mgrs_projector.h>
#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_extension/visualization/visualization.h>

#include <imm_ukf_pda/ukf.h>

struct ImmUkfPdaParam
{
  // switch sukf and ImmUkfPda
  bool use_sukf_;

  // probabilistic data association params
  double gating_threshold_;
  double gate_probability_;
  double detection_probability_;

  // object association param
  int life_time_threshold_;

  // static classification param
  double static_velocity_threshold_;
  int static_num_history_threshold_;

  // prevent explode param for ukf
  double prevent_explosion_threshold_;

  // for lanelet2 assisted tarcking
  double lane_direction_chi_threshold_;
  double nearest_lane_distance_threshold_;

  double merge_distance_threshold_;
};

class ImmUkfPdaLanelet2
{
private:
  int target_id_;
  bool is_tracker_initialized_;
  double last_timestamp_;

  ImmUkfPdaParam param_;

  std::vector<UKF> tracking_targets_;

  // whether if benchmarking tracking result
  bool is_benchmark_;
  int frame_count_;
  std::string kitti_data_dir_;

  // for benchmark
  std::string result_file_path_;

  // for lanelet2 assisted tarcking
  bool use_map_info_;
  bool has_loaded_lanelet_map_;

  std::string map_frame_;
  lanelet::LaneletMapPtr lanelet_map_;

  static constexpr double CENTROID_DISTANCE = 0.2;  // distance to consider centroids the same

  std::string tracking_frame_;

  tf::TransformListener tf_listener_;
  tf::StampedTransform tf_local2global_;
  tf::StampedTransform tf_map2tracking_;
  tf::Transform tf_tracking2map_;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber sub_detected_array_;
  ros::Publisher pub_object_array_;

  std_msgs::Header input_header_;

  void callback(const autoware_msgs::DetectedObjectArray& input);

  void binMapCallback(const autoware_lanelet2_msgs::MapBin& msg);

  void transformPoseToGlobal(const autoware_msgs::DetectedObjectArray& input,
                             autoware_msgs::DetectedObjectArray* transformed_input_ptr);
  void transformPoseToLocal(autoware_msgs::DetectedObjectArray* detected_objects_output_ptr);

  bool updateNecessaryTransform();

  void measurementValidation(const autoware_msgs::DetectedObjectArray& input, UKF* target_ptr, const bool second_init,
                             const Eigen::VectorXd& max_det_z, const Eigen::MatrixXd& max_det_s,
                             std::vector<autoware_msgs::DetectedObject>* object_vec_ptr,
                             std::vector<bool>* matching_vec_ptr);
  void updateBehaviorState(const UKF& target, const bool use_sukf, autoware_msgs::DetectedObject* object_ptr);

  void initTracker(const autoware_msgs::DetectedObjectArray& input, double timestamp);
  void secondInit(UKF* target_ptr, const std::vector<autoware_msgs::DetectedObject>& object_vec, double dt);

  void updateTrackingNum(const std::vector<autoware_msgs::DetectedObject>& object_vec, UKF* target_ptr);

  bool probabilisticDataAssociation(const autoware_msgs::DetectedObjectArray& input, const double dt,
                                    std::vector<bool>* matching_vec,
                                    std::vector<autoware_msgs::DetectedObject>* object_vec_ptr, UKF* target_ptr);
  void makeNewTargets(const double timestamp, const autoware_msgs::DetectedObjectArray& input,
                      const std::vector<bool>& matching_vec);

  void staticClassification();

  void makeOutput(const autoware_msgs::DetectedObjectArray& input, const std::vector<bool>& matching_vec,
                  autoware_msgs::DetectedObjectArray* detected_objects_output_ptr);

  void removeUnnecessaryTarget();

  void dumpResultText(const autoware_msgs::DetectedObjectArray& detected_objects);

  void tracker(const autoware_msgs::DetectedObjectArray& transformed_input,
               autoware_msgs::DetectedObjectArray* detected_objects_output_ptr);

  bool updateDirection(const double smallest_nis, const autoware_msgs::DetectedObject& in_object,
                       autoware_msgs::DetectedObject* out_object_ptr, UKF* target_ptr);

  bool storeObjectWithNearestLaneDirection(const autoware_msgs::DetectedObject& in_object,
                                           autoware_msgs::DetectedObject* out_object_ptr);

  autoware_msgs::DetectedObjectArray removeRedundantObjects(
      const autoware_msgs::DetectedObjectArray& in_detected_objects, const std::vector<size_t>& in_tracker_indices);

  autoware_msgs::DetectedObjectArray forwardNonMatchedObject(const autoware_msgs::DetectedObjectArray& tmp_objects,
                                                             const autoware_msgs::DetectedObjectArray& input,
                                                             const std::vector<bool>& matching_vec);

  bool arePointsClose(const geometry_msgs::Point& in_point_a, const geometry_msgs::Point& in_point_b,
                      float in_radius = CENTROID_DISTANCE);

  bool isPointInPool(const std::vector<geometry_msgs::Point>& in_pool, const geometry_msgs::Point& in_point);

  void updateTargetWithAssociatedObject(const std::vector<autoware_msgs::DetectedObject>& object_vec, UKF* target_ptr);

public:
  ImmUkfPdaLanelet2();
  void run();
};

#endif  // IMM_UKF_PDA_LANELET2_IMM_UKF_PDA_LANELET2_H
