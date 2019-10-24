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

#include <imm_ukf_pda_lanelet2/imm_ukf_pda_lanelet2.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Point.h>
#include <amathutils_lib/amathutils.hpp>

#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace
{
geometry_msgs::Pose getTransformedPose(const geometry_msgs::Pose& in_pose, const tf::Transform& tf_stamp)
{
  tf::Transform transform;
  geometry_msgs::PoseStamped out_pose;
  transform.setOrigin(tf::Vector3(in_pose.position.x, in_pose.position.y, in_pose.position.z));
  transform.setRotation(
      tf::Quaternion(in_pose.orientation.x, in_pose.orientation.y, in_pose.orientation.z, in_pose.orientation.w));
  geometry_msgs::PoseStamped pose_out;
  tf::poseTFToMsg(tf_stamp * transform, out_pose.pose);
  return out_pose.pose;
}
}  // namespace

ImmUkfPdaLanelet2::ImmUkfPdaLanelet2()
  : target_id_(0), is_tracker_initialized_(false), frame_count_(0), has_loaded_lanelet_map_(false), private_nh_("~")
{
  private_nh_.param<std::string>("tracking_frame", tracking_frame_, "world");
  private_nh_.param<int>("life_time_threshold", param_.life_time_threshold_, 8);
  private_nh_.param<double>("gating_threshold", param_.gating_threshold_, 9.22);
  private_nh_.param<double>("gate_probability", param_.gate_probability_, 0.99);
  private_nh_.param<double>("detection_probability", param_.detection_probability_, 0.9);
  private_nh_.param<double>("static_velocity_threshold", param_.static_velocity_threshold_, 0.5);
  private_nh_.param<int>("static_velocity_history_thres", param_.static_num_history_threshold_, 3);
  private_nh_.param<double>("prevent_explosion_threshold", param_.prevent_explosion_threshold_, 1000);
  private_nh_.param<double>("merge_distance_threshold", param_.merge_distance_threshold_, 0.5);
  private_nh_.param<bool>("use_sukf", param_.use_sukf_, false);

  // for lanelet map  assisted tracking
  private_nh_.param<bool>("use_map_info", use_map_info_, false);
  private_nh_.param<double>("lane_direction_chi_threshold", param_.lane_direction_chi_threshold_, 2.71);
  private_nh_.param<double>("nearest_lane_distance_threshold", param_.nearest_lane_distance_threshold_, 1.0);
  private_nh_.param<std::string>("map_frame", map_frame_, "map");

  // rosparam for benchmark
  private_nh_.param<bool>("is_benchmark", is_benchmark_, false);
  private_nh_.param<std::string>("kitti_data_dir", kitti_data_dir_, "");
  if (is_benchmark_)
  {
    result_file_path_ = kitti_data_dir_ + "benchmark_results.txt";
    std::remove(result_file_path_.c_str());
  }
}

void ImmUkfPdaLanelet2::run()
{
  ros::Subscriber sub = nh_.subscribe("lanelet_map_bin", 1, &ImmUkfPdaLanelet2::binMapCallback, this);

  if (use_map_info_)
  {
    while (ros::ok() && !has_loaded_lanelet_map_)
    {
      ros::spinOnce();
      ros::Duration(0.1).sleep();
    }
  }

  pub_object_array_ = nh_.advertise<autoware_msgs::DetectedObjectArray>("detection/objects", 1);
  sub_detected_array_ = nh_.subscribe("detection/fusion_tools/objects", 1, &ImmUkfPdaLanelet2::callback, this);
}

void ImmUkfPdaLanelet2::binMapCallback(const autoware_lanelet2_msgs::MapBin& msg)
{
  lanelet_map_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(msg, lanelet_map_);
  has_loaded_lanelet_map_ = true;
}

void ImmUkfPdaLanelet2::callback(const autoware_msgs::DetectedObjectArray& input)
{
  input_header_ = input.header;

  bool success = updateNecessaryTransform();
  if (!success)
  {
    ROS_WARN("Could not find coordiante transformation");
    return;
  }

  autoware_msgs::DetectedObjectArray transformed_input;
  autoware_msgs::DetectedObjectArray detected_objects_output;
  transformPoseToGlobal(input, &transformed_input);
  tracker(transformed_input, &detected_objects_output);
  transformPoseToLocal(&detected_objects_output);

  pub_object_array_.publish(detected_objects_output);

  if (is_benchmark_)
  {
    dumpResultText(detected_objects_output);
  }
}
bool ImmUkfPdaLanelet2::updateNecessaryTransform()
{
  bool success = true;
  try
  {
    tf_listener_.waitForTransform(input_header_.frame_id, tracking_frame_, ros::Time(0), ros::Duration(1.0));
    tf_listener_.lookupTransform(tracking_frame_, input_header_.frame_id, ros::Time(0), tf_local2global_);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    success = false;
  }
  if (use_map_info_ && has_loaded_lanelet_map_)
  {
    try
    {
      tf_listener_.waitForTransform(map_frame_, tracking_frame_, ros::Time(0), ros::Duration(1.0));
      tf_listener_.lookupTransform(tracking_frame_, map_frame_, ros::Time(0), tf_map2tracking_);
      tf_tracking2map_ = tf_map2tracking_.inverse();
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      success = false;
    }
  }
  return success;
}

void ImmUkfPdaLanelet2::transformPoseToGlobal(const autoware_msgs::DetectedObjectArray& input,
                                              autoware_msgs::DetectedObjectArray* transformed_input_ptr)
{
  transformed_input_ptr->header = input_header_;
  for (auto const& object : input.objects)
  {
    geometry_msgs::Pose out_pose = getTransformedPose(object.pose, tf_local2global_);

    autoware_msgs::DetectedObject dd;
    dd.header = input.header;
    dd = object;
    dd.pose = out_pose;

    transformed_input_ptr->objects.push_back(dd);
  }
}

void ImmUkfPdaLanelet2::transformPoseToLocal(autoware_msgs::DetectedObjectArray* detected_objects_output_ptr)
{
  detected_objects_output_ptr->header = input_header_;

  tf::Transform inv_local2global = tf_local2global_.inverse();
  tf::StampedTransform global2local;
  global2local.setData(inv_local2global);
  for (auto& object : detected_objects_output_ptr->objects)
  {
    geometry_msgs::Pose out_pose = getTransformedPose(object.pose, global2local);
    object.header = input_header_;
    object.pose = out_pose;
  }
}

void ImmUkfPdaLanelet2::measurementValidation(const autoware_msgs::DetectedObjectArray& input, UKF* target_ptr,
                                              const bool second_init, const Eigen::VectorXd& max_det_z,
                                              const Eigen::MatrixXd& max_det_s,
                                              std::vector<autoware_msgs::DetectedObject>* object_vec_ptr,
                                              std::vector<bool>* matching_vec_ptr)
{
  // alert: different from original imm-pda filter, here picking up most likely measurement
  // if making it allows to have more than one measurement, you will see non semipositive definite covariance
  bool exists_smallest_nis_object = false;
  double smallest_nis = std::numeric_limits<double>::max();
  int smallest_nis_ind = 0;
  for (size_t i = 0; i < input.objects.size(); i++)
  {
    const double x = input.objects[i].pose.position.x;
    const double y = input.objects[i].pose.position.y;

    Eigen::VectorXd meas = Eigen::VectorXd(2);
    meas << x, y;

    Eigen::VectorXd diff = meas - max_det_z;
    const double nis = diff.transpose() * max_det_s.inverse() * diff;

    if (nis < param_.gating_threshold_)
    {
      if (nis < smallest_nis)
      {
        smallest_nis = nis;
        target_ptr->object_ = input.objects[i];
        smallest_nis_ind = i;
        exists_smallest_nis_object = true;
      }
    }
  }
  if (exists_smallest_nis_object)
  {
    matching_vec_ptr->at(smallest_nis_ind) = true;
    if (use_map_info_ && has_loaded_lanelet_map_)
    {
      autoware_msgs::DetectedObject direction_updated_object;
      const bool use_direction_meas =
          updateDirection(smallest_nis, target_ptr->object_, &direction_updated_object, target_ptr);
      if (use_direction_meas)
      {
        object_vec_ptr->push_back(direction_updated_object);
      }
      else
      {
        object_vec_ptr->push_back(target_ptr->object_);
      }
    }
    else
    {
      object_vec_ptr->push_back(target_ptr->object_);
    }
  }
}

bool ImmUkfPdaLanelet2::updateDirection(const double smallest_nis, const autoware_msgs::DetectedObject& in_object,
                                        autoware_msgs::DetectedObject* out_object_ptr, UKF* target_ptr)
{
  bool use_lane_direction = false;
  target_ptr->is_direction_cv_available_ = false;
  target_ptr->is_direction_ctrv_available_ = false;
  const bool get_lane_success = storeObjectWithNearestLaneDirection(in_object, out_object_ptr);
  if (!get_lane_success)
  {
    return use_lane_direction;
  }
  target_ptr->checkLaneDirectionAvailability(*out_object_ptr, param_.lane_direction_chi_threshold_, param_.use_sukf_);
  if (target_ptr->is_direction_cv_available_ || target_ptr->is_direction_ctrv_available_)
  {
    use_lane_direction = true;
  }
  return use_lane_direction;
}

bool ImmUkfPdaLanelet2::storeObjectWithNearestLaneDirection(const autoware_msgs::DetectedObject& in_object,
                                                            autoware_msgs::DetectedObject* out_object_ptr)
{
  geometry_msgs::Pose map_frame_pose = getTransformedPose(in_object.pose, tf_tracking2map_);
  double min_dist = std::numeric_limits<double>::max();

  double min_yaw = 0;

  // find direction of lane neares object pose in map frame
  lanelet::BasicPoint2d obj_p(map_frame_pose.position.x, map_frame_pose.position.y);

  std::vector<std::pair<double, lanelet::Lanelet> > nearest_lls =
      lanelet::geometry::findNearest(lanelet_map_->laneletLayer, obj_p, 10);
  lanelet::Lanelet nearest_lanelet;
  bool found_road_lanelet = false;
  for (auto nll_pair : nearest_lls)
  {
    lanelet::Lanelet ll = nll_pair.second;
    if (ll.hasAttribute(lanelet::AttributeName::Subtype) &&
        ((ll.attribute(lanelet::AttributeName::Subtype).value()) == lanelet::AttributeValueString::Road))
    {
      nearest_lanelet = ll;
      found_road_lanelet = true;
      break;
    }
  }
  if (!found_road_lanelet)
    return false;

  lanelet::ConstLineString3d center_ls = nearest_lanelet.centerline();
  for (auto i = center_ls.begin(); i != center_ls.end(); i++)
  {
    lanelet::BasicPoint2d p = lanelet::utils::to2D(*i);

    double d = lanelet::geometry::distance(lanelet::utils::to2D(p), obj_p);

    if (d < min_dist)
    {
      min_dist = d;
      // calc heading of lane at point p
      lanelet::BasicPoint2d p1;
      if (i + 1 != center_ls.end())
        p1 = lanelet::utils::to2D(*(i + 1));
      else if (i != center_ls.begin())
      {
        p1 = p;
        p = lanelet::utils::to2D(*(i - 1));
      }
      else
      {
        return false;
      }
      min_yaw = std::atan2(p1.y() - p.y(), p1.x() - p.x());
    }
  }

  if (min_dist >= param_.nearest_lane_distance_threshold_)
  {
    return false;
  }

  // map yaw in rotation matrix representation
  tf::Quaternion map_quat = tf::createQuaternionFromYaw(min_yaw);
  tf::Matrix3x3 map_matrix(map_quat);

  // map_frame to tracking_frame rotation matrix
  tf::Quaternion rotation_quat = tf_map2tracking_.getRotation();
  tf::Matrix3x3 rotation_matrix(rotation_quat);

  // rotated yaw in matrix representation
  tf::Matrix3x3 rotated_matrix = rotation_matrix * map_matrix;
  double roll, pitch, yaw;
  rotated_matrix.getRPY(roll, pitch, yaw);

  *out_object_ptr = in_object;
  out_object_ptr->angle = yaw;
  return true;
}

void ImmUkfPdaLanelet2::updateTargetWithAssociatedObject(const std::vector<autoware_msgs::DetectedObject>& object_vec,
                                                         UKF* target_ptr)
{
  target_ptr->lifetime_++;
  if (!target_ptr->object_.label.empty() && target_ptr->object_.label != "unknown")
  {
    target_ptr->label_ = target_ptr->object_.label;
  }
  updateTrackingNum(object_vec, target_ptr);
  if (target_ptr->tracking_num_ == TrackingState::Stable || target_ptr->tracking_num_ == TrackingState::Occlusion)
  {
    target_ptr->is_stable_ = true;
  }
}

void ImmUkfPdaLanelet2::updateBehaviorState(const UKF& target, const bool use_sukf,
                                            autoware_msgs::DetectedObject* object_ptr)
{
  if (use_sukf)
  {
    object_ptr->behavior_state = MotionModel::CTRV;
  }
  else if (target.mode_prob_cv_ > target.mode_prob_ctrv_ && target.mode_prob_cv_ > target.mode_prob_rm_)
  {
    object_ptr->behavior_state = MotionModel::CV;
  }
  else if (target.mode_prob_ctrv_ > target.mode_prob_cv_ && target.mode_prob_ctrv_ > target.mode_prob_rm_)
  {
    object_ptr->behavior_state = MotionModel::CTRV;
  }
  else
  {
    object_ptr->behavior_state = MotionModel::RM;
  }
}

void ImmUkfPdaLanelet2::initTracker(const autoware_msgs::DetectedObjectArray& input, double timestamp)
{
  for (size_t i = 0; i < input.objects.size(); i++)
  {
    const double px = input.objects[i].pose.position.x;
    const double py = input.objects[i].pose.position.y;
    Eigen::VectorXd init_meas = Eigen::VectorXd(2);
    init_meas << px, py;

    UKF ukf;
    ukf.initialize(init_meas, timestamp, target_id_);
    tracking_targets_.push_back(ukf);
    target_id_++;
  }
  last_timestamp_ = timestamp;
  is_tracker_initialized_ = true;
}

void ImmUkfPdaLanelet2::secondInit(UKF* target_ptr, const std::vector<autoware_msgs::DetectedObject>& object_vec,
                                   double dt)
{
  if (object_vec.size() == 0)
  {
    target_ptr->tracking_num_ = TrackingState::Die;
    return;
  }
  // record init measurement for env classification
  target_ptr->init_meas_ << target_ptr->x_merge_(0), target_ptr->x_merge_(1);

  // state update
  const double target_x = object_vec[0].pose.position.x;
  const double target_y = object_vec[0].pose.position.y;
  const double target_diff_x = target_x - target_ptr->x_merge_(0);
  const double target_diff_y = target_y - target_ptr->x_merge_(1);
  const double target_yaw = amathutils::normalizeRadian(atan2(target_diff_y, target_diff_x));
  const double dist = sqrt(target_diff_x * target_diff_x + target_diff_y * target_diff_y);
  const double target_v = dist / dt;

  target_ptr->x_merge_(0) = target_ptr->x_cv_(0) = target_ptr->x_ctrv_(0) = target_ptr->x_rm_(0) = target_x;
  target_ptr->x_merge_(1) = target_ptr->x_cv_(1) = target_ptr->x_ctrv_(1) = target_ptr->x_rm_(1) = target_y;
  target_ptr->x_merge_(2) = target_ptr->x_cv_(2) = target_ptr->x_ctrv_(2) = target_ptr->x_rm_(2) = target_v;
  target_ptr->x_merge_(3) = target_ptr->x_cv_(3) = target_ptr->x_ctrv_(3) = target_ptr->x_rm_(3) = target_yaw;

  target_ptr->tracking_num_++;
  return;
}

void ImmUkfPdaLanelet2::updateTrackingNum(const std::vector<autoware_msgs::DetectedObject>& object_vec, UKF* target_ptr)
{
  if (!object_vec.empty())
  {
    if (target_ptr->tracking_num_ < TrackingState::Stable)
    {
      target_ptr->tracking_num_++;
    }
    else if (target_ptr->tracking_num_ == TrackingState::Stable)
    {
      target_ptr->tracking_num_ = TrackingState::Stable;
    }
    else if (target_ptr->tracking_num_ >= TrackingState::Stable && target_ptr->tracking_num_ < TrackingState::Lost)
    {
      target_ptr->tracking_num_ = TrackingState::Stable;
    }
    else if (target_ptr->tracking_num_ == TrackingState::Lost)
    {
      target_ptr->tracking_num_ = TrackingState::Die;
    }
  }
  else
  {
    if (target_ptr->tracking_num_ < TrackingState::Stable)
    {
      target_ptr->tracking_num_ = TrackingState::Die;
    }
    else if (target_ptr->tracking_num_ >= TrackingState::Stable && target_ptr->tracking_num_ < TrackingState::Lost)
    {
      target_ptr->tracking_num_++;
    }
    else if (target_ptr->tracking_num_ == TrackingState::Lost)
    {
      target_ptr->tracking_num_ = TrackingState::Die;
    }
  }

  return;
}

bool ImmUkfPdaLanelet2::probabilisticDataAssociation(const autoware_msgs::DetectedObjectArray& input, const double dt,
                                                     std::vector<bool>* matching_vec_ptr,
                                                     std::vector<autoware_msgs::DetectedObject>* object_vec_ptr,
                                                     UKF* target_ptr)
{
  double det_s = 0;
  Eigen::VectorXd max_det_z;
  Eigen::MatrixXd max_det_s;
  bool success = true;

  if (param_.use_sukf_)
  {
    max_det_z = target_ptr->z_pred_ctrv_;
    max_det_s = target_ptr->s_ctrv_;
    det_s = max_det_s.determinant();
  }
  else
  {
    // find maxDetS associated with predZ
    target_ptr->findMaxZandS(max_det_z, max_det_s);
    det_s = max_det_s.determinant();
  }

  // prevent ukf not to explode
  if (std::isnan(det_s) || det_s > param_.prevent_explosion_threshold_)
  {
    target_ptr->tracking_num_ = TrackingState::Die;
    success = false;
    return success;
  }

  bool is_second_init;
  if (target_ptr->tracking_num_ == TrackingState::Init)
  {
    is_second_init = true;
  }
  else
  {
    is_second_init = false;
  }

  // measurement gating
  measurementValidation(input, target_ptr, is_second_init, max_det_z, max_det_s, object_vec_ptr, matching_vec_ptr);

  // second detection for a target_ptr: update v and yaw
  if (is_second_init)
  {
    secondInit(target_ptr, *object_vec_ptr, dt);
    success = false;
    return success;
  }

  updateTargetWithAssociatedObject(*object_vec_ptr, target_ptr);

  if (target_ptr->tracking_num_ == TrackingState::Die)
  {
    success = false;
    return success;
  }
  return success;
}

void ImmUkfPdaLanelet2::makeNewTargets(const double timestamp, const autoware_msgs::DetectedObjectArray& input,
                                       const std::vector<bool>& matching_vec)
{
  for (size_t i = 0; i < input.objects.size(); i++)
  {
    if (matching_vec[i] == false)
    {
      double px = input.objects[i].pose.position.x;
      double py = input.objects[i].pose.position.y;
      Eigen::VectorXd init_meas = Eigen::VectorXd(2);
      init_meas << px, py;

      UKF ukf;
      ukf.initialize(init_meas, timestamp, target_id_);
      ukf.object_ = input.objects[i];
      tracking_targets_.push_back(ukf);
      target_id_++;
    }
  }
}

void ImmUkfPdaLanelet2::staticClassification()
{
  for (size_t i = 0; i < tracking_targets_.size(); i++)
  {
    // tracking_targets_[i].x_merge_(2) is referred for estimated velocity
    const double current_velocity = std::abs(tracking_targets_[i].x_merge_(2));
    tracking_targets_[i].vel_history_.push_back(current_velocity);
    if (tracking_targets_[i].tracking_num_ == TrackingState::Stable &&
        tracking_targets_[i].lifetime_ > param_.life_time_threshold_)
    {
      int index = 0;
      double sum_vel = 0;
      double avg_vel = 0;
      for (auto rit = tracking_targets_[i].vel_history_.rbegin(); index < param_.static_num_history_threshold_; ++rit)
      {
        index++;
        sum_vel += *rit;
      }
      avg_vel = static_cast<double>(sum_vel / param_.static_num_history_threshold_);

      if (avg_vel < param_.static_velocity_threshold_ && current_velocity < param_.static_velocity_threshold_)
      {
        tracking_targets_[i].is_static_ = true;
      }
    }
  }
}

bool ImmUkfPdaLanelet2::arePointsClose(const geometry_msgs::Point& in_point_a, const geometry_msgs::Point& in_point_b,
                                       float in_radius)
{
  return (std::abs(in_point_a.x - in_point_b.x) <= in_radius) && (std::abs(in_point_a.y - in_point_b.y) <= in_radius);
}

bool ImmUkfPdaLanelet2::isPointInPool(const std::vector<geometry_msgs::Point>& in_pool,
                                      const geometry_msgs::Point& in_point)
{
  for (size_t j = 0; j < in_pool.size(); j++)
  {
    if (arePointsClose(in_pool[j], in_point))
    {
      return true;
    }
  }
  return false;
}

autoware_msgs::DetectedObjectArray ImmUkfPdaLanelet2::removeRedundantObjects(
    const autoware_msgs::DetectedObjectArray& in_detected_objects, const std::vector<size_t>& in_tracker_indices)
{
  if (in_detected_objects.objects.size() != in_tracker_indices.size())
    return in_detected_objects;

  autoware_msgs::DetectedObjectArray resulting_objects;
  resulting_objects.header = in_detected_objects.header;

  std::vector<geometry_msgs::Point> centroids;
  // create unique points
  for (size_t i = 0; i < in_detected_objects.objects.size(); i++)
  {
    if (!isPointInPool(centroids, in_detected_objects.objects[i].pose.position))
    {
      centroids.push_back(in_detected_objects.objects[i].pose.position);
    }
  }
  // assign objects to the points
  std::vector<std::vector<size_t> > matching_objects(centroids.size());
  for (size_t k = 0; k < in_detected_objects.objects.size(); k++)
  {
    const auto& object = in_detected_objects.objects[k];
    for (size_t i = 0; i < centroids.size(); i++)
    {
      if (arePointsClose(object.pose.position, centroids[i], param_.merge_distance_threshold_))
      {
        matching_objects[i].push_back(k);  // store index of matched object to this point
      }
    }
  }
  // get oldest object on each point
  for (size_t i = 0; i < matching_objects.size(); i++)
  {
    size_t oldest_object_index = 0;
    int oldest_lifespan = -1;
    std::string best_label;
    for (size_t j = 0; j < matching_objects[i].size(); j++)
    {
      size_t current_index = matching_objects[i][j];
      int current_lifespan = tracking_targets_[in_tracker_indices[current_index]].lifetime_;
      if (current_lifespan > oldest_lifespan)
      {
        oldest_lifespan = current_lifespan;
        oldest_object_index = current_index;
      }
      if (!tracking_targets_[in_tracker_indices[current_index]].label_.empty() &&
          tracking_targets_[in_tracker_indices[current_index]].label_ != "unknown")
      {
        best_label = tracking_targets_[in_tracker_indices[current_index]].label_;
      }
    }
    // delete nearby targets except for the oldest target
    for (size_t j = 0; j < matching_objects[i].size(); j++)
    {
      size_t current_index = matching_objects[i][j];
      if (current_index != oldest_object_index)
      {
        tracking_targets_[in_tracker_indices[current_index]].tracking_num_ = TrackingState::Die;
      }
    }
    autoware_msgs::DetectedObject best_object;
    best_object = in_detected_objects.objects[oldest_object_index];
    if (best_label != "unknown" && !best_label.empty())
    {
      best_object.label = best_label;
    }

    resulting_objects.objects.push_back(best_object);
  }

  return resulting_objects;
}

void ImmUkfPdaLanelet2::makeOutput(const autoware_msgs::DetectedObjectArray& input,
                                   const std::vector<bool>& matching_vec,
                                   autoware_msgs::DetectedObjectArray* detected_objects_output_ptr)
{
  autoware_msgs::DetectedObjectArray tmp_objects;
  tmp_objects.header = input.header;
  std::vector<size_t> used_targets_indices;
  for (size_t i = 0; i < tracking_targets_.size(); i++)
  {
    const double& tx = tracking_targets_[i].x_merge_(0);
    const double& ty = tracking_targets_[i].x_merge_(1);

    const double& tv = tracking_targets_[i].x_merge_(2);
    const double tyaw = amathutils::normalizeRadian(tracking_targets_[i].x_merge_(3));
    const double& tyaw_rate = tracking_targets_[i].x_merge_(4);

    tf::Quaternion q = tf::createQuaternionFromYaw(tyaw);

    autoware_msgs::DetectedObject dd;
    dd = tracking_targets_[i].object_;
    dd.id = tracking_targets_[i].ukf_id_;
    dd.velocity.linear.x = tv;
    dd.acceleration.linear.y = tyaw_rate;
    dd.velocity_reliable = tracking_targets_[i].is_stable_;
    dd.pose_reliable = tracking_targets_[i].is_stable_;

    if (!tracking_targets_[i].is_static_ && tracking_targets_[i].is_stable_)
    {
      // Aligh the longest side of dimentions with the estimated orientation
      if (tracking_targets_[i].object_.dimensions.x < tracking_targets_[i].object_.dimensions.y)
      {
        dd.dimensions.x = tracking_targets_[i].object_.dimensions.y;
        dd.dimensions.y = tracking_targets_[i].object_.dimensions.x;
      }

      dd.pose.position.x = tx;
      dd.pose.position.y = ty;

      if (!std::isnan(q[0]))
        dd.pose.orientation.x = q[0];
      if (!std::isnan(q[1]))
        dd.pose.orientation.y = q[1];
      if (!std::isnan(q[2]))
        dd.pose.orientation.z = q[2];
      if (!std::isnan(q[3]))
        dd.pose.orientation.w = q[3];
    }
    updateBehaviorState(tracking_targets_[i], param_.use_sukf_, &dd);

    if (tracking_targets_[i].is_stable_ || (tracking_targets_[i].tracking_num_ >= TrackingState::Init &&
                                            tracking_targets_[i].tracking_num_ < TrackingState::Stable))
    {
      tmp_objects.objects.push_back(dd);
      used_targets_indices.push_back(i);
    }
  }
  *detected_objects_output_ptr = removeRedundantObjects(tmp_objects, used_targets_indices);
}

void ImmUkfPdaLanelet2::removeUnnecessaryTarget()
{
  auto result = std::remove_if(tracking_targets_.begin(), tracking_targets_.end(),
                               [](const UKF& t) { return t.tracking_num_ == TrackingState::Die; });  // NOLINT
  tracking_targets_.erase(result, tracking_targets_.end());
}

void ImmUkfPdaLanelet2::dumpResultText(const autoware_msgs::DetectedObjectArray& detected_objects)
{
  std::ofstream outputfile(result_file_path_, std::ofstream::out | std::ofstream::app);
  for (size_t i = 0; i < detected_objects.objects.size(); i++)
  {
    double yaw = tf::getYaw(detected_objects.objects[i].pose.orientation);

    // KITTI tracking benchmark data format:
    // (frame_number,tracked_id, object type, truncation, occlusion, observation angle, x1,y1,x2,y2, h, w, l, cx, cy,
    // cz, yaw)
    // x1, y1, x2, y2 are for 2D bounding box.
    // h, w, l, are for height, width, length respectively
    // cx, cy, cz are for object centroid

    // Tracking benchmark is based on frame_number, tracked_id,
    // bounding box dimentions and object pose(centroid and orientation) from bird-eye view
    outputfile << std::to_string(frame_count_) << " " << std::to_string(detected_objects.objects[i].id) << " "
               << "Unknown"
               << " "
               << "-1"
               << " "
               << "-1"
               << " "
               << "-1"
               << " "
               << "-1 -1 -1 -1"
               << " " << std::to_string(detected_objects.objects[i].dimensions.x) << " "
               << std::to_string(detected_objects.objects[i].dimensions.y) << " "
               << "-1"
               << " " << std::to_string(detected_objects.objects[i].pose.position.x) << " "
               << std::to_string(detected_objects.objects[i].pose.position.y) << " "
               << "-1"
               << " " << std::to_string(yaw) << "\n";
  }
  frame_count_++;
}

void ImmUkfPdaLanelet2::tracker(const autoware_msgs::DetectedObjectArray& input,
                                autoware_msgs::DetectedObjectArray* detected_objects_output_ptr)
{
  const double timestamp = input.header.stamp.toSec();
  std::vector<bool> matching_vec(input.objects.size(), false);

  if (!is_tracker_initialized_)
  {
    initTracker(input, timestamp);
    makeOutput(input, matching_vec, detected_objects_output_ptr);
    return;
  }

  const double dt = (timestamp - last_timestamp_);
  last_timestamp_ = timestamp;

  // start UKF process
  for (size_t i = 0; i < tracking_targets_.size(); i++)
  {
    tracking_targets_[i].is_stable_ = false;
    tracking_targets_[i].is_static_ = false;

    if (tracking_targets_[i].tracking_num_ == TrackingState::Die)
    {
      continue;
    }
    // prevent ukf not to explode
    if (tracking_targets_[i].p_merge_.determinant() > param_.prevent_explosion_threshold_ ||
        tracking_targets_[i].p_merge_(4, 4) > param_.prevent_explosion_threshold_)
    {
      tracking_targets_[i].tracking_num_ = TrackingState::Die;
      continue;
    }

    tracking_targets_[i].prediction(param_.use_sukf_, has_loaded_lanelet_map_, dt);

    std::vector<autoware_msgs::DetectedObject> object_vec;
    const bool success = probabilisticDataAssociation(input, dt, &matching_vec, &object_vec, &tracking_targets_[i]);
    if (!success)
    {
      continue;
    }

    tracking_targets_[i].update(param_.use_sukf_, param_.detection_probability_, param_.gate_probability_,
                                param_.gating_threshold_, object_vec);
  }
  // end UKF process

  // making new ukf target for no data association objects
  makeNewTargets(timestamp, input, matching_vec);

  // static dynamic classification
  staticClassification();

  // making output for visualization
  makeOutput(input, matching_vec, detected_objects_output_ptr);

  // remove unnecessary ukf object
  removeUnnecessaryTarget();
}
