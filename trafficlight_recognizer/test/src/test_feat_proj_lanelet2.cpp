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

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <vector>

#include "./test_feat_proj_lanelet2.h"

namespace trafficlight_recognizer
{
class FeatProjLanelet2TestSuite : public ::testing::Test
{
public:
  FeatProjLanelet2TestSuite()
  {
  }
  ~FeatProjLanelet2TestSuite()
  {
  }

  FeatProjLanelet2TestClass test_obj;

protected:
  virtual void SetUp()
  {
    test_obj.fpll2 = new FeatProjLanelet2();
  }
  virtual void TearDown()
  {
    delete test_obj.fpll2;
  }
};

TEST_F(FeatProjLanelet2TestSuite, test_inRange)
{
  lanelet::BasicPoint2d p(0, 0);         // p.x = 0; p.y = 0;
  lanelet::BasicPoint2d near_cam(9, 0);  // cam.x = 11.0; cam.y = 0.0;
  lanelet::BasicPoint2d far_cam(11, 0);  // cam.x = 11.0; cam.y = 0.0;
  EXPECT_EQ(true, test_obj.inRange(p, near_cam, 10.0)) << "inRange returned false when should be true";
  EXPECT_EQ(false, test_obj.inRange(p, far_cam, 10.0)) << "inRange returned true when should be false";
}

TEST_F(FeatProjLanelet2TestSuite, test_transform)
{
  Eigen::Vector3f v, v_tf, v_gt;
  tf::StampedTransform stf;
  tf::Transform tf = tf::Transform::getIdentity();

  stf.setData(tf);
  v << 1, -1, 0;
  v_tf = test_obj.transform(v, stf);
  v_gt = v;
  EXPECT_EQ(v_gt, v_tf) << "vector transformed by identity is not equal";

  tf.setRotation(tf::createQuaternionFromRPY(0, 0, M_PI / 2));
  stf.setData(tf);
  v << 1.000, 0.000, 0.000;
  v_tf = test_obj.transform(v, stf);
  v_gt << 0.000, 1.000, 0.000;

  // use isApprox member function of Eigen to check when rounding errors may occur
  EXPECT_TRUE(v_gt.isApprox(v_tf)) << "vector rotated by 90 around yaw \n\t v_gt = " << v_gt.transpose()
                                   << "\n\t v_tf = " << v_tf.transpose() << "\n";
}

TEST_F(FeatProjLanelet2TestSuite, test_project2)
{
  Eigen::Vector3f pt_front, pt_behind, pt_left, pt_up, pt_far_left, pt_far_up;
  tf::StampedTransform stf;
  tf::Transform tf = tf::Transform::getIdentity();

  // +z is depth from camera
  stf.setData(tf);
  pt_front << 0, 0, 10;
  pt_behind << 0, 0, -10;
  pt_left << -10, 0, 10;
  pt_far_left << -25, 0, 10;
  pt_up << 0, 10, 10;
  pt_far_up << 0, 25, 10;
  int u, v;
  bool in_image;

  test_obj.setCameraInfo(200, 200, 100, 100, 400, 400);
  test_obj.setCameraToMapTransform(stf);

  in_image = test_obj.project2(pt_front, &u, &v);
  EXPECT_EQ(true, in_image) << "pt not in view when directly in front " << pt_front.transpose() << " u = " << u
                            << " v = " << v;

  in_image = test_obj.project2(pt_behind, &u, &v);
  EXPECT_EQ(false, in_image) << "pt in view when should be behind" << pt_behind.transpose() << " u = " << u
                             << " v = " << v;

  in_image = test_obj.project2(pt_left, &u, &v);
  EXPECT_EQ(true, in_image) << "pt in view when front to wide left " << pt_left.transpose() << " u = " << u
                            << " v = " << v;

  in_image = test_obj.project2(pt_up, &u, &v);
  EXPECT_EQ(true, in_image) << "pt not in view when should front up " << pt_up.transpose() << " u = " << u
                            << " v = " << v;

  in_image = test_obj.project2(pt_far_left, &u, &v);
  EXPECT_EQ(false, in_image) << "pt in view when front to wide left " << pt_far_left.transpose() << " u = " << u
                             << " v = " << v;

  in_image = test_obj.project2(pt_far_up, &u, &v);
  EXPECT_EQ(false, in_image) << "pt in view when should front up high" << pt_far_up.transpose() << " u = " << u
                             << " v = " << v;
}

TEST_F(FeatProjLanelet2TestSuite, test_inView)
{
  lanelet::BasicPoint2d p(10, 0);
  lanelet::BasicPoint2d cam(0, 0);
  double heading = 0;
  double max_a = M_PI / 4.0;
  double max_r = 50.0;

  EXPECT_EQ(true, test_obj.inView(p, cam, heading, max_a, max_r)) << "in view should return true for point in front";

  p.x() = -10;

  EXPECT_EQ(false, test_obj.inView(p, cam, heading, max_a, max_r)) << "in view should return false for point in behind";

  // boundary at edge of angle bound
  p.x() = 10;
  p.y() = 9;
  EXPECT_EQ(true, test_obj.inView(p, cam, heading, max_a, max_r)) << "in view should return true for point in front "
                                                                     "left";
  p.x() = 10;
  p.y() = 11;
  EXPECT_EQ(false, test_obj.inView(p, cam, heading, max_a, max_r)) << "in view should return false for point in front "
                                                                      "left\n";
  p.x() = 10;
  p.y() = -9;
  EXPECT_EQ(true, test_obj.inView(p, cam, heading, max_a, max_r)) << "in view should return true for point in front "
                                                                     "right";
  p.x() = 10;
  p.y() = -11;
  EXPECT_EQ(false, test_obj.inView(p, cam, heading, max_a, max_r)) << "in view should return false for point in front "
                                                                      "right\n";
}

TEST_F(FeatProjLanelet2TestSuite, test_trafficLightVisibilityCheck)
{
  std::vector<lanelet::TrafficLight::Ptr> tl;
  std::vector<lanelet::autoware::AutowareTrafficLight::Ptr> atl;
  std::vector<lanelet::TrafficLight::Ptr> vis_tl;
  std::vector<lanelet::autoware::AutowareTrafficLight::Ptr> vis_at;

  // construct tl objects for test

  EXPECT_EQ(true, true) << "dummy test";
}

}  // namespace trafficlight_recognizer

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "FeatProjLanelet2TestNode");
  ros::NodeHandle rosnode;
  return RUN_ALL_TESTS();
}
