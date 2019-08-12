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

#include <iostream>

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <autoware_msgs/VehicleStatus.h>
#include "twist_generator/vehicle_status_converter.hpp"

class VehicleStatusConverterTestSuite : public ::testing::Test
{
public:
    VehicleStatusConverterTestSuite() {}
    ~VehicleStatusConverterTestSuite() {}
protected: 
    virtual void SetUp() {
       obj_ = new VehicleStatusConverter();
    };
    virtual void TearDown() {
       delete obj_;
    };
    void update(const double& w_ndt, const double& vel, const double& steer){
       obj_->updateSteeringOffset(w_ndt, vel, steer);
    };
    double getCoefficientVx(){
       return obj_->adaptive_coefficient_vx_;
    };
    double getCoefficientWz(){
       return obj_->adaptive_coefficient_wz_;
    };
    double getSteeringOffset(){
       return obj_->steering_offset_;
    };
    double getWheelBase(){
       return obj_->wheelbase_;
    }
    VehicleStatusConverter* obj_;

};

TEST_F(VehicleStatusConverterTestSuite, updateSteeringOffset)
{
    double vel = 3.0;
    double steer = amathutils::deg2rad(0.0);
    double w_ndt = vel * std::tan(steer + amathutils::deg2rad(1.0)) / getWheelBase();
    for (int i = 0; i < 10000 ; i++){
        update(w_ndt, vel, steer);
    }
    EXPECT_NEAR(getSteeringOffset(), amathutils::deg2rad(1.0), amathutils::deg2rad(1.0) / 100) << "steering offset is approximately " << amathutils::deg2rad(1.0) << " " << getSteeringOffset();
}
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "VehicleStatusConverterTestSuite");

    return RUN_ALL_TESTS();
}