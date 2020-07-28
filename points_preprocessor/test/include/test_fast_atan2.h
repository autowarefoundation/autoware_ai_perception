/*
 * Copyright 2020 Autoware Foundation. All rights reserved.
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

#include "points_preprocessor/ray_ground_filter/atan2_utils.h"

TEST(RayGroundFilter, fast_atan2)
{
  double nb_of_sample = 100;
  double epsilon_rad = 1.0E-5F; // to avoid the non defined value of tan/atan
  double max_error_expected = 0.005;  // radian

  double step_size = PI/nb_of_sample;
  for( double angle = -PI_2+epsilon_rad; angle < PI_2-epsilon_rad; angle += step_size)
  {
      double y_value = tan(angle);
      double reference_value = atan2(y_value, 1);
      double tested_value = fast_atan2(y_value, 1);
      ASSERT_LT(fabsf(reference_value-tested_value), max_error_expected) \
          << "atan2 approximation's error is above the expected error : " \
          << fabsf(reference_value-tested_value);
      reference_value = atan2(y_value, -1);
      tested_value = fast_atan2(y_value, -1);
      ASSERT_LT(fabsf(reference_value-tested_value), max_error_expected) \
          << "atan2 approximation's error is above the expected error : " \
          << fabsf(reference_value-tested_value);
  }
}
