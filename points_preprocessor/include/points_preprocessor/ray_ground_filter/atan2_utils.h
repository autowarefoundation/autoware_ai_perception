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
 */
/// \file
/// \brief This file provides a fast but approximate atan2 function
#ifndef POINTS_PREPROCESSOR_RAY_GROUND_FILTER__ATAN2_UTILS_H
#define POINTS_PREPROCESSOR_RAY_GROUND_FILTER__ATAN2_UTILS_H

#include <cmath>

/// pi
constexpr float PI = 3.14159265359F;
/// pi/2
constexpr float PI_2 = 1.5707963267948966F;

///
/// Approximation was taken from:
/// http://www-labs.iro.umontreal.ca/~mignotte/IFT2425/Documents/EfficientApproximationArctgFunction.pdf
///
/// |Error = fast_atan2(y, x) - atan2f(y, x)| < 0.00468 rad
///
/// Octants:
///         pi/2
///       ` 3 | 2 /
///        `  |  /
///       4 ` | / 1
///   pi -----+----- 0
///       5 / | ` 8
///        /  |  `
///       / 6 | 7 `
///         3pi/2
///
///
float fast_atan2(float y, float x)
{
  constexpr float scaling_constant = 0.28086f;

  if (x == 0.0f) {
    // Special case atan2(0.0, 0.0) = 0.0
    if (y == 0.0f) {
      return 0.0f;
    }

    // x is zero so we are either at pi/2 for (y > 0) or -pi/2 for (y < 0)
    return ::std::copysign(PI_2, y);
  }

  // Calculate quotient of y and x
  float div = y / x;

  // Determine in which octants we can be, if |y| is smaller than |x| (|div|<1)
  // then we are either in 1,4,5 or 8 else we are in 2,3,6 or 7.
  if (fabsf(div) < 1.0f) {
    // We are in 1,4,5 or 8

    float atan = div / (1.0f + scaling_constant * div * div);

    // If we are in 4 or 5 we need to add pi or -pi respectively
    if (x < 0.0f) {
      return ::std::copysign(PI, y) + atan;
    }
    return atan;
  }

  // We are in 2,3,6 or 7
  return ::std::copysign(PI_2, y) - div / (div * div + scaling_constant);
}



#endif  // POINTS_PREPROCESSOR_RAY_GROUND_FILTER__ATAN2_UTILS_H
