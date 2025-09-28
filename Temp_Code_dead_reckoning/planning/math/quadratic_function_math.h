/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file quadratic_function_math.h
 **/

#pragma once
#include <utility>

namespace TL {
namespace planning {
class QuadraticFunctionMath {
 public:
  QuadraticFunctionMath() = delete;
  /**
   * @brief quardratic formula: ax^2 + bx + c = 0, return lower solution
   *        which x is an arbitrary parameter.
   */

  static bool QuardraticFormulaSolution(double a, double b, double c,
                                        std::pair<double, double>* sol);
};

}  // namespace planning
}  // namespace TL
