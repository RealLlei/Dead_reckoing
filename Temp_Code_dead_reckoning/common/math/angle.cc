/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "common/math/angle.h"

#include "common/math/atan_table.h"
#include "common/math/sin_table.h"

//NOLINTBEGIN
namespace TL {
namespace common {
namespace math {

float sin(Angle16 a) {
  int16_t idx = a.raw();

  if (idx < -Angle16::RAW_PI_2) {
    idx = static_cast<int16_t>(idx + Angle16::RAW_PI);
    return -SIN_TABLE[idx % SIN_TABLE_SIZE];
  }
  if (idx < 0) {
    return -SIN_TABLE[(-idx) % SIN_TABLE_SIZE];
  }
  if (idx < Angle16::RAW_PI_2) {
    return SIN_TABLE[idx % SIN_TABLE_SIZE];
  }
  idx = static_cast<int16_t>(Angle16::RAW_PI - idx);
  return SIN_TABLE[idx % SIN_TABLE_SIZE];
}

float sin(double rad) {
  int16_t idx = static_cast<int16_t>(std::lround(rad * Angle16::RAD_TO_RAW));
  if (idx < -Angle16::RAW_PI_2) {
    idx = static_cast<int16_t>(idx + Angle16::RAW_PI);
    return -SIN_TABLE[idx % SIN_TABLE_SIZE];
  }
  if (idx < 0) {
    return -SIN_TABLE[(-idx) % SIN_TABLE_SIZE];
  }
  if (idx < Angle16::RAW_PI_2) {
    return SIN_TABLE[idx % SIN_TABLE_SIZE];
  }
  idx = static_cast<int16_t>(Angle16::RAW_PI - idx);
  return SIN_TABLE[idx % SIN_TABLE_SIZE];
}

float cos(Angle16 a) {
  Angle16 b(static_cast<int16_t>(Angle16::RAW_PI_2 - a.raw()));
  return sin(b);
}

float cos(double rad) {
  int16_t idx = Angle16::RAW_PI_2 -
                static_cast<int16_t>(std::lround(rad * Angle16::RAD_TO_RAW));
  if (idx < -Angle16::RAW_PI_2) {
    idx = static_cast<int16_t>(idx + Angle16::RAW_PI);
    return -SIN_TABLE[idx % SIN_TABLE_SIZE];
  }
  if (idx < 0) {
    return -SIN_TABLE[(-idx) % SIN_TABLE_SIZE];
  }
  if (idx < Angle16::RAW_PI_2) {
    return SIN_TABLE[idx % SIN_TABLE_SIZE];
  }
  idx = static_cast<int16_t>(Angle16::RAW_PI - idx);
  return SIN_TABLE[idx % SIN_TABLE_SIZE];
}

float tan(Angle16 a) {
  return sin(a) / cos(a);
}

float sin(Angle8 a) {
  Angle16 b(static_cast<int16_t>(a.raw() << 8));
  return sin(b);
}

float cos(Angle8 a) {
  Angle16 b(static_cast<int16_t>(a.raw() << 8));
  return cos(b);
}

float tan(Angle8 a) {
  Angle16 b(static_cast<int16_t>(a.raw() << 8));
  return tan(b);
}

float atan2(float y, float x) {
  float Y_X, Y_X_temp;
  float result = 0;

  Y_X = (x <= 1e-6 && x >= -1e-6)
            ? static_cast<float>(x) / static_cast<float>(y)
            : static_cast<float>(y) / static_cast<float>(x);
  Y_X_temp = Y_X;

  if (y <= 1e-6 && y >= -1e-6) {
    if (x >= 1e-6)
      return 1e-6;
    else
      return (M_PI);  // Return Pi
  } else if (x > 1e-6 && y > 1e-6) {
    Y_X = (Y_X <= 1) ? Y_X : 1 / Y_X;
    result =
        (Y_X_temp > 1)
            ? M_PI_2 -
                  g_atan_tab[static_cast<int16_t>(Y_X * (ATAN2_TABLE_SIZE - 1))]
            : g_atan_tab[static_cast<int16_t>(Y_X * (ATAN2_TABLE_SIZE - 1))];
  } else if (x < -1e-6 && y > 1e-6) {
    Y_X = (Y_X < -1) ? Y_X = -1 / Y_X : -Y_X;
    result =
        (Y_X_temp >= -1)
            ? M_PI -
                  g_atan_tab[static_cast<int16_t>(Y_X * (ATAN2_TABLE_SIZE - 1))]
            : M_PI_2 + g_atan_tab[static_cast<int16_t>(Y_X *
                                                       (ATAN2_TABLE_SIZE - 1))];
  } else if (x < -1e-6 && y < -1e-6) {
    Y_X = (Y_X <= 1) ? Y_X : 1 / Y_X;
    result =
        (Y_X_temp <= 1)
            ? g_atan_tab[static_cast<int16_t>(Y_X * (ATAN2_TABLE_SIZE - 1))] -
                  M_PI
            : -M_PI_2 - g_atan_tab[static_cast<int16_t>(
                            Y_X * (ATAN2_TABLE_SIZE - 1))];
  } else if (x > 1e-6 && y < -1e-6) {
    Y_X = (Y_X < -1) ? Y_X = -1 / Y_X : -Y_X;
    result =
        (Y_X_temp >= -1)
            ? -g_atan_tab[static_cast<int16_t>(Y_X * (ATAN2_TABLE_SIZE - 1))]
            : -(M_PI_2 -
                g_atan_tab[static_cast<int16_t>(Y_X * (ATAN2_TABLE_SIZE - 1))]);
  } else if ((x <= 1e-6 && x >= -1e-6) && y > 1e-6) {
    result = M_PI_2;
  } else {
    result = -M_PI_2;
  }

  return result;
}

//NOLINTEND

}  // namespace math
}  // namespace common
}  // namespace TL
