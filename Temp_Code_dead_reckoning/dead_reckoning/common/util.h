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

#pragma once
#include <Eigen/Core>
#include <math.h>
#include <algorithm>
#include <vector>
#include "dead_reckoning/common/structs.h"

namespace apollo {
namespace dead_reckoning {

inline double Rad2Deg(const double& rad) { return rad / M_PI * 180.0; }

inline double Deg2Rad(const double& deg) { return deg * M_PI / 180.0; }

template <typename T>
inline T Sign(T x) {
  return x > 0 ? 1 : -1;
}

template <typename T>
inline bool Clamp(const T& v, const T& lo, const T& hi) {
  return v >= lo && v <= hi;
}

bool ClampPoint(const Eigen::Vector2d& point, const double& x_min,
                const double& x_max, const double& y_min, const double& y_max);

double DistancePoint2Linesegment(const Eigen::Vector3d& p,
                                 const Eigen::Vector3d& la,
                                 const Eigen::Vector3d& lb);

double DistanceLineSegments(const Eigen::Vector3d& l1_a,
                            const Eigen::Vector3d& l1_b,
                            const Eigen::Vector3d& l2_a,
                            const Eigen::Vector3d& l2_b);

inline double GaussFun(double x, double mean, double sgm) {
  return (1.0 / (sgm * sqrt(2 * M_PI))) *
         exp(-0.5 * pow((x - mean) / sgm, 2.0));
}

void SortVertexsInCounterClockwise(std::vector<Eigen::Vector3d>* vertexs);

Eigen::Vector3d Quaternion2EulerZyx(const Eigen::Quaterniond& q);
Eigen::Vector3d Quaternion2EulerZxy(const Eigen::Quaterniond& q);

template <typename sensor_data_type>
class SensorDataStruct {
 public:
  sensor_data_type _data;
  sensor_data_type _last_data;
  void DataUpdate(const sensor_data_type& cur_data) {
    if (!_is_valid) {
      _data = _last_data = cur_data;
      _is_valid = true;
    }
    _is_update = ((cur_data.timestamp - _data.timestamp) > 1.0e-3);
    // DLOG(INFO) << "cur_data.timestamp: " << cur_data.timestamp
    //           << ", _data.timestamp: " << _data.timestamp
    //           << "_is_update: " << _is_update;
    if (_is_update) {
      _last_data = _data;
      _data = cur_data;
    }
  }
  bool _is_update = false;
  bool _is_valid = false;
};

bool PointInConvexpoly(const Eigen::Vector3d& point,
                       const std::vector<Eigen::Vector3d>& points);
void ShrinkPoly(std::vector<Eigen::Vector3d>* points, double ratio);

bool FittingLineLs(const std::vector<Eigen::Vector3d>& ref_pts, double* ref_ang,
                   Eigen::VectorXd* err_vec);
// bool FittingLineLsRecur(const std::vector<Eigen::Vector3d>& ref_pts,
//                            double& ref_ang, double thre = 0.03,
//                            int max_iter_cnt = -1);
bool FittingLineRansac(const std::vector<Eigen::Vector3d>& ref_pts,
                       double* ref_ang, double thre = 0.1,
                       int max_iter_cnt = 20);

}  // namespace dead_reckoning
}  // namespace apollo
