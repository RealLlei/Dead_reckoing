

//  Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.

#include "dead_reckoning/common/util.h"

namespace apollo {
namespace dead_reckoning {

bool ClampPoint(const Eigen::Vector2d& point, const double& x_min,
                const double& x_max, const double& y_min, const double& y_max) {
  return Clamp(point.x(), x_min, x_max) && Clamp(point.y(), y_min, y_max);
}

double DistancePoint2Linesegment(const Eigen::Vector3d& point,
                                 const Eigen::Vector3d& line_a,
                                 const Eigen::Vector3d& line_b) {
  Eigen::Vector3d p(point.x(), point.y(), 0.0);
  Eigen::Vector3d la(line_a.x(), line_a.y(), 0.0);
  Eigen::Vector3d lb(line_b.x(), line_b.y(), 0.0);
  double d = 0.0;
  auto cal_ang_vec = [](const Eigen::Vector3d& vec_a,
                        const Eigen::Vector3d& vec_b) -> double {
    double cos_theta = vec_a.dot(vec_b) / (vec_a.norm() * vec_b.norm());
    return acos(cos_theta);
  };
  double ang_ab_ap = cal_ang_vec(lb - la, p - la);
  double ang_ba_bp = cal_ang_vec(la - lb, p - lb);
  if (ang_ab_ap > 0.5 * M_PI || ang_ba_bp > 0.5 * M_PI) {
    d = std::min((p - la).norm(), (p - lb).norm());
  } else {
    d = (p - la).norm() * sin(ang_ab_ap);
  }
  return d;
}

double DistanceLineSegments(const Eigen::Vector3d& l1_a,
                            const Eigen::Vector3d& l1_b,
                            const Eigen::Vector3d& l2_a,
                            const Eigen::Vector3d& l2_b) {
  double d[4];
  d[0] = DistancePoint2Linesegment(l1_a, l2_a, l2_b);
  d[1] = DistancePoint2Linesegment(l1_b, l2_a, l2_b);
  d[2] = DistancePoint2Linesegment(l2_a, l1_a, l1_b);
  d[3] = DistancePoint2Linesegment(l2_b, l1_a, l1_b);
  double min_d = *(std::min_element(d, d + 4));
  return min_d;
}

void sort_vertexs_in_counterclockwise(std::vector<Eigen::Vector3d>* vertexs) {
  std::vector<Eigen::Vector3d> temp = *vertexs;
  if (vertexs->empty()) {
    return;
  } else if (vertexs->size() == 4) {
    for (int i = 0; i != 4; i++) {
      vertexs->at(i) = temp[(4 - i) % 4];
    }
  } else if (vertexs->size() == 8) {
    for (int i = 0; i != 8; i++) {
      if (i <= 3) {
        vertexs->at(i) = temp[(4 - i) % 4];
      } else {
        vertexs->at(i) = temp[(4 - i) % 4 + 7];
      }
    }
  }
}

Eigen::Vector3d Quaternion2EulerZyx(const Eigen::Quaterniond& q) {
  Eigen::Vector3d euler;
  euler(0) = atan2(2.0 * (q.y() * q.z() + q.w() * q.x()),
                   1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));
  euler(1) = asin(-2.0 * (q.x() * q.z() - q.w() * q.y()));
  euler(2) = atan2(2.0 * (q.x() * q.y() + q.w() * q.z()),
                   1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
  return euler;
}

Eigen::Vector3d Quaternion2EulerZxy(const Eigen::Quaterniond& q) {
  // Rot = M(Z(euler(2)))*M(X(euler(0)))*M(Y(euler(1)))
  Eigen::Vector3d euler;
  euler(0) = asin(2.0 * (q.y() * q.z() + q.w() * q.x()));
  euler(1) = atan2(2.0 * (q.x() * q.z() - q.w() * q.y()),
                   1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));
  euler(2) = atan2(-2.0 * (q.x() * q.y() - q.w() * q.z()),
                   1.0 - 2.0 * (q.x() * q.x() + q.z() * q.z()));
  return euler;
}

bool PointInConvexpoly(const Eigen::Vector3d& point,
                       const std::vector<Eigen::Vector3d>& points) {
  if (points.size() < 3) {
    return false;
  }
  Eigen::Vector3d tmp = point;
  tmp.z() = 0.0;
  double angle = 0.0;
  std::vector<Eigen::Vector3d> vectors;
  for (auto p : points) {
    p.z() = 0.0;
    vectors.push_back(p - tmp);
  }
  for (int i = 0; i < vectors.size(); i++) {
    int j = (i + 1) % vectors.size();
    angle += acos(vectors[i].normalized().dot(vectors[j].normalized()));
  }
  return (fabs(fabs(angle) - 2 * M_PI) < 0.000001);
}

void ShrinkPoly(std::vector<Eigen::Vector3d>* points, double ratio) {
  if (points->size() < 3) return;

  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  for (auto p : *points) {
    center += p;
  }
  center /= points->size();

  for (int i = 0; i < points->size(); i++) {
    points->at(i).x() = ratio * points->at(i).x() + (1 - ratio) * center.x();
    points->at(i).y() = ratio * points->at(i).y() + (1 - ratio) * center.y();
  }
}

bool FittingLineLs(const std::vector<Eigen::Vector3d>& ref_pts, double* ref_ang,
                   Eigen::VectorXd* err_vec) {
  int dim = ref_pts.size();
  if (dim < 2) {  // give enough points
    return false;
  }

  Eigen::MatrixXd A_l(dim, 2);
  Eigen::Vector2d x_l;
  Eigen::VectorXd b_l(dim);

  // x is dominant, y = k * x + b
  for (size_t i = 0; i < dim; i++) {
    A_l.row(i) << ref_pts[i].x(), 1;
    b_l.row(i) << ref_pts[i].y();
  }
  x_l = A_l.colPivHouseholderQr().solve(b_l);
  *err_vec = b_l - A_l * x_l;
  double error = err_vec->norm() / sqrt(dim - 1);
  *ref_ang = atan2(x_l(0), 1);

  // y is dominant, x = k * y + b
  for (size_t i = 0; i < dim; i++) {
    A_l.row(i) << ref_pts[i].y(), 1;
    b_l.row(i) << ref_pts[i].x();
  }
  x_l = A_l.colPivHouseholderQr().solve(b_l);
  Eigen::VectorXd tmp_err_vec = b_l - A_l * x_l;
  double tmp_error_y = tmp_err_vec.norm() / sqrt(dim - 1);

  if (error > tmp_error_y) {
    err_vec = &tmp_err_vec;
    error = tmp_error_y;
    *ref_ang = atan2(1, x_l(0));
  }

  return true;
}

// bool fitting_line_ls_recur(const std::vector<Eigen::Vector3d> &ref_pts,
// double &ref_ang, double thre, int max_iter_cnt){
//     int dim = ref_pts.size();
//     if (dim < 2) {     // give enough points
//         return false;
//     }

//     Eigen::VectorXd err_vec;
//     FittingLineLs(ref_pts, ref_ang, err_vec);
//     double error = err_vec.norm() / sqrt(dim - 1);

//     if (error < thre || 0 == max_iter_cnt) {
//         return true;
//     } else {
//         std::vector<Eigen::Vector3d> pts;
//         for (size_t i = 0; i < dim; i++) {
//             if (fabs(err_vec(i)) < error) {
//                 pts.push_back(ref_pts[i]);
//             }
//         }
//         if (pts.size() < dim) {
//             max_iter_cnt--;
//             return FittingLineLsRecur(pts, ref_ang, max_iter_cnt);
//         } else {
//             return true;
//         }
//     }
// }

bool FittingLineRansac(const std::vector<Eigen::Vector3d>& ref_pts,
                       double* ref_ang, double thre, int max_iter_cnt) {
  int dim = ref_pts.size();

  if (dim < 2) {  // give enough points
    return false;
  }

  int max_inlier_cnt = 0;
  std::vector<Eigen::Vector3d> max_inliers;

  std::srand(std::time(0));
  for (size_t iter_cnt = 0; iter_cnt < max_iter_cnt; iter_cnt++) {
    std::vector<Eigen::Vector3d> sam_pts;
    sam_pts.resize(2);
    for (size_t i = 0; i < 2; i++) {
      sam_pts[i] =
          ref_pts[static_cast<int>(static_cast<double>(std::rand()) /
                                   static_cast<double>(RAND_MAX) * (dim - 1))];
    }

    // don't calculate when two points are too close
    double sam_pts_dis = (sam_pts[1] - sam_pts[0]).norm();
    if (sam_pts_dis < 0.6) {
      continue;
    }

    // fitting line with two sample points
    double sam_ang = 0.0;
    Eigen::VectorXd err_vec;
    FittingLineLs(sam_pts, &sam_ang, &err_vec);

    // calculate inliers
    int inlier_cnt = 0;
    std::vector<Eigen::Vector3d> inliers;
    Eigen::Vector3d sam_ang_unit_vec(cos(sam_ang), sin(sam_ang), 0.0);
    for (size_t i = 0; i < dim; i++) {
      Eigen::Vector3d pt_vec = ref_pts[i] - sam_pts[0];
      double dis = fabs(sam_ang_unit_vec.cross(pt_vec).z());
      // std::cout << "dis = " << dis << std::endl;
      if (dis < thre) {
        inliers.emplace_back(ref_pts[i]);
        inlier_cnt++;
      }
    }

    // update the line
    if (inlier_cnt > max_inlier_cnt) {
      max_inlier_cnt = inlier_cnt;
      max_inliers = inliers;
    }

    // update the parameters of ransac
    double inlier_ratio =
        static_cast<double>(max_inlier_cnt) / static_cast<double>(dim);
    max_iter_cnt =
        std::min(max_iter_cnt,
                 static_cast<int>(log(0.003) / log(1 - pow(inlier_ratio, 2))));
    // std::cout << "iter_cnt = " << iter_cnt << std::endl;
  }

  // std::cout << "dim = " << dim << ", inliers =" << max_inliers.size() <<
  // std::endl;
  if (max_inliers.size() < static_cast<double>(dim) * 2.0 / 3.0) {
    return false;
  } else {
    // fitting line with inliers
    Eigen::VectorXd err_vec;
    return FittingLineLs(max_inliers, ref_ang, &err_vec);
  }
}

}  // namespace dead_reckoning
}  // namespace apollo
