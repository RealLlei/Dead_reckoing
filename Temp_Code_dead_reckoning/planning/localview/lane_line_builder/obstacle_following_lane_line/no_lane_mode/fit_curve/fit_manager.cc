/******************************************************************************
 * Copyright (c) TL Technologies Co., Ltd. 2019-2022. All rights reserved.
 * Author: Lingpeng
 * Created time: 2022/06/29
 *****************************************************************************/

#include "planning/localview/lane_line_builder/obstacle_following_lane_line/fit_curve/fit_manager.h"

#include <string>
#include <utility>

namespace TL {
namespace planning {
namespace nolane {

FitManager::FitManager(FitType type, double vehicle_speed, int order)
    : coordinate_transform_(CoordinateTransform<2, double>()),
      fit_curve_pimpl_(nullptr),
      distance_piecewise_(
          fmax(vehicle_speed * FLAGS_nolane_trajectory_fit_factor,
               FLAGS_nolane_trajectory_fit_radius)),
      coordinate_fit_(TransCoordSys::NoTranSystem) {
  switch (type) {
    case FitType::PolyNomial:
      fit_curve_pimpl_ = std::make_unique<FitPolynomialCurve>(3);
      break;
    case FitType::Osqp:
      fit_curve_pimpl_ = std::make_unique<FitOsqpCurve>();
      break;
    case FitType::Convoluted_Helix:
      fit_curve_pimpl_ = std::make_unique<FitConvolutedHelix>();
      break;
    default:
      fit_curve_pimpl_ = std::make_unique<FitPolynomialCurve>(3);
      break;
  }
}

bool FitManager::FitPiecewise(
    std::vector<std::pair<common::SLPoint, Vec2d>>* const points,
    std::vector<Vec2d>* const fit_points,
    const std::shared_ptr<ReferenceLine>& reference_line_ptr) {
  if (!fit_curve_pimpl_->FitPiecewise(points, fit_points, reference_line_ptr)) {
    return false;
  }
  // if (typeid(*fit_curve_pimpl_) != typeid(FitPolynomialCurve)) {
  //   return false;
  // }

  const double start_s = points->front().first.s();
  const double end_s = points->back().first.s();
  double spec_s = start_s;
  Vec2d spec_xy;
  using TypePos = std::pair<common::SLPoint, Vec2d>;
  TypePos position_spec;
  common::SLPoint compare_sl;

  std::pair<std::vector<TypePos>::iterator, std::vector<TypePos>::iterator>
      range;

  auto ComparePoint = [](const TypePos& lhs, const TypePos& rhs) {
    return DefinitelyLess(lhs.first.s(), rhs.first.s());
  };

  std::vector<Vec2d> raw_points;
  double base_x = 0;
  double base_y = 0;

  // lp: test fit in 1-order,then rotate the system if the slope > 60.
  auto CoordiFitProcess = [&](const std::vector<Vec2d>& points) {
    coordinate_fit_ =
        static_cast<TransCoordSys>((!fit_curve_pimpl_->TestFit(raw_points)));
    if (coordinate_fit_ == TransCoordSys::NoTranSystem &&
        (fit_curve_pimpl_->GetError() / raw_points.size()) > 15) {
      coordinate_fit_ = TransCoordSys::Error;
    }
    if (coordinate_fit_ == TransCoordSys::NoTranSystem) {
      fit_curve_pimpl_->Fit(raw_points);
    } else {
      CoordinateFit(raw_points);
    }
  };
  // lp: get the fit point of point that projects' point is (s_v,0) at reference
  // line
  auto GetPointAfterTrans = [&](double s_v, Vec2d* const point_o) {
    compare_sl.set_s(s_v);
    compare_sl.set_l(0.0);
    reference_line_ptr->SLToXY(compare_sl, point_o);
    if (static_cast<int>(coordinate_fit_)) {
      point_o->set_x(point_o->x() - base_x);
      point_o->set_y(point_o->y() - base_y);
      RestoreVector(point_o);
      point_o->set_x(point_o->x() + base_x);
      point_o->set_y(point_o->y() + base_y);
    } else {
      point_o->set_x(point_o->x());
      point_o->set_y(
          fit_curve_pimpl_->GenerateSinglePoint(point_o->x() - base_x) +
          base_y);
    }
  };
  if (debug_flag[37]) {
    AERROR << "start_s:" << start_s << "   end_s:" << end_s
           << "  start_3*dis:" << start_s + 3 * distance_piecewise_;
  }
  if (DefinitelyLessEqual(end_s, start_s + 3 * distance_piecewise_)) {
    base_x = points->front().second.x();
    base_y = points->front().second.y();
    raw_points.reserve(points->size());
    for (auto& p : *points) {
      raw_points.push_back({p.second.x() - base_x, p.second.y() - base_y});
    }
    CoordiFitProcess(raw_points);
    while (DefinitelyLess(spec_s, end_s)) {
      GetPointAfterTrans(spec_s, &spec_xy);
      fit_points->push_back(spec_xy);
      spec_s += fit_curve_pimpl_->GetSInterval();
    }
  } else {
    bool start_point_fit = false;
    std::size_t min_size = 3;
    enum class Direction { Left, Right, AlwaysLeft, AlwaysRight };
    Direction direction = Direction::Right;
    while (spec_s < end_s) {
      position_spec.first.set_s(fmax(spec_s - distance_piecewise_, start_s));
      range.first = std::lower_bound(points->begin(), points->end(),
                                     position_spec, ComparePoint);
      position_spec.first.set_s(fmin(spec_s + distance_piecewise_, end_s));
      range.second = std::upper_bound(points->begin(), points->end(),
                                      position_spec, ComparePoint);

      while (std::distance(range.first, range.second) <= min_size) {
        if (direction == Direction::Right) {
          if (range.second != points->end()) {
            ++range.second;
            direction = Direction::Left;
          } else {
            direction = Direction::AlwaysLeft;
          }
        } else if (direction == Direction::Left) {
          if (range.first != points->begin()) {
            --range.first;
            direction = Direction::Right;
          } else {
            direction = Direction::AlwaysRight;
          }
        } else if (direction == Direction::AlwaysLeft) {
          --range.first;
        } else if (direction == Direction::AlwaysRight) {
          ++range.second;
        }
      }
      if (std::distance(range.first, range.second) <= min_size) {
        AERROR << "the distance must be big than min_size:" << min_size
               << "  first:" << std::distance(points->begin(), range.first)
               << "  second:" << std::distance(points->begin(), range.second);
        return false;
      }
      if (range.second != points->end()) {
        raw_points.clear();
        base_x = range.first->second.x();
        base_y = range.first->second.y();
        for (auto it = range.first; it != range.second; ++it) {
          raw_points.push_back(
              {it->second.x() - base_x, it->second.y() - base_y});
        }
        CoordiFitProcess(raw_points);
        if (!start_point_fit) {
          // lp: fit start_s to start_s + distance_piecewise_
          double t_s = start_s;
          while (t_s < spec_s) {
            GetPointAfterTrans(t_s, &spec_xy);
            fit_points->push_back(spec_xy);
            t_s += fit_curve_pimpl_->GetSInterval();
          }
          start_point_fit = true;
        }
        GetPointAfterTrans(spec_s, &spec_xy);
        fit_points->push_back(spec_xy);
      } else if (start_point_fit) {
        if (debug_flag[9]) {
          AERROR << "reach near to end, use prev coefficient.  point_size:"
                 << std::distance(range.first, range.second)
                 << "  spec_s:" << spec_s << "  s_range:["
                 << range.first->first.s() << "," << range.second->first.s()
                 << "]";
        }
        GetPointAfterTrans(spec_s, &spec_xy);
        fit_points->push_back(spec_xy);
      } else {
        AERROR << "spec_s:" << spec_s << "  s_range:[" << range.first->first.s()
               << "," << range.second->first.s() << "]";
        continue;
      }

      if (debug_flag[9]) {
        std::string fit_coef = "";
        TL::common::util::vec2str(fit_curve_pimpl_->GetParameter(),
                                     &fit_coef);
        std::string end_xy = "";
        std::string end_sl = "";
        std::string fit_xy = "";
        if (range.second != points->end()) {
          end_xy = std::to_string(range.second->second.x()) + "," +
                   std::to_string(range.second->second.y());
          end_sl = std::to_string(range.second->first.s()) + "," +
                   std::to_string(range.second->first.l());
        } else {
          end_xy = "end_x,end_y";
          end_sl = "end_s,end_l";
        }
        if (!fit_points->empty()) {
          fit_xy = std::to_string(fit_points->back().x()) + "," +
                   std::to_string(fit_points->back().y());
        } else {
          fit_xy = "fit is empty.";
        }

        AERROR << PRECISION(3) << "base_xy:[" << base_x << "," << base_y
               << "]  sl_range:[" << range.first->first.s() << ","
               << range.first->first.l() << "," << end_sl << "]  xy_range:["
               << range.first->second.x() << "," << range.first->second.y()
               << "," << end_xy << "]  fit_coef:" << fit_coef
               << "  error:" << fit_curve_pimpl_->GetError()
               << "  coordinate_fit:" << static_cast<int>(coordinate_fit_)
               << "  spec_s:" << spec_s << "  spec_xy:[" << spec_xy.x() << ","
               << spec_xy.y() << "]  fit_point_xy:[" << fit_xy << "]";
        AERROR << "";
      }
      spec_s += fit_curve_pimpl_->GetSInterval();
    }
  }
  if (debug_flag[13]) {
    int counter = 0;
    for (const auto& val : *fit_points) {
      AERROR << "counter:" << counter++ << PRECISION(3)
             << "   piecewise_fit_x:" << val.x()
             << "  piecewise_fit_y:" << val.y();
    }
  }
  return true;
}

void FitManager::CoordinateFit(const std::vector<Vec2d>& points) {
  double degree = atan(fit_curve_pimpl_->GetParameter().back());
  Eigen::Matrix<double, 2, 2> base_vector;
  base_vector(0, 0) = 0;   // cos(degree);
  base_vector(1, 0) = 1;   // sin(degree);
  base_vector(0, 1) = -1;  // -sin(degree);
  base_vector(1, 1) = 0;   // cos(degree);
  if (debug_flag[36]) {
    AERROR << "degree:" << degree * 180 / M_PI << "   base_vector:"
           << "[" << base_vector(0, 0) << "," << base_vector(1, 0) << "],["
           << base_vector(0, 1) << "," << base_vector(1, 1) << "]";
  }
  coordinate_transform_.SetNewBaseVector(base_vector);
  std::vector<Vec2d> points_trans;
  points_trans.reserve(points.size());
  std::vector<double> point_original(2);
  std::vector<double> point_trans(2);
  for (const auto& p : points) {
    point_original.clear();
    point_original.push_back(p.x());
    point_original.push_back(p.y());
    coordinate_transform_.Origin2NewBaseRotate(point_original, &point_trans);
    points_trans.push_back({point_trans[0], point_trans[1]});
  }
  fit_curve_pimpl_->Fit(points_trans);
}

void FitManager::RestoreVector(const Vec2d& origin_point_in,
                               Vec2d* const origin_point_out) {
  std::vector<double> origin_p(2);
  std::vector<double> new_base_p(2);
  coordinate_transform_.Origin2NewBaseRotate(
      {origin_point_in.x(), origin_point_in.y()}, &new_base_p);
  new_base_p.at(1) = fit_curve_pimpl_->GenerateSinglePoint(new_base_p.at(0));
  coordinate_transform_.NewBase2OriginRotate(new_base_p, &origin_p);
  origin_point_out->set_x(origin_p.at(0));
  origin_point_out->set_y(origin_p.at(1));
}

void FitManager::RestoreVector(Vec2d* const point) {
  Vec2d point_in = *point;
  RestoreVector(point_in, point);
  if (debug_flag[35]) {
    double x = point_in.x();
    double y = point_in.y();
    AERROR << "orgin->point [" << x << "," << y << "] -> [" << point->x() << ","
           << point->y() << "]";
  }
}

void FitManager::AdjustDistancePiecewise() {}

}  // namespace nolane
}  // namespace planning
}  // namespace TL
