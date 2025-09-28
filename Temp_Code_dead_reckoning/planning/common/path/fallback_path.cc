/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  planning
 */

#include "planning/common/path/fallback_path.h"

#include <utility>
#include <vector>

#include "common/util/point_factory.h"

namespace TL {
namespace planning {

using common::util::PointFactory;

namespace {
constexpr double kPathOptimizationFallbackCost = 2e4;
}

FallBackPath::FallBackPath(const ScenarioConfig::StageConfig& config,
                           const std::shared_ptr<DependencyInjector>& injector)
    : injector_(injector) {
  UNUSED(config);
}

bool FallBackPath::GenerateFallbackPathProfile(
    const Frame* frame, ReferenceLineInfo* const reference_line_info,
    PathData* const path_data) const {
  CHECK(reference_line_info && frame && path_data);

  if (!reference_line_info->path_data().Empty()) {
    ADEBUG << "current path data is not empty.";
    return false;
  }
  if (RetrieveLastFramePathProfile(reference_line_info, frame, path_data)) {
    ADEBUG << "retrieve last frame path successfully.";
    return true;
  }
  GenerateFallbackPathProfile(reference_line_info, path_data,
                              frame->GetMachineStateType());
  reference_line_info->AddCost(kPathOptimizationFallbackCost);
  reference_line_info->set_trajectory_type(ADCTrajectory::PATH_FALLBACK);
  return true;
}

bool FallBackPath::RetrieveLastFramePathProfile(
    const ReferenceLineInfo* reference_line_info, const Frame* frame,
    PathData* const path_data) const {
  const auto* ptr_last_frame = injector_->frame_history()->Latest();
  if (ptr_last_frame == nullptr ||
      ptr_last_frame->DriveReferenceLineInfo() == nullptr ||
      ptr_last_frame->DriveReferenceLineInfo()->IsChangeLanePath()) {
    AERROR << "Last no lane change frame doesn't succeed, fail to retrieve "
              "last frame path data";

    AERROR << "Last frame doesn't succeed, fail to retrieve last frame path "
              "data";
    return false;
  }
  const auto& reference_line = reference_line_info->reference_line();

  const auto& last_frame_discretized_path =
      ptr_last_frame->current_frame_planned_path();
  if (last_frame_discretized_path->empty()) {
    AERROR << "Last no lane change frame discretized path is empty, and fail "
              "to retrieve "
              "last frame path data";
    return false;
  }

  path_data->SetReferenceLine(&reference_line);
  path_data->SetDiscretizedPath(*last_frame_discretized_path);
  const auto adc_frenet_frame_point =
      reference_line_info->reference_line().GetFrenetPoint(
          frame->PlanningStartPoint().path_point());

  bool trim_success = path_data->LeftTrimWithRefS(adc_frenet_frame_point);
  if (!trim_success) {
    AERROR << "Fail to trim path_data. adc_frenet_frame_point: "
           << adc_frenet_frame_point.ShortDebugString();
    return false;
  }
  AERROR << "Use last frame good path to do speed plan.";
  return true;
}

void FallBackPath::GenerateFallbackPathProfile(
    const ReferenceLineInfo* reference_line_info, PathData* const path_data,
    const functionmanager::MachineStateType& type) const {
  const auto& reference_line = reference_line_info->reference_line();
  ADEBUG << "Using init point and reference line to generate fallback path.";
  const auto& adc_point = injector_->ego_info()->start_point();
  DCHECK(adc_point.has_path_point());
  const auto adc_point_x = adc_point.path_point().x();
  const auto adc_point_y = adc_point.path_point().y();

  common::SLPoint adc_point_s_l;
  static constexpr double kUnitS = 1.0;
  const auto unit_step = kUnitS;
  const auto step = static_cast<int>(std::round(kUnitS / unit_step));
  if (!reference_line.XYToSL(adc_point.path_point(), &adc_point_s_l)) {
    AERROR << "Fail to project ADC to reference line when calculating path "
              "fallback. Straight forward path is generated";
    const auto adc_point_heading = adc_point.path_point().theta();
    const auto adc_point_kappa = adc_point.path_point().kappa();
    const auto adc_point_dkappa = adc_point.path_point().dkappa();
    std::vector<common::PathPoint> path_points;
    double adc_traversed_x = adc_point_x;
    double adc_traversed_y = adc_point_y;

    const double sign =
        type == functionmanager::MachineStateType::HISTORY_TRACE_TYPE ? -1.0
                                                                      : 1.0;
    static constexpr double kPathMaxLength = 100.0;

    const auto max_step =
        static_cast<int>(std::round(kPathMaxLength / unit_step));
    for (int i = 0; i < max_step; i += step) {
      const auto s = i * unit_step;
      path_points.emplace_back(PointFactory::ToPathPoint(
          adc_traversed_x, adc_traversed_y, 0.0, s, adc_point_heading,
          adc_point_kappa, adc_point_dkappa));
      adc_traversed_x += sign * kUnitS * std::cos(adc_point_heading);
      adc_traversed_y += sign * kUnitS * std::sin(adc_point_heading);
    }
    path_data->SetReferenceLine(&reference_line);
    path_data->SetDiscretizedPath(DiscretizedPath(std::move(path_points)));
    return;
  }

  // Generate a fallback path along the reference line direction
  const auto adc_s = adc_point_s_l.s();
  const auto& adc_ref_point =
      reference_line.GetReferencePoint(adc_point_x, adc_point_y);
  const double dx = adc_point_x - adc_ref_point.x();
  const double dy = adc_point_y - adc_ref_point.y();
  const auto adc_step_s = static_cast<int>(std::round(adc_s / unit_step));

  std::vector<common::PathPoint> path_points;
  if (type != functionmanager::MachineStateType::HISTORY_TRACE_TYPE) {
    const double max_s = reference_line.Length();
    const auto max_step = static_cast<int>(std::round(max_s / unit_step));
    for (int i = adc_step_s; i < max_step; i += step) {
      const auto s = i * unit_step;
      const auto& ref_point = reference_line.GetReferencePoint(s);
      path_points.emplace_back(PointFactory::ToPathPoint(
          ref_point.x() + dx, ref_point.y() + dy, 0.0, s - adc_s,
          ref_point.heading(), ref_point.kappa(), ref_point.dkappa()));
    }
  } else {
    const double min_s = 0.0;
    const auto min_step = static_cast<int>(std::round(min_s / unit_step));
    for (int i = adc_step_s; i > min_step; i -= step) {
      const auto s = i * unit_step;
      const auto& ref_point = reference_line.GetReferencePoint(s);
      path_points.emplace_back(PointFactory::ToPathPoint(
          ref_point.x() + dx, ref_point.y() + dy, 0.0, adc_s - s,
          ref_point.heading(), ref_point.kappa(), ref_point.dkappa()));
    }
  }

  path_data->SetReferenceLine(&reference_line);
  path_data->SetDiscretizedPath(DiscretizedPath(std::move(path_points)));
}

}  // namespace planning
}  // namespace TL
