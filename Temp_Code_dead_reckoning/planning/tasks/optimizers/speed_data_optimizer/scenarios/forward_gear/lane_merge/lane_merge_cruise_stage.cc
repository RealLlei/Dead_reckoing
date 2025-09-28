/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file lane_merge_cruise_stage.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/forward_gear/lane_merge/lane_merge_cruise_stage.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>

#include "common/file/log.h"

#include "planning/proto/speed_evaluator_config.pb.h"

namespace TL {
namespace planning {

LaneMergeCruiseStage::LaneMergeCruiseStage(const SpeedStageConfig& config)
    : SpeedStage(config), config_(config.lane_merge_cruise_stage_config()) {}

bool LaneMergeCruiseStage::PreProcess(Frame* frame,
                                      ReferenceLineInfo* reference_line_info) {
  if (frame == nullptr || reference_line_info == nullptr) {
    return false;
  }

  GetSpeedEvaluator()->SetMergeStop(false);

  return true;
}

bool LaneMergeCruiseStage::FinishStage(
    Frame* frame, ReferenceLineInfo* reference_line_info,
    const common::TrajectoryPoint& init_point, const SpeedCache& cache,
    const std::shared_ptr<SpeedDataGenerator>& generator,
    SpeedData* speed_data) {
  const auto forward_gear_generator =
      std::dynamic_pointer_cast<ForwardGearSpeedDataGenerator>(generator);
  if (forward_gear_generator == nullptr) {
    return false;
  }

  if (frame == nullptr || reference_line_info == nullptr ||
      speed_data->empty() /**|| speed_data->GetIsFallback()*/ ||
      CheckStartMergeStop(frame, reference_line_info, init_point, cache,
                          forward_gear_generator)) {
    GetSpeedEvaluator()->SetMergeStop(true);
    forward_gear_generator->GeneratorMergeStopSpeedData(
        frame, reference_line_info, init_point, cache, GetSpeedEvaluator(),
        speed_data);
    return true;
  }
  return false;
}

bool LaneMergeCruiseStage::CheckStartMergeStop(
    Frame* const frame, ReferenceLineInfo* const reference_line_info,
    const common::TrajectoryPoint& init_point, const SpeedCache& cache,
    const std::shared_ptr<ForwardGearSpeedDataGenerator>& generator) {
  if (frame == nullptr || reference_line_info == nullptr ||
      generator == nullptr) {
    return true;
  }
  const auto& best_curve = generator->GetBestCurve();
  if (best_curve == nullptr) {
    return true;
  }

  if (cache.GetBasicCache().GetHighRoadRightEndS() >
      cache.GetBasicCache().GetExpectedStopS()) {
    return false;
  }

  static constexpr double kMergePreviewDistance = 20.0;
  static constexpr double kLBuffer = 0.1;
  static constexpr double kStartDecAcc = -1.0;
  static constexpr double kMinUpperSafeDistance = 1.0;
  static constexpr double kMinLowerSafeDistance = 0.5;
  static constexpr double kSSafeDistanceTime = 0.25;
  const auto merge_s = cache.GetBasicCache().GetHighRoadRightEndS();
  const auto dec = -pow(init_point.v(), 2) / (2.0 * std::fmax(merge_s, 0.1));
  const auto point_count = std::max(best_curve->GetCurveDensePointCount(),
                                    best_curve->GetMinDensePointCount());
  const auto dense_points = best_curve->GetDensePoints();
  if (dec > kStartDecAcc && !generator->StartMergeStop() &&
      merge_s > kMergePreviewDistance) {
    generator->SetStartMergeStop(false);
    return false;
  }
  ADEBUG << "cruise merge_s : " << merge_s;
  ADEBUG << " ends  : " << best_curve->GetEndS()
         << "endv  :" << best_curve->GetEndV();
  generator->SetStartMergeStop(true);
  const auto& vehicle_param_ =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  const auto adc_l_lower = -vehicle_param_.width() / 2.0 - kLBuffer;
  const auto adc_l_upper = -adc_l_lower;
  for (std::size_t i = 0; i < point_count; ++i) {
    const auto& point = dense_points.at(i);
    const auto t = point.t();
    const auto s = point.s();
    if (s < merge_s - kMergePreviewDistance) {
      continue;
    }
    ADEBUG << "t : " << t << "s :" << s << "v :" << point.v()
           << " a :" << point.a();
    const auto s_buffer = point.v() * kSSafeDistanceTime;
    const auto adc_s_upper = s + vehicle_param_.front_edge_to_center() +
                             kMinUpperSafeDistance + s_buffer;
    const auto adc_s_lower = s - vehicle_param_.back_edge_to_center() -
                             kMinLowerSafeDistance - s_buffer;
    ADEBUG << " adc_s_lower :" << adc_s_lower << " adc_s_upper :" << adc_s_upper
           << " adc_l_lower :" << adc_l_lower
           << " adc_l_upper :" << adc_l_upper;
    for (const auto* slt_obstacle_cache :
         cache.GetSafeSLTObstacleCachesWithOutDecision()) {
      if (slt_obstacle_cache == nullptr ||
          (slt_obstacle_cache->GetObstacle()->PerceptionSLBoundary().end_s() <
               reference_line_info->AdcSlBoundary().start_s() &&
           slt_obstacle_cache->GetObstacle()->IsStatic())) {
        continue;
      }

      // check if obstacle is in st graph at time t
      if (t < slt_obstacle_cache->GetMinT() ||
          t > slt_obstacle_cache->GetMaxT()) {
        continue;
      }
      // calucalate s_upper, s_lower, obstacle_v
      const auto& obstacle_info = slt_obstacle_cache->GetObstacleInfoAtTime(t);
      ADEBUG << " obs id :" << slt_obstacle_cache->GetId()
             << " obstacle_info.s_lower :" << obstacle_info.s_lower
             << " obstacle_info.s_upper :" << obstacle_info.s_upper
             << " v :" << obstacle_info.ds
             << " obstacle_info.l_lower :" << obstacle_info.l_lower
             << " obstacle_info.l_upper :" << obstacle_info.l_upper;
      if ((adc_s_lower <= obstacle_info.s_upper &&
           adc_s_upper >= obstacle_info.s_lower) &&
          (adc_l_lower <= obstacle_info.l_upper &&
           adc_l_upper >= obstacle_info.l_lower)) {
        ADEBUG << "!!!!!!DUANG";
        // 有碰撞
        return true;
      }
    }
  }
  return false;
}
}  // namespace planning
}  // namespace TL
