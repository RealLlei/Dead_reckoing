/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file lane_merge_stop_stage.cc
 **/

#include "planning/tasks/optimizers/speed_data_optimizer/scenarios/forward_gear/lane_merge/lane_merge_stop_stage.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>

#include "common/file/log.h"
#include "planning/tasks/optimizers/speed_data_optimizer/evaluators/speed_evaluator_factory.h"

#include "planning/proto/speed_evaluator_config.pb.h"

namespace TL {
namespace planning {

LaneMergeStopStage::LaneMergeStopStage(const SpeedStageConfig& config)
    : SpeedStage(config), config_(config.lane_merge_stop_stage_config()) {}

bool LaneMergeStopStage::PreProcess(Frame* frame,
                                    ReferenceLineInfo* reference_line_info) {
  if (frame == nullptr || reference_line_info == nullptr) {
    return false;
  }

  GetSpeedEvaluator()->SetMergeStop(false);
  return true;
}

bool LaneMergeStopStage::PostProcess(
    Frame* frame, ReferenceLineInfo* reference_line_info,
    const common::TrajectoryPoint& init_point, const SpeedCache& cache,
    const std::shared_ptr<SpeedDataGenerator>& generator,
    SpeedData* speed_data) {
  UNUSED(frame);
  UNUSED(reference_line_info);
  UNUSED(init_point);
  UNUSED(cache);
  UNUSED(generator);
  UNUSED(speed_data);
  return true;
}

bool LaneMergeStopStage::FinishStage(
    Frame* frame, ReferenceLineInfo* reference_line_info,
    const common::TrajectoryPoint& init_point, const SpeedCache& cache,
    const std::shared_ptr<SpeedDataGenerator>& generator,
    SpeedData* const speed_data) {
  const auto* last_frame = cache.GetBasicCache().GetLastFrame();
  if (speed_data == nullptr || speed_data->empty() || last_frame == nullptr) {
    return false;
  }

  const auto forward_gear_generator =
      std::dynamic_pointer_cast<ForwardGearSpeedDataGenerator>(generator);
  if (forward_gear_generator == nullptr) {
    return false;
  }

  if (CheckEndMergeStop(frame, reference_line_info, init_point, cache,
                        forward_gear_generator)) {
    frame->SetIsMergeStopFallback(false);
    return true;
  }

  if (cache.GetBasicCache().GetHighRoadRightEndS() >
      cache.GetBasicCache().GetExpectedStopS()) {
    return false;
  }

  GetSpeedEvaluator()->SetMergeStop(true);
  forward_gear_generator->GeneratorMergeStopSpeedData(
      frame, reference_line_info, init_point, cache, GetSpeedEvaluator(),
      speed_data);
  if (last_frame->GetIsMergeStopFallback() ||
      TL::common::math::double_type::DefinitelyLessEqual(
          cache.GetBasicCache().GetHighRoadRightEndS(),
          kMergeStopFallbackSMin) ||
      (TL::common::math::double_type::DefinitelyLessEqual(
           cache.GetBasicCache().GetHighRoadRightEndS(),
           kMergeStopFallbackSMax) &&
       TL::common::math::double_type::DefinitelyLessEqual(
           init_point.a(), kMergeStopFallbackA))) {
    frame->SetIsMergeStopFallback(true);
  }

  return false;
}

bool LaneMergeStopStage::CheckEndMergeStop(
    Frame* const frame, ReferenceLineInfo* const reference_line_info,
    const common::TrajectoryPoint& init_point, const SpeedCache& cache,
    const std::shared_ptr<ForwardGearSpeedDataGenerator>& generator) {
  UNUSED(init_point);

  if (frame == nullptr || reference_line_info == nullptr ||
      generator == nullptr) {
    return false;
  }
  const auto& best_curve = generator->GetBestCurve();
  if (best_curve == nullptr) {
    generator->SetStartMergeStop(true);
    return false;
  }

  ADEBUG << "stopppppppppppppppp  merge_s "
         << cache.GetBasicCache().GetHighRoadRightEndS()
         << "mode :" << static_cast<int>(best_curve->GetTarget().mode)
         << " GetSLength :" << best_curve->GetSLength()
         << " GetMinV :" << best_curve->GetMinV()
         << " GetEndS :" << best_curve->GetEndS()
         << " GetEndV :" << best_curve->GetEndV();
  static constexpr double kMergePreviewDistance = 10;
  static constexpr double kLBuffer = 0.2;
  static constexpr double kMinSafeDistance = 2.0;
  const auto point_count = std::max(best_curve->GetCurveDensePointCount(),
                                    best_curve->GetMinDensePointCount());
  const auto dense_points = best_curve->GetDensePoints();
  const auto& vehicle_param_ =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  const auto adc_l_lower = -vehicle_param_.width() / 2.0;
  const auto adc_l_upper = -adc_l_lower;
  ADEBUG << " ends  : " << best_curve->GetEndS()
         << "endv  :" << best_curve->GetEndV();
  for (std::size_t i = 0; i < point_count; ++i) {
    const auto& point = dense_points.at(i);
    const auto t = point.t();
    const auto s = point.s();
    if (s <
        cache.GetBasicCache().GetHighRoadRightEndS() - kMergePreviewDistance) {
      continue;
    }
    ADEBUG << " t : " << t << "s :" << s << "v :" << point.v()
           << " a :" << point.a();
    const auto adc_s_upper = s + vehicle_param_.front_edge_to_center();
    const auto adc_s_lower = s - vehicle_param_.back_edge_to_center();

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

      auto safe_distance = kMinSafeDistance + fmin(point.v() * 0.25, 5.0);
      auto upper_buffer = kMinSafeDistance;
      auto lower_buffer = kMinSafeDistance;
      // calucalate s_upper, s_lower, obstacle_v
      const auto& obstacle_info = slt_obstacle_cache->GetObstacleInfoAtTime(t);

      const auto obstacle_location =
          (adc_s_upper < obstacle_info.s_lower)
              ? STObstacleLocation::ABOVE
              : ((adc_s_lower > obstacle_info.s_upper)
                     ? STObstacleLocation::BELOW
                     : STObstacleLocation::CROSS);
      if (obstacle_location == STObstacleLocation::ABOVE) {
        const auto dv = 1.05 * point.v() - obstacle_info.ds;
        safe_distance += dv > 0 ? dv * dv * 0.3 : 0.0;
        lower_buffer = 0.0;
        upper_buffer = safe_distance;
      } else if (obstacle_location == STObstacleLocation::BELOW) {
        const auto dv = 1.05 * obstacle_info.ds - point.v();
        if (dv > 0) {
          safe_distance += dv * dv * 0.3;
        } else {
          safe_distance = fmax(safe_distance + dv * 6.0, 0.0);
        }
        lower_buffer = safe_distance;
        upper_buffer = 0.0;
      } else {
        upper_buffer = kMinSafeDistance;
        lower_buffer = kMinSafeDistance;
      }
      ADEBUG << " obs id :" << slt_obstacle_cache->GetId()
             << " obstacle_info.s_lower :" << obstacle_info.s_lower
             << " lower_buffer :" << lower_buffer
             << " obstacle_info.s_upper :" << obstacle_info.s_upper
             << " upper_buffer :" << upper_buffer << " v :" << obstacle_info.ds
             << " obstacle_info.l_lower :" << obstacle_info.l_lower
             << " obstacle_info.l_upper :" << obstacle_info.l_upper;
      if ((adc_s_lower - lower_buffer <= obstacle_info.s_upper &&
           adc_s_upper + upper_buffer >= obstacle_info.s_lower) &&
          (adc_l_lower - kLBuffer <= obstacle_info.l_upper &&
           adc_l_upper + kLBuffer >= obstacle_info.l_lower)) {
        ADEBUG << "!!!!!!DUANG";
        // 有碰撞
        return false;
      }
    }
  }
  // if (best_curve->GetEndV() < vehicle_param_.max_abs_speed_when_stopped() &&
  //     best_curve->GetEndS() < (cache.GetBasicCache().GetHighRoadRightEndS() +
  //                              vehicle_param_.front_edge_to_center())) {
  //   ADEBUG << "-------no duang but need stop";
  //   return false;
  // }
  generator->SetStartMergeStop(false);
  return true;
}

}  // namespace planning
}  // namespace TL
