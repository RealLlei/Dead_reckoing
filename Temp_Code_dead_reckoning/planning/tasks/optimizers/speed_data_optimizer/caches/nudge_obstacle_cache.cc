/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file nudge_obstacle_cache.cc
 **/
#include "planning/tasks/optimizers/speed_data_optimizer/caches/nudge_obstacle_cache.h"

#include <cmath>
#include <limits>
#include <memory>

#include "common/file/log.h"
#include "proto/planning/lanemarkers_lane_line.pb.h"

namespace TL {
namespace planning {

NudgeObstacleCache::NudgeObstacleCache(
    const SpeedCacheConfig& config,
    const ReferenceLineInfo& reference_line_info, const Obstacle* obstacle,
    const double time_unit, const int time_count, int index,
    const NudgeObstacleCache* old_cache, double time_from_last_frame,
    bool frozon)
    : SLTObstacleCache(config, reference_line_info, obstacle, time_unit,
                       time_count, index) {
  hexagons_.assign(time_count, nullptr);
  if (GetObstacle() == nullptr) {
    return;
  }

  if (old_cache != nullptr && frozon) {
    CacheObstacleSpeed(*old_cache, time_from_last_frame);
  }

  CacheHexagon(config, reference_line_info.path_data());

  const auto& decisions = obstacle->decisions();
  const auto& decider_tags = obstacle->decider_tags();
  if (decisions.empty() || decisions.size() != decider_tags.size()) {
    return;
  }

  for (std::size_t i = 0; i < decisions.size(); ++i) {
    if (decisions.at(i).has_overtake() &&
        decider_tags.at(i) == "obstacles_decider/-overtake") {
      target_nudge_state_ = SpeedCacheConfig::OVERTAKE;
      return;
    }
    if (decisions.at(i).has_yield() &&
        decider_tags.at(i) == "obstacles_decider/-yield") {
      target_nudge_state_ = SpeedCacheConfig::FOLLOW;
      return;
    }
  }
}

void NudgeObstacleCache::CacheObstacleSpeed(const NudgeObstacleCache& old_cache,
                                            double time_from_last_frame) {
  if (old_cache.GetMaxT() < old_cache.GetMinT()) {
    return;
  }
  const auto last_time = old_cache.GetMaxT();
  const auto& last_old_obstacle_info =
      old_cache.GetObstacleInfoAtTime(last_time);

  auto* obstacle_infos = GetMutableObstacleInfos();
  if (obstacle_infos == nullptr) {
    return;
  }
  for (int i = 0; i < static_cast<int>(obstacle_infos->size()); ++i) {
    const auto t = i * GetTimeUnit() + time_from_last_frame;
    if (t < last_time) {
      const auto& old_obstacle_info = old_cache.GetObstacleInfoAtTime(t);
      obstacle_infos->at(i).ds = old_obstacle_info.ds;
      obstacle_infos->at(i).dl = old_obstacle_info.dl;
    } else {
      obstacle_infos->at(i).ds = last_old_obstacle_info.ds;
      obstacle_infos->at(i).dl = last_old_obstacle_info.dl;
    }
  }
}

void NudgeObstacleCache::CacheHexagon(const SpeedCacheConfig& config,
                                      const PathData& path_data) {
  UNUSED(path_data);
  std::vector<TL::common::math::Vec2d> t_vec_hexagon(6);
  for (int i = 0; i < GetTimeCount(); ++i) {
    const auto t = i * GetTimeUnit();
    if (t < GetMinT() || t > GetMaxT()) {
      continue;
    }

    const auto& obstacle_info = GetObstacleInfos().at(i);
    /** construct a cost_hexagon
     *                    ^ delta_v
     *                    |
     *            p1      |        p2
     *           /--------|--------\
     *          /         |         \
     *     ----p5---------|----------p6----> s
     *          \         |         /
     *           \--------|--------/
     *            p3      |        p4
     *                    |
    **/
    auto t_p1_x = -config.hexagon_cost_big_car_p1_x_buffer();
    auto t_p2_x = config.hexagon_cost_big_car_p1_x_buffer();
    auto t_p1_y = config.hexagon_cost_big_car_p1_y_buffer();
    auto t_p5_x = t_p1_x - obstacle_info.ds * config.follow_time_desired();
    auto t_p6_x = t_p2_x + obstacle_info.ds * config.follow_time_desired();

    t_vec_hexagon.at(0).set_x(t_p1_x);
    t_vec_hexagon.at(0).set_y(t_p1_y);
    t_vec_hexagon.at(1).set_x(t_p2_x);
    t_vec_hexagon.at(1).set_y(t_p1_y);  // insert p2
    t_vec_hexagon.at(2).set_x(t_p6_x);
    t_vec_hexagon.at(2).set_y(0);  // insert p6
    t_vec_hexagon.at(3).set_x(t_p2_x);
    t_vec_hexagon.at(3).set_y(-t_p1_y);  // insert p4
    t_vec_hexagon.at(4).set_x(t_p1_x);
    t_vec_hexagon.at(4).set_y(-t_p1_y);  // insert p3
    t_vec_hexagon.at(5).set_x(t_p5_x);
    t_vec_hexagon.at(5).set_y(0);  // insert p5
    hexagons_.at(i) =
        std::make_shared<TL::common::math::Polygon2d>(t_vec_hexagon);
    ADEBUG << "t:" << t;
    ADEBUG << "p1 x:" << t_p1_x << ", y:" << t_p1_y;
    ADEBUG << "p2 x:" << t_p2_x << ", y:" << t_p1_y;
    ADEBUG << "p3 x:" << t_p6_x << ", y:" << 0;
    ADEBUG << "p4 x:" << t_p2_x << ", y:" << -t_p1_y;
    ADEBUG << "p5 x:" << t_p1_x << ", y:" << -t_p1_y;
    ADEBUG << "p6 x:" << t_p5_x << ", y:" << 0;
  }
}

}  // namespace planning
}  // namespace TL
