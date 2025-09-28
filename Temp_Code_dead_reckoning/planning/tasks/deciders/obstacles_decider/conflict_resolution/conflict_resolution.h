/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2023. All rights reserved.
 * Description:  conflict_resolution.h
 */
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include "planning/common/obstacle.h"
#include "planning/common/reference_line_info.h"
#include "proto/perception/perception_obstacle.pb.h"

namespace TL::planning {

class ConflictResolution {
 public:
  explicit ConflictResolution(bool enable_debug);
  ConflictResolution() = default;
  ~ConflictResolution() = default;

  /**
   * @brief 冲突解决.
   * @param reference_line_info
   */
  void Process(ReferenceLineInfo* reference_line_info);
  /**
   * @brief 清空缓存
   */
  void ClearCache();

 private:
  enum BehaviorType {
    UNKNOW = 0,
    LEFT_NUDGE = 1,
    RIGHT_NUDGE = 2,
  };

  using DecisionInfo = std::pair<BehaviorType, int>;
  using ObsPredictionAndPerceptionId = std::pair<std::string, int32_t>;

  struct IdHash {
    size_t operator()(const ObsPredictionAndPerceptionId& id) const {
      return std::hash<std::string>{}(id.first);
    }
  };

  /**
   * @brief 维护历史信息.
   * @param reference_line_info
   * @param obstacle_id
   * @param behavior_type
   */
  void MaintainingHistoricalInfomation(
      const ReferenceLineInfo& reference_line_info,
      const ObsPredictionAndPerceptionId& obstacle_id,
      const BehaviorType& behavior_type);
  /**
   * @brief 稳定决策.
   * @param reference_line_info
   */
  void StableDecisionInfomation(ReferenceLineInfo* reference_line_info);
  /**
   * @brief Debug.
   * @param reference_line_info
   */
  void Debug(ReferenceLineInfo* reference_line_info);

  std::unordered_map<ObsPredictionAndPerceptionId, DecisionInfo, IdHash>
      history_decision_nudge_counter_info_;
  bool enable_debug_ = false;
};
}  // namespace TL::planning
