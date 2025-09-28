/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
 * @file
 **/

#pragma once

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "common/filters/mean_filter.h"
#include "common/status/status.h"
#include "planning/common/frame.h"
#include "planning/tasks/task.h"

#include "planning/proto/planning_config.pb.h"

namespace TL {
namespace planning {
namespace scenario {

class Stage {
 public:
  enum StageStatus {
    ERROR = 1,
    READY = 2,
    RUNNING = 3,
    FINISHED = 4,
  };

  Stage(const ScenarioConfig::StageConfig& config,
        const std::shared_ptr<DependencyInjector>& injector);

  virtual ~Stage() = default;

  const ScenarioConfig::StageConfig& config() const { return config_; }

  ScenarioStatus::StageType stage_type() const { return config_.stage_type(); }

  /**
   * @brief Each stage does its business logic inside Process function.
   * If the stage want to transit to a different stage after finish,
   * it should set the type of 'next_stage_'.
   */
  virtual std::pair<StageStatus, common::Status> Process(
      const common::TrajectoryPoint& planning_init_point, Frame* frame) = 0;

  /**
   * @brief The sequence of tasks inside the stage. These tasks usually will be
   * executed in order.
   */
  const std::vector<Task*>& TaskList() const { return task_list_; }

  const std::string& Name() const;

  template <typename T>
  T* GetContextAs() {
    return static_cast<T*>(context_);
  }

  void SetContext(void* context) { context_ = context; }

  Task* FindTask(TaskConfig::TaskType task_type) const;

  ScenarioStatus::StageType NextStage() const { return next_stage_; }

  common::Status PlanOnReferenceLine(
      const common::TrajectoryPoint& planning_start_point, Frame* frame,
      ReferenceLineInfo* reference_line_info);

  bool ExecuteTaskSuccessfully(const common::Status& ret, const Task* task,
                               const Frame* frame, uint* final_path_task_order,
                               ReferenceLineInfo* reference_line_info,
                               bool* path_task_ok);

  void PlanFallbackTrajectory(
      const common::TrajectoryPoint& planning_start_point, Frame* frame,
      ReferenceLineInfo* reference_line_info);

  void GenerateFallbackPathProfile(const ReferenceLineInfo* reference_line_info,
                                   PathData* path_data);

  bool RetrieveLastFrameSpeedProfile(
      const ReferenceLineInfo* reference_line_info, const Frame* frame,
      SpeedData* speed_data);

  bool RetrieveLastFramePathProfile(
      const ReferenceLineInfo& reference_line_info, const Frame& frame,
      PathData* path_data);

  static common::SLPoint GetStopSL(const ObjectStop& stop_decision,
                                   const ReferenceLine& reference_line);

  static void RecordObstacleDebugInfo(ReferenceLineInfo* reference_line_info);

  bool AdjustStitchingTrajectoryPoints(
      const Frame* frame, const ReferenceLineInfo& reference_line_info);

  std::pair<Stage::StageStatus, common::Status> ExecuteTaskOnReferenceLine(
      const common::TrajectoryPoint& planning_start_point, Frame* frame);

  common::Status ExecuteTaskOnOpenSpace(Frame* frame,
                                        bool init_open_space = false);

  virtual Stage::StageStatus FinishScenario();

  static void RecordDebugInfo(ReferenceLineInfo* reference_line_info,
                              const std::string& name, double time_diff_ms);

  void GeneratePauseTrajectory(Frame* frame);
  bool GeneratePauseTrajectory(
      const common::TrajectoryPoint& planning_start_point, Frame* frame);

  void DealHmiChangeLaneStatus(const Frame* frame);

  static double PauseACC(const common::VehicleState& vehicle_state);

 private:
  static void RecordOpenSpaceTaskConsumeTime(const std::string& name,
                                             double time_ms,
                                             OpenSpaceInfo* open_space_info);

  void ResetFilter();

 protected:
  std::map<TaskConfig::TaskType, std::unique_ptr<Task>> tasks_;
  std::vector<Task*> task_list_;
  ScenarioConfig::StageConfig config_;
  ScenarioStatus::StageType next_stage_;
  void* context_ = nullptr;
  std::string name_;
  std::shared_ptr<DependencyInjector> injector_;

 private:
  double road_traffic_jams_start_timestamp_ = 0.0;
  bool is_lane_change_prepare_overtime_ = false;

  bool last_curv_check_ = true;
  bool use_joint_optizimizer_ = false;
  double smooth_mean_dx_ = 0.0;
  double smooth_mean_dy_ = 0.0;
  double smooth_mean_dtheta_ = 0.0;
  constexpr static std::uint_fast8_t StiTrajPointFilterWinSize = 10;
  common::MeanFilter adjust_stitch_dx_filter_{StiTrajPointFilterWinSize};
  common::MeanFilter adjust_stitch_dy_filter_{StiTrajPointFilterWinSize};
  common::MeanFilter adjust_stitch_dtheta_filter_{StiTrajPointFilterWinSize};
};

#define DECLARE_STAGE(NAME, CONTEXT)                                      \
  class(NAME) : public Stage {                                            \
   public:                                                                \
    explicit NAME(const ScenarioConfig::StageConfig& config)              \
        : Stage(config) {}                                                \
    std::pair<Stage::StageStatus, Status> Process(                        \
        const common::TrajectoryPoint& planning_init_point, Frame* frame) \
        override;                                                         \
    (CONTEXT) * GetContext() {                                            \
      return GetContextAs<CONTEXT>();                                     \
    }                                                                     \
  }

}  // namespace scenario
}  // namespace planning
}  // namespace TL
