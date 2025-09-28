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

#include "planning/tasks/task_factory.h"

#include "planning/tasks/deciders/avoid_alongside_decider/avoid_alongside_decider.h"
#include "planning/tasks/deciders/creep_decider/creep_decider.h"
#include "planning/tasks/deciders/lane_change_decider/lane_change_decider.h"
#include "planning/tasks/deciders/obstacles_decider/obstacles_decider.h"
#include "planning/tasks/deciders/open_space_decider/open_space_roi_decider.h"
#include "planning/tasks/deciders/path_assessment_decider/path_assessment_decider.h"
#include "planning/tasks/deciders/path_bounds_decider/path_bounds_decider.h"
#include "planning/tasks/deciders/path_decider/path_decider.h"
#include "planning/tasks/deciders/path_lane_borrow_decider/path_lane_borrow_decider.h"
// #include "planning/tasks/deciders/path_reference_decider/path_reference_decider.h"
#include "planning/tasks/deciders/path_reuse_decider/path_reuse_decider.h"
// #include "planning/tasks/deciders/rss_decider/rss_decider.h"
#include "planning/tasks/deciders/astar_path_decider/astar_path_decider.h"
#include "planning/tasks/deciders/guide_line_bounds_decider/guide_line_bounds_decider.h"
#include "planning/tasks/deciders/obstacle_filter_decider/obstacle_filter_decider.h"
#include "planning/tasks/deciders/obstacle_game_decider/obstacle_game_decider.h"
#include "planning/tasks/deciders/precise_path_bounds_decider/precise_path_bounds_decider.h"
#include "planning/tasks/deciders/rule_based_stop_decider/rule_based_stop_decider.h"
#include "planning/tasks/deciders/speed_bounds_decider/speed_bounds_decider.h"
#include "planning/tasks/deciders/speed_decider/speed_decider.h"
#include "planning/tasks/deciders/st_bounds_decider/st_bounds_decider.h"
#include "planning/tasks/optimizers/guide_line_path_optimizer/guide_line_path_optimizer.h"
#include "planning/tasks/optimizers/open_space_path_generation/open_space_path_provider.h"
#include "planning/tasks/optimizers/open_space_path_partition/open_space_path_partition.h"
#include "planning/tasks/optimizers/open_space_speed_optimizer/open_space_speed_optimizer.h"
#include "planning/tasks/optimizers/open_space_straight_path/open_space_straight_path_provider.h"
// #include "planning/tasks/optimizers/path_time_heuristic/path_time_heuristic_optimizer.h"
#include "planning/tasks/optimizers/piecewise_jerk_path/piecewise_jerk_path_optimizer.h"
// #include "planning/tasks/optimizers/piecewise_jerk_speed/piecewise_jerk_speed_optimizer.h"
#include "planning/tasks/deciders/joint_post_decider/joint_post_decider.h"
#include "planning/tasks/optimizers/joint_optimizer/joint_optimizer.h"
#include "planning/tasks/optimizers/speed_data_optimizer/speed_data_optimizer.h"
#include "planning/tasks/optimizers/ssc_trajectory_optimizer/ssc_trajectory_optimizer.h"
#include "planning/tasks/task.h"

#include "planning/proto/planning_config.pb.h"
#include "planning/proto/task_config.pb.h"

namespace TL {
namespace planning {

TL::common::util::Factory<
    TaskConfig::TaskType, Task,
    Task* (*)(const TaskConfig& config,
              const std::shared_ptr<DependencyInjector>& injector),
    std::unordered_map<
        TaskConfig::TaskType,
        Task* (*)(const TaskConfig& config,
                  const std::shared_ptr<DependencyInjector>& injector),
        std::hash<int>>>
    TaskFactory::task_factory_;

std::unordered_map<TaskConfig::TaskType, TaskConfig, std::hash<int>>
    TaskFactory::default_task_configs_;

void TaskFactory::Init(const PlanningConfig& config,
                       const std::shared_ptr<DependencyInjector>& injector) {
  UNUSED(injector);
  ///////////////////////////
  // deciders
  task_factory_.Register(
      TaskConfig::CREEP_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new CreepDecider(config, injector);
      });
  task_factory_.Register(
      TaskConfig::LANE_CHANGE_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new LaneChangeDecider(config, injector);
      });
  task_factory_.Register(
      TaskConfig::OBSTACLES_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new ObstaclesDecider(config, injector);
      });
  task_factory_.Register(
      TaskConfig::PATH_ASSESSMENT_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new PathAssessmentDecider(config, injector);
      });
  task_factory_.Register(
      TaskConfig::PATH_BOUNDS_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new PathBoundsDecider(config, injector);
      });
  task_factory_.Register(
      TaskConfig::PATH_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new PathDecider(config, injector);
      });
  task_factory_.Register(
      TaskConfig::AVOID_ALONGSIDE_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new AvoidAlongsideDecider(config, injector);
      });
  task_factory_.Register(
      TaskConfig::PATH_LANE_BORROW_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new PathLaneBorrowDecider(config, injector);
      });
  // task_factory_.Register(
  //     TaskConfig::PATH_REFERENCE_DECIDER,
  //     [](const TaskConfig& config,
  //        const std::shared_ptr<DependencyInjector>& injector) -> Task* {
  //       return new PathReferenceDecider(config, injector);
  //     });
  task_factory_.Register(
      TaskConfig::PATH_REUSE_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new PathReuseDecider(config, injector);
      });
  // task_factory_.Register(
  //     TaskConfig::RSS_DECIDER,
  //     [](const TaskConfig& config,
  //        const std::shared_ptr<DependencyInjector>& injector) -> Task* {
  //       return new RssDecider(config);
  //     });
  task_factory_.Register(
      TaskConfig::RULE_BASED_STOP_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new RuleBasedStopDecider(config, injector);
      });
  task_factory_.Register(
      TaskConfig::SPEED_BOUNDS_PRIORI_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new SpeedBoundsDecider(config, injector);
      });
  task_factory_.Register(
      TaskConfig::SPEED_BOUNDS_FINAL_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new SpeedBoundsDecider(config, injector);
      });
  task_factory_.Register(
      TaskConfig::SPEED_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new SpeedDecider(config, injector);
      });
  task_factory_.Register(
      TaskConfig::ST_BOUNDS_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new STBoundsDecider(config, injector);
      });
  task_factory_.Register(
      TaskConfig::OPEN_SPACE_ROI_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new OpenSpaceRoiDecider(config, injector);
      });
  task_factory_.Register(
      TaskConfig::GUIDE_LINE_BOUNDS_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new GuideLineBoundsDecider(config, injector);
      });
  task_factory_.Register(
      TaskConfig::OBSTACLE_FILTER_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new ObstacleFilterDecider(config, injector);
      });
  task_factory_.Register(
      TaskConfig::OBSTACLE_GAME_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new ObstacleGameDecider(config, injector);
      });
  task_factory_.Register(
      TaskConfig::GUIDE_LINE_PATH_OPTIMIZER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new GuideLinePathOptimizer(config, injector);
      });
  task_factory_.Register(
      TaskConfig::ASTAR_PATH_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new AstarPathDecider(config, injector);
      });

  ///////////////////////////
  // optimizers
  task_factory_.Register(
      TaskConfig::OPEN_SPACE_STRAIGHT_PATH,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new OpenSpaceStraightPathProvider(config, injector);
      });
  task_factory_.Register(
      TaskConfig::OPEN_SPACE_PATH_PARTITION,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new OpenSpacePathPartition(config, injector);
      });
  task_factory_.Register(
      TaskConfig::OPEN_SPACE_PATH_PROVIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new OpenSpacePathProvider(config, injector);
      });
  task_factory_.Register(
      TaskConfig::OPEN_SPACE_SPEED_OPTIMIZER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new OpenSpaceSpeedOptimizer(config, injector);
      });
  task_factory_.Register(
      TaskConfig::PIECEWISE_JERK_PATH_OPTIMIZER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new PiecewiseJerkPathOptimizer(config, injector);
      });
  // task_factory_.Register(
  //     TaskConfig::PIECEWISE_JERK_SPEED_OPTIMIZER,
  //     [](const TaskConfig& config,
  //        const std::shared_ptr<DependencyInjector>& injector) -> Task* {
  //       return new PiecewiseJerkSpeedOptimizer(config);
  //     });
  // task_factory_.Register(
  //     TaskConfig::SPEED_HEURISTIC_OPTIMIZER,
  //     [](const TaskConfig& config,
  //        const std::shared_ptr<DependencyInjector>& injector) -> Task* {
  //       return new PathTimeHeuristicOptimizer(config);
  //     });
  task_factory_.Register(
      TaskConfig::SPEED_DATA_OPTIMIZER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new SpeedDataOptimizer(config, injector);
      });
  task_factory_.Register(
      TaskConfig::PRECISE_PATH_BOUNDS_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new PrecisePathBoundsDecider(config, injector);
      });
  task_factory_.Register(
      TaskConfig::SSC_TRAJECTORY_OPTIMIZER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new SscTrajectoryOptimizer(config, injector);
      });
  task_factory_.Register(
      TaskConfig::JOINT_OPTIMIZER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new JointOptimizer(config, injector);
      });
  task_factory_.Register(
      TaskConfig::JOINT_POST_DECIDER,
      [](const TaskConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Task* {
        return new JointPostDecider(config, injector);
      });
  ///////////////////////////
  // other tasks
  for (const auto& default_task_config : config.default_task_config()) {
    default_task_configs_[default_task_config.task_type()] =
        default_task_config;
  }
}

std::unique_ptr<Task> TaskFactory::CreateTask(
    const TaskConfig& task_config,
    const std::shared_ptr<DependencyInjector>& injector) {
  TaskConfig merged_config;
  if (default_task_configs_.find(task_config.task_type()) !=
      default_task_configs_.end()) {
    merged_config = default_task_configs_[task_config.task_type()];
  }
  merged_config.MergeFrom(task_config);
  return task_factory_.CreateObject(task_config.task_type(), merged_config,
                                    injector);
}

}  // namespace planning

}  // namespace TL
