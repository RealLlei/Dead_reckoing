/******************************************************************************
 * Copyright 2021 The TL Authors. All Rights Reserved.
 *****************************************************************************/
#pragma once

#include <memory>
#include <string>

#include "common/status/status.h"
#include "planning/common/frame.h"
#include "planning/common/function_statistics.h"
#include "planning/prediction/common/message_process.h"
#include "planning/prediction/inference_manager.h"
#include "planning/prediction/predictor/predictor_manager.h"
#include "planning/prediction/scenario/scenario_manager.h"

/**
 * @namespace TL::prediction
 * @brief TL::prediction
 */
namespace TL {
namespace prediction {

using TL::common::ErrorCode;
using TL::common::Status;

class Prediction {
 public:
  Prediction() = default;
  ~Prediction();

  Prediction(const Prediction&) = delete;
  Prediction& operator=(const Prediction&) = delete;
  Prediction(Prediction&&) = default;
  Prediction& operator=(Prediction&&) = default;

  Status Init(const PredictionConf& prediction_conf);
  Status Init();

  // bool Proc(
  //     TL::planning::LocalView* local_view,
  //     TL::localization::Localization* localization_est_ptr,
  //     TL::perception::PerceptionObstacles* perception_obs_ptr,
  //     const TL::planning::Frame* trajectorys,
  //     const TL::localization::Localization& localization_msg,
  //     const TL::perception::PerceptionObstacles& perception_obstacles,
  //     std::shared_ptr<prediction::PredictionObstacles> prediction_obstacles);

  bool Proc(const std::shared_ptr<TL::planning::LocalView>& local_view,
            prediction::PredictionObstacles* prediction_obstacles);

  ContainerManager* container_manager() { return container_manager_.get(); }

  PredictorManager* predictor_manager() { return predictor_manager_.get(); }

  ScenarioManager* scenario_manager() { return scenario_manager_.get(); }

 private:
  std::shared_ptr<InferenceManager> inference_manager_;

  std::shared_ptr<ContainerManager> container_manager_;

  std::unique_ptr<PredictorManager> predictor_manager_;

  std::unique_ptr<ScenarioManager> scenario_manager_;
};

}  // namespace prediction
}  // namespace TL
