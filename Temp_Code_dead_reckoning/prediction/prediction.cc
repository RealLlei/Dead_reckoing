
//   Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.

#include "planning/prediction/prediction.h"

#include "common/feature_output.h"
#include "common/file/file.h"
#include "common/prediction_system_gflags.h"
#include "common/status/status.h"
#include "common/time/clock.h"
#include "common/util/message_util.h"
#include "planning/prediction/inference_manager.h"
#include "planning/prediction/proto/prediction_conf.pb.h"

namespace TL {
namespace prediction {

using TL::common::Status;

Prediction::~Prediction() {
  TL::planning::FunctionStatistics done_guard("~Prediction()");
  if (FLAGS_enable_online_record4Prediction) {
    FeatureOutput::Close();
  }
}

Status Prediction::Init(const PredictionConf& prediction_conf) {
  container_manager_ = std::make_shared<ContainerManager>();
  inference_manager_ = std::make_shared<InferenceManager>();
  predictor_manager_ = std::make_unique<PredictorManager>();
  scenario_manager_ = std::make_unique<ScenarioManager>();

  return MessageProcess::Init(container_manager_.get(),
                              predictor_manager_.get(),
                              inference_manager_.get(), prediction_conf);
}

Status Prediction::Init() {
  container_manager_ = std::make_shared<ContainerManager>();
  inference_manager_ = std::make_shared<InferenceManager>();
  predictor_manager_ = std::make_unique<PredictorManager>();
  scenario_manager_ = std::make_unique<ScenarioManager>();

  PredictionConf prediction_conf;
  if (!common::GetProtoFromFile(FLAGS_prediction_conf_file, &prediction_conf)) {
    return Status(ErrorCode::PREDICTION_INPUT_ERROR,
                  "Unable to load predicition conf file");
  }
  common::GetProtoFromFile(FLAGS_prediction_conf_file, &prediction_conf);

  return MessageProcess::Init(container_manager_.get(),
                              predictor_manager_.get(),
                              inference_manager_.get(), prediction_conf);
}

// bool Prediction::Proc(
//     TL::planning::LocalView* local_view,
//     TL::localization::Localization* localization_est_ptr,
//     TL::perception::PerceptionObstacles* perception_obs_ptr,
//     const TL::planning::Frame* trajectorys,
//     const TL::localization::Localization& localization_msg,
//     const TL::perception::PerceptionObstacles& perception_obstacles,
//     std::shared_ptr<prediction::PredictionObstacles> prediction_obstacles) {
//   double timestamp_last = 0.0;
//   while (true) {
//     std::this_thread::sleep_for(std::chrono::milliseconds(50));
//     if (local_view != nullptr && localization_est_ptr != nullptr &&
//         perception_obs_ptr != nullptr && trajectorys != nullptr &&
//         perception_obs_ptr->header().data_stamp() != timestamp_last) {
//       timestamp_last = perception_obs_ptr->header().data_stamp();

//       // reset prediction
//       local_view->SetPredictionObstaclesPtr(prediction_obstacles);

//       TL::planning::ADCTrajectory adc_trajectory =
//           trajectorys->current_frame_planned_trajectory();

//       double timestamp_1 = common::Clock::NowInSeconds();

//       if (localization_msg.has_header()) {
//         MessageProcess::OnLocalization(container_manager_.get(),
//                                        localization_msg);
//       }
//       double timestamp_2 = common::Clock::NowInSeconds();

//       ADEBUG << "localization period = " << (timestamp_2 - timestamp_1) *
//       1000; if (adc_trajectory.has_header()) {
//         // MessageProcess::OnPlanning(container_manager_.get(),
//         adc_trajectory);
//       }
//       double timestamp_3 = common::Clock::NowInSeconds();

//       ADEBUG << "adc_trajectory period = "
//              << (timestamp_3 - timestamp_2) * 1000;
//       if (perception_obstacles.has_header()) {
//         MessageProcess::OnPerception(
//             perception_obstacles, container_manager_,
//             evaluator_manager_.get(), predictor_manager_.get(),
//             scenario_manager_.get(), prediction_obstacles.get());
//       } else {
//         return false;
//       }
//       double timestamp_4 = common::Clock::NowInSeconds();

//       ADEBUG << "on perception period = " << (timestamp_4 - timestamp_3) *
//       1000
//              << "all period = " << (timestamp_4 - timestamp_1) * 1000;

//       return true;
//     }
//   }
// }

bool Prediction::Proc(
    const std::shared_ptr<TL::planning::LocalView>& local_view,
    PredictionObstacles* prediction_obstacles) {
  double start_timestamp = common::Clock::NowInSeconds();

  auto scenario_manager_status =
      scenario_manager_->ProcessLocalView(local_view);

  auto on_localization_status = MessageProcess::OnLocalization(
      container_manager_.get(), *local_view->GetLocalization(),
      scenario_manager_.get());

  // if (adc_trajectory.has_header()) {
  // MessageProcess::OnPlanning(container_manager_.get(), adc_trajectory);
  // }

  auto on_perception_status = MessageProcess::OnPerception(
      local_view, container_manager_, predictor_manager_.get(),
      scenario_manager_.get(), prediction_obstacles);
  double end_timestamp = common::Clock::NowInSeconds();

  prediction_obstacles->set_start_timestamp(start_timestamp);
  prediction_obstacles->set_end_timestamp(end_timestamp);
  if (scenario_manager_status.ok() && on_localization_status.ok() &&
      on_perception_status.ok()) {
    Status(common::ErrorCode::OK)
        .Save(prediction_obstacles->mutable_header()->mutable_status());
  } else {
    Status(common::ErrorCode::PREDICTION_PREDICTOR_ERROR)
        .Save(prediction_obstacles->mutable_header()->mutable_status());
    AERROR << "prediction failed! ErrorCode: "
           << prediction_obstacles->mutable_header()->DebugString();
  }
  common::util::FillHeader("prediction", prediction_obstacles);
  // align header timestamp with vehicle state timestamp
  if (local_view->HasValidVehicleStateHeader() &&
      local_view->GetVehicleState()->has_timestamp()) {
    prediction_obstacles->mutable_header()->set_data_stamp(
        local_view->GetVehicleState()->timestamp());
  }

  const auto time_diff_ms = (end_timestamp - start_timestamp) * 1000;
  AINFO << "------prediction_period_end------seq "
        << prediction_obstacles->header().seq() << ", run " << time_diff_ms
        << " ms, prediction_end_time:" << FIXED << SETPRECISION(3)
        << end_timestamp;

  return true;
}

}  // namespace prediction
}  // namespace TL
