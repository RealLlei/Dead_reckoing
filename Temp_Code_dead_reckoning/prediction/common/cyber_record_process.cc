/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "planning/prediction/common/cyber_record_process.h"

#include <filesystem>
#include <memory>

#include "common/adapters/adapter_gflags.h"
#include "common/file/file.h"
#include "common/file/log.h"
#include "common/time/clock.h"
#include "planning/localview/local_view.h"
#include "planning/prediction/common/feature_output.h"
#include "planning/prediction/common/junction_analyzer.h"
#include "planning/prediction/common/message_process.h"
#include "planning/prediction/common/prediction_constants.h"
#include "planning/prediction/common/prediction_gflags.h"
#include "planning/prediction/common/prediction_system_gflags.h"
#include "planning/prediction/common/validation_checker.h"
#include "planning/prediction/predictor/predictor_manager.h"
#include "planning/prediction/scenario/prioritization/obstacles_prioritizer.h"
#include "planning/prediction/scenario/right_of_way/right_of_way.h"
#include "planning/prediction/util/data_extraction.h"

#include "proto/fsm/function_manager.pb.h"
#include "proto/perception/perception_obstacle.pb.h"
#include "proto/prediction/prediction_obstacle.pb.h"
#include "proto/routing/routing.pb.h"

#include "cyber/record/record_reader.h"
#include "cyber/record/record_writer.h"

namespace TL {
namespace prediction {

using TL::cyber::record::RecordMessage;
using TL::cyber::record::RecordReader;
using TL::cyber::record::RecordWriter;
using TL::functionmanager::FunctionManagerIn;
using TL::functionmanager::FunctionManagerOut;
using TL::localization::Localization;
using TL::perception::PerceptionObstacles;
using TL::planning::ADCTrajectory;
using TL::planning::LocalView;
using TL::routing::RoutingResponse;

// using TL::storytelling::Stories;

void CyberRecordProcess::ProcessOfflineData(
    const PredictionConf& prediction_conf,
    const std::shared_ptr<ContainerManager>& container_manager,
    PredictorManager* predictor_manager, ScenarioManager* scenario_manager,
    const std::string& record_filepath) {
  RecordReader reader(record_filepath);
  RecordMessage message;
  RecordWriter writer;
  std::shared_ptr<hdmap::HDMap> hd_map = std::make_shared<hdmap::HDMap>();

  std::string process_path = "./data/records/" + record_filepath;
  std::filesystem::path path(process_path);
  const auto& parent_path = path.parent_path();
  if (!std::filesystem::exists(parent_path)) {
    std::filesystem::create_directories(parent_path);
  }
  if (FLAGS_prediction_offline_mode == PredictionConstants::kDumpRecord) {
    writer.Open(process_path);
  }
  static bool localization_init = false;
  static bool map_init = false;
  static bool perception_update = false;
  static bool function_manager_init = false;
  static PerceptionObstacles perception_obstacles;
  static FunctionManagerIn function_manager_in;
  static auto local_view = std::make_shared<LocalView>();
  static RoutingResponse routing_response;

  bool use_ehp_map = false;
  // 判断是不是使用静态地图
  while (reader.ReadMessage(&message)) {
    if (message.channel_name == fLS::FLAGS_routing_response_topic) {
      if (routing_response.ParseFromString(message.content) &&
          routing_response.road_size() > 0 &&
          routing_response.measurement().info_size() > 0) {
        auto map_msg = std::make_shared<navigation_hdmap::MapMsg>();
        map_msg->ParseFromString(routing_response.measurement().info(0));
        if (map_msg->has_hdmap() && map_msg->hdmap().lane_size() > 0) {
          use_ehp_map = true;
          break;
        }
      }
    }
  }
  if (!use_ehp_map) {
    hd_map = hdmap::CreateMap(hdmap::BaseMapFile());
    hdmap::HDMapUtil::SetMapForPrediction(hd_map);
    map_init = true;
  }
  AERROR << "Use_ehp_map " << use_ehp_map;

  reader.Reset();
  while (reader.ReadMessage(&message)) {
    auto msg_time = message.time;
    if (std::isgreater(FLAGS_offline_mode_start_timestamp, 0.0) &&
        std::isgreater(FLAGS_offline_mode_end_timestamp, 0.0) &&
        FLAGS_offline_mode_end_timestamp > FLAGS_offline_mode_start_timestamp) {
      auto start_time_str = std::to_string(FLAGS_offline_mode_start_timestamp);
      auto end_time_str = std::to_string(FLAGS_offline_mode_end_timestamp);
      auto msg_time_str = std::to_string(msg_time);

      if (msg_time_str < start_time_str) {
        continue;
      }
      if (msg_time_str > end_time_str) {
        break;
      }
    }

    if (localization_init && map_init &&
        message.channel_name ==
            prediction_conf.topic_conf().planning_trajectory_topic()) {
      ADCTrajectory adc_trajectory;
      if (adc_trajectory.ParseFromString(message.content)) {
        MessageProcess::OnPlanning(container_manager.get(), adc_trajectory);
        if (FLAGS_prediction_offline_mode == PredictionConstants::kDumpRecord) {
          writer.WriteMessage<ADCTrajectory>(message.channel_name,
                                             adc_trajectory, msg_time);
        }
      }
    } else if (function_manager_init &&
               message.channel_name ==
                   prediction_conf.topic_conf().localization_topic()) {
      static Localization localization;
      if (localization.ParseFromString(message.content)) {
        if (FLAGS_prediction_offline_mode == PredictionConstants::kDumpRecord) {
          writer.WriteMessage<Localization>(message.channel_name, localization,
                                            msg_time);
        }
        if (localization.rtk_status() == 0 ||
            localization.location_state() == 0) {
          AERROR << "localization is not init! rtk_status: "
                 << localization.rtk_status()
                 << "\t location_state: " << localization.location_state()
                 << "\n"
                 << localization.header().ShortDebugString();
        } else {
          localization_init = true;
          MessageProcess::OnLocalization(container_manager.get(), localization,
                                         scenario_manager);
        }
      }
    } else if (message.channel_name == fLS::FLAGS_routing_response_topic) {
      if (routing_response.ParseFromString(message.content) &&
          routing_response.road_size() > 0 &&
          routing_response.measurement().info_size() > 0) {
        if (use_ehp_map) {
          auto map_msg = std::make_shared<navigation_hdmap::MapMsg>();
          map_msg->ParseFromString(routing_response.measurement().info(0));
          if (map_msg->has_hdmap() && map_msg->hdmap().lane_size() > 0) {
            hd_map->LoadMapFromProto(map_msg->hdmap());
            hdmap::HDMapUtil::SetMapForPrediction(hd_map);

            if (FLAGS_prediction_offline_mode ==
                PredictionConstants::kDumpRecord) {
              writer.WriteMessage<RoutingResponse>(message.channel_name,
                                                   routing_response, msg_time);
            }

            map_init = true;
          } else {
            ADEBUG << "map init false";
          }
        } else {
          if (FLAGS_prediction_offline_mode ==
              PredictionConstants::kDumpRecord) {
            writer.WriteMessage<RoutingResponse>(message.channel_name,
                                                 routing_response, msg_time);
          }
        }
      }
    } else if (message.channel_name ==
               prediction_conf.topic_conf().perception_obstacle_topic()) {
      if (perception_obstacles.ParseFromString(message.content)) {
        if (FLAGS_prediction_offline_mode == PredictionConstants::kDumpRecord) {
          writer.WriteMessage<PerceptionObstacles>(
              message.channel_name, perception_obstacles, msg_time);
        }
        perception_update = true;
      }
    } else if (message.channel_name == fLS::FLAGS_planning_fct_topic) {
      if (function_manager_in.ParseFromString(message.content)) {
        local_view->SetFunctionManagerInPtr(
            std::make_shared<functionmanager::FunctionManagerIn>(
                function_manager_in));

        FunctionManagerOut function_manager_out;
        local_view->SetFunctionManagerOutPtr(
            std::make_shared<functionmanager::FunctionManagerOut>(
                function_manager_out));

        scenario_manager->ProcessLocalView(local_view);
        function_manager_init = true;
      }
    } else if (message.channel_name ==
               prediction_conf.topic_conf().prediction_topic()) {
      PredictionObstacles prediction_obstacles;
      local_view->SetPerceptionObstaclesPtr(
          std::make_shared<PerceptionObstacles>(perception_obstacles));

      MessageProcess::OnPerception(local_view, container_manager,
                                   predictor_manager, scenario_manager,
                                   &prediction_obstacles);
      if (localization_init && map_init && perception_update &&
          FLAGS_prediction_offline_mode == PredictionConstants::kDumpRecord) {
        prediction_obstacles.mutable_header()->CopyFrom(
            perception_obstacles.header());
        writer.WriteMessage<PredictionObstacles>(
            prediction_conf.topic_conf().prediction_topic(),
            prediction_obstacles, msg_time);
        ADEBUG << "Generated a new prediction message.";
      }
    }
  }
  if (FLAGS_prediction_offline_mode == PredictionConstants::kDumpRecord) {
    writer.Close();

    // rm uncompleted record
    if (!map_init) {
      std::filesystem::path raw_record_fp(record_filepath);
      std::string raw_filename = raw_record_fp.filename().string();
      AERROR << "map init false,do not export this record file";
      for (const auto& entry :
           std::filesystem::directory_iterator(parent_path)) {
        if (entry.is_regular_file()) {
          if (entry.path().filename().string().find(raw_filename) !=
              std::string::npos) {
            std::filesystem::remove(entry.path());
          }
        }
      }
    }
  }
}

}  // namespace prediction
}  // namespace TL
