/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */

#pragma once

#include <algorithm>
#include <atomic>
#include <future>
#include <limits>
#include <list>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "common/configs/config_gflags.h"
#include "common/status/status.h"
#include "common/vehicle_state/vehicle_state_provider.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/decider_data.h"

#include "map/ehr/ehr_factory.h"
#include "map/ehr/map_state_data_processor.h"
#include "map/hdmap/hdmap_util.h"
#include "planning/common/dependency_injector.h"
#include "planning/common/planning_context.h"
#include "planning/common/planning_gflags.h"
#include "planning/common/util/util.h"
#include "planning/localview/lane_line_builder/lane_line_base.h"
#include "planning/localview/lane_line_builder/real_hdmap_lane_line/failure_location_checker.h"
#include "planning/localview/lane_line_builder/real_hdmap_lane_line/inside_active_condition.h"
#include "planning/localview/local_view.h"
#include "planning/routing/routing.h"
#include "proto/map/map.pb.h"
#include "proto/routing/routing.pb.h"

namespace TL {
namespace planning {
using TL::hdmap::MapStateData;
using ChangeLaneTypes =
    std::unordered_map<std::string, routing::ChangeLaneType>;

class RealHDMapLaneLine : public LaneLineBase {
 public:
  RealHDMapLaneLine();

  /**
   * @brief destructor
   */
  ~RealHDMapLaneLine() override = default;

  /**
   * @brief module initialization function
   * @return initialization status
   */
  TL::common::Status Init(
      const std::shared_ptr<LocalViewData>& local_view_data);
  TL::common::Status Init() override;

  /**
   * @brief module start function
   * @return start status
   */
  TL::common::Status Start() override;

  /**
   * @brief module stop function
   */
  void Stop() override;

  /**
   * @brief main logic of the navigation_hdmap module, runs periodically
   * triggered by timer.
   */
  bool Process(const std::shared_ptr<LocalView>& local_view,
               functionmanager::FunctionManagerOut* to_fct) override;
  const std::shared_ptr<navigation_hdmap::MapMsg>& GetMapMsg(
      bool refresh) override;

  const std::shared_ptr<routing::RoutingResponse>& GetRoutingResponse()
      override {
    std::lock_guard<std::mutex> lock(routing_response_mutex_);
    return current_routing_response_;
  }

  const std::shared_ptr<hdmap::HDMap>& GetHDMapPtr() { return hd_map_; }

  bool UpdateEHPData(const std::shared_ptr<TL::ehp::EHP>& ehp_message,
                     int* received_ehp_count);

 private:
  void LogMapStateData();
  bool SetMapStateData();
  void GenerateEHRThread();
  void LoadMapStateData();

  bool UpdateRoutingResponse();
  bool SolveChangeLaneType(const std::shared_ptr<LocalView>& local_view);

  /**
   * @brief Get the Adc To Other Lane Type Len object
   * @param to_fct 
   */
  void GetAdcToOtherLaneTypeLen(functionmanager::FunctionManagerOut* to_fct);
  /**
   * @brief Set the Shrink Map object
   *  maintenance deleted map list, each time only delete one map
   * @param shrinked_map 
   * @param shrinked_map_protos 
   */
  void SetShrinkMap(const TL::hdmap::Map& shrinked_map,
                    std::list<TL::hdmap::Map>* shrinked_map_protos);

  bool ReturnStatus(functionmanager::FunctionManagerOut* to_fct, bool status);

  std::shared_ptr<hdmap::HDMap> hd_map_ = nullptr;
  std::shared_ptr<hdmap::HDMap> hd_map_ehp_ = nullptr;
  std::shared_ptr<const common::VehicleState> vehicle_state_ = nullptr;
  std::mutex vehicle_state_mutex_;
  routing::Routing routing_;

  std::shared_ptr<navigation_hdmap::MapMsg> current_map_msg_ = nullptr;
  std::shared_ptr<routing::RoutingResponse> current_routing_response_ = nullptr;
  // routing线程使用
  std::shared_ptr<routing::RoutingResponse> last_routing_response_ = nullptr;
  routing::RoutingResponse last_routing_;
  std::mutex ehp_data_mutex_;
  std::vector<std::shared_ptr<TL::ehp::EHP>> ehp_data_list_;
  std::unique_ptr<TL::ehr::Ehr> ehr_ = nullptr;
  std::thread task_ehr_thread_;
  std::atomic<bool> is_ehr_running_{false};
  std::atomic<bool> routing_success_{false};
  std::mutex routing_response_mutex_;
  int landmark_index_ = 0;
  bool is_in_hdmap_ = false;
  std::string updated_map_time_;
  std::shared_ptr<hdmap::PncMap> pnc_map_ = nullptr;
  routing::PerceptionChangeLaneTypes change_lane_types_;
  routing::Landmark ehp_navigation_landmark_;
  std::shared_ptr<navigation_hdmap::MapMsg> empty_map_msg_ = nullptr;
  std::string adc_path_id_;
  std::string target_path_id_;
  std::mutex adc_target_path_id_mutex_;
  std::atomic<int> new_utm_zone_{-1};
  std::atomic<int> previous_utm_zone_{-1};
  MapStateData map_state_data_;
  std::mutex ehr_data_mutex_;
  TL::ehr::MapStateDataProcessor map_state_data_processor_;
  TL::planning::lanelineprocess::DebounceModule local_hdmap_debounce_;
  std::unique_ptr<InsideActiveCondition> inside_active_conditon_ = nullptr;
  std::unique_ptr<FailureLocationChecker> failure_location_checker_ = nullptr;
  std::list<TL::hdmap::Map> shrinked_map_protos_;
  bool hdmap_state_{true};
};

}  // namespace planning
}  // namespace TL
