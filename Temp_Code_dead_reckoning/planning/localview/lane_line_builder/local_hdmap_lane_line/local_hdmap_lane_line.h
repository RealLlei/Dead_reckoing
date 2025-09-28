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
#include <utility>
#include <vector>

#include "common/configs/config_gflags.h"
#include "common/status/status.h"
#include "map/hdmap/hdmap_util.h"
#include "planning/common/dependency_injector.h"
#include "planning/common/planning_context.h"
#include "planning/common/planning_gflags.h"
#include "planning/common/util/util.h"
#include "planning/localview/lane_line_builder/lane_line_base.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/decider_data.h"
#include "planning/localview/lane_line_builder/real_hdmap_lane_line/inside_active_condition.h"
#include "planning/localview/local_view.h"
#include "planning/routing/routing.h"

namespace TL {
namespace planning {

class LocalHDMapLaneLine : public LaneLineBase {
 public:
  LocalHDMapLaneLine();

  /**
   * @brief destructor
   */
  ~LocalHDMapLaneLine() override = default;

  /**
   * @brief module initialization function
   * @return initialization status
   */
  TL::common::Status Init() override;

  TL::common::Status Init(
      const std::shared_ptr<LocalViewData>& local_view_data);
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

  const std::shared_ptr<hdmap::HDMap>& GetHDMapPtr() { return local_hd_map_; }

 private:
  void GenerateRoutingThread();
  bool UpdateRoutingRequest(const std::shared_ptr<LocalView>& local_view,
                            const functionmanager::HdmapSubState& state_type);
  bool UpdateRoutingResponse();
  bool BuildFirstRouting(const std::shared_ptr<LocalView>& local_view);
  bool SolveChangeLaneType(const std::shared_ptr<LocalView>& local_view);
  bool ReturnStatus(functionmanager::FunctionManagerOut* to_fct, bool status);
  std::shared_ptr<const common::VehicleState> vehicle_state_{nullptr};
  routing::Routing routing_;
  std::shared_ptr<hdmap::HDMap> local_hd_map_{nullptr};
  std::shared_ptr<navigation_hdmap::MapMsg> current_map_msg_{nullptr};
  // routing线程使用
  std::shared_ptr<routing::RoutingRequest> current_routing_request_{nullptr};
  std::shared_ptr<routing::RoutingResponse> current_routing_response_{nullptr};
  std::shared_ptr<routing::RoutingResponse> from_file_routing_response_{
      nullptr};
  std::shared_ptr<routing::RoutingRequest> last_routing_request_{nullptr};
  std::shared_ptr<routing::RoutingResponse> last_routing_response_{nullptr};
  std::shared_ptr<navigation_hdmap::MapMsg> empty_map_msg_{nullptr};
  routing::RoutingResponse last_routing_;

  std::thread task_routing_thread_;

  std::atomic<int> previous_utm_zone_{-1};
  int landmark_index_{0};

  std::mutex vehicle_state_mutex_;
  std::mutex routing_response_mutex_;
  std::mutex routing_request_mutex_;

  std::string updated_map_time_;
  std::shared_ptr<hdmap::PncMap> pnc_map_;
  TL::planning::lanelineprocess::DebounceModule local_hdmap_debounce_;
  std::unique_ptr<InsideActiveCondition> inside_active_conditon_ = nullptr;
  std::atomic<bool> is_routing_running_{false};
  std::atomic<bool> routing_success_{false};

  std::atomic<bool> is_in_hdmap_{false};
  std::atomic<bool> need_first_routing_{false};
  bool is_new_routing_responce_{true};
  bool is_init_success_{true};
};

}  // namespace planning
}  // namespace TL
