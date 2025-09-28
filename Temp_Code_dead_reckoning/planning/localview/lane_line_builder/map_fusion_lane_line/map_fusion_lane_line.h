/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */

#pragma once

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <future>
#include <limits>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "common/configs/config_gflags.h"
#include "common/status/status.h"
#include "map/hdmap/hdmap.h"
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
using MapTypePair = std::pair<int32_t, navigation_hdmap::MapMsg_MapType>;

class MapFusionLaneLine : public LaneLineBase {
 public:
  MapFusionLaneLine() = default;

  MapFusionLaneLine(const MapFusionLaneLine&) = delete;

  MapFusionLaneLine& operator=(const MapFusionLaneLine&) = delete;

  /**
   * @brief destructor
   */
  ~MapFusionLaneLine() override {
    {
      std::lock_guard<std::mutex> lg(hdmap_mutex_);
      is_exit_ = true;
      mapmsg_cv_.notify_all();
    }
    if (!FLAGS_is_record_replay && hdmap_thread_.joinable()) {
      hdmap_thread_.join();
    }
  };

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
    return routing_response_local_;
  }

  void SetMapRouting();

  void AddVirtualLaneLine(const std::shared_ptr<LocalView>& local_view);
  static void VirtualLaneLine(perception::LaneMarker* next_lane_marker,
                              const perception::LaneMarker& lane_marker,
                              const perception::LaneMarker& road_edge,
                              double width);
  double GetLaneWidth(const std::shared_ptr<LocalView>& local_view,
                      const TL::hdmap::Id& lane_id);

  const std::shared_ptr<hdmap::HDMap>& GetHDMapPtr() { return hdmap_local_; }

 private:
  bool SolveChangeLaneType(const std::shared_ptr<LocalView>& local_view);
  void GetAdcToOtherLaneTypeLen(functionmanager::FunctionManagerOut* to_fct);
  bool ReturnStatus(functionmanager::FunctionManagerOut* to_fct, bool status,
                    const std::string& msg);
  bool ReturnLocalMapStatus(functionmanager::FunctionManagerOut* to_fct,
                            bool status);
  void Fusion2NolaneChecker(const std::shared_ptr<LocalView>& local_view);
  void DealMaptype(const std::shared_ptr<LocalView>& local_view,
                   functionmanager::FunctionManagerOut* to_fct);
  std::shared_ptr<navigation_hdmap::MapMsg> empty_map_msg_;
  std::shared_ptr<hdmap::HDMap> hdmap_local_ = nullptr;
  std::shared_ptr<hdmap::HDMap> hdmap_ = nullptr;
  std::shared_ptr<hdmap::HDMap> hdmap_bak_ = nullptr;
  std::vector<std::shared_ptr<const navigation_hdmap::MapMsg>> map_fusion_vec_;
  std::mutex hdmap_mutex_;
  std::condition_variable mapmsg_cv_;
  std::thread hdmap_thread_;
  bool is_exit_{false};
  std::shared_ptr<routing::RoutingResponse> routing_response_local_ = nullptr;
  std::shared_ptr<routing::RoutingResponse> current_routing_response_ = nullptr;
  std::shared_ptr<routing::RoutingResponse> routing_response_bak_ = nullptr;
  std::shared_ptr<const common::VehicleState> vehicle_state_ = nullptr;
  std::shared_ptr<hdmap::PncMap> pnc_map_ = nullptr;
  std::unique_ptr<InsideActiveCondition> inside_active_conditon_ = nullptr;
  TL::planning::lanelineprocess::DebounceModule map_fusion_hdmap_debounce_{
      1.0, 0.0, 0.1};
  TL::navigation_hdmap::MapMsg_MapType his_fusion_maptype_{
      TL::navigation_hdmap::MapMsg_MapType_INVALID};
  TL::navigation_hdmap::MapMsg_MapType now_fusion_maptype_{
      TL::navigation_hdmap::MapMsg_MapType_INVALID};
  bool is_in_hdmap_ = false;
  bool is_map_type_change_{false};
  bool per_fusion_map_status_{false};
  size_t time_interval_{0};
  std::deque<MapTypePair> map_types_{};
};

}  // namespace planning
}  // namespace TL
