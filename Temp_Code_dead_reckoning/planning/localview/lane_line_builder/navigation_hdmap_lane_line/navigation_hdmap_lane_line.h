/*
 * @Author: 80040285 zhangyu@TLauto.com
 * @Date: 2023-08-31 13:56:57
 * @LastEditors: 80040285 zhangyu@TLauto.com
 * @LastEditTime: 2023-09-07 17:21:54
 * @FilePath: /europa/planning/localview/lane_line_builder/navigation_hdmap_lane_line/navigation_hdmap_lane_line.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
// #include "common/monitor_log/monitor_log_buffer.h"
#include "common/status/status.h"
#include "planning/localview/lane_line_builder/lane_line_base.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_logical_decider.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/lanemarker_new_decider.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/navigation_lane_map_creator.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/navigation_lane_path_generator.h"
#include "planning/localview/local_view.h"

#include "planning/proto/navigation_hdmap_config.pb.h"
#include "proto/map/navigation.pb.h"
#include "proto/perception/perception_obstacle.pb.h"

namespace TL {
namespace planning {
using functionmanager::MachineStateType;  // NOLINT

class NavigationHdmap : public LaneLineBase {
 public:
  NavigationHdmap();

  /**
   * @brief module name
   */
  static std::string Name() { return "NavigationHdmap"; }

  /**
   * @brief module initialization function
   * @return initialization status
   */
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
      bool refresh) override {
    UNUSED(refresh);
    return current_map_msg_;
  };

  const std::shared_ptr<routing::RoutingResponse>& GetRoutingResponse()
      override {
    return current_routing_response_;
  };

  planning::PerceptionMapConfig GetConfig() const;

  std::shared_ptr<LanemarkersLaneLine> GetLanemarkerDebugPtr() {
    return lanemarkerdebug_;
  }

  void Lanemarkers2Points(
      const LaneMarkers& lanemarkers,
      std::unordered_map<std::string, std::vector<Vec2d>>* lane_points_map);
  std::vector<Vec2d> Lanemarker2Points(const LaneMarker& lanemarker);
  bool DoProjection_test(const std::vector<Vec2d>& ref_v, const double& width,
                         std::vector<Vec2d>* results);
  bool FindPerpendicular_test(const Vec2d& p0, const Vec2d& p1,
                              const double& distance, Vec2d* res);
  bool LaneLineStatusToMcu(const std::shared_ptr<LocalView>& local_view,
                           functionmanager::FunctionManagerOut* to_fct);

 private:
  bool NewLanemarkerDecider(const std::shared_ptr<LocalView>& local_view,
                            functionmanager::FunctionManagerOut* to_fct);
  static void DefaultLanemarker(LaneMarker* lane_marker);
  void DebounceStatus(functionmanager::FunctionManagerOut* to_fct, bool status,
                      bool lane_line_status_to_mcu = true);
  static bool CheckLanemarkers(
      LaneMarkers* lane_markers,
      const std::shared_ptr<const perception::LaneMarkers>& lanemarkers_input);
  static void CopyLanemarker(LaneMarker* lane_marker,
                             const LaneMarker& lanemarker_input);
  planning::PerceptionMapConfig config_;
  //   TL::common::monitor::MonitorLogBuffer monitor_logger_buffer_;

  std::unique_ptr<NavigationLanePathGenerator> navigation_lane_path_generator_{
      nullptr};
  std::unique_ptr<NavigationLaneMapCreator> navigation_lane_map_creator_{
      nullptr};
  std::unique_ptr<lanelineprocess::LanemarkerNewDecider>
      lanemarker_new_decider_{nullptr};
  std::shared_ptr<const common::VehicleState> vehicle_state_{nullptr};
  std::shared_ptr<navigation_hdmap::MapMsg> current_map_msg_{nullptr};
  std::shared_ptr<routing::RoutingResponse> current_routing_response_{nullptr};
  TL::planning::lanelineprocess::DebounceModule laneline_rise_debounce_;
  TL::planning::lanelineprocess::DebounceModule lane_status_to_mcu_debounce_;
  functionmanager::PerceptionSubState history_perception_sub_state_{
      functionmanager::SUB_INITIAL_TYPE};
  functionmanager::NNPSysState history_nnp_state_{
      functionmanager::NNPS_PASSIVE};
  functionmanager::FctToNnpInput::NPILOT_State history_pilot_state_{
      functionmanager::FctToNnpInput::PILOT_OFF};
  std::shared_ptr<LanemarkersLaneLine> lanemarkerdebug_{nullptr};
  bool laneline_status_{false};
  bool steer_torque_to_suspend_{false};
  bool vehicle_pos_or_heading_err_{false};
};

}  // namespace planning
}  // namespace TL
