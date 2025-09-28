/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */

#pragma once

#include <algorithm>
#include <memory>
#include <string>

#include "common/configs/vehicle_config_helper.h"
#include "common/status/status.h"
#include "planning/localview/lane_line_builder/ldp_ldw_creater/lane_line_detection/lane_line_creater.h"
#include "planning/localview/lane_line_builder/ldp_ldw_creater/ldp_creater/ldp_creater.h"
#include "planning/localview/lane_line_builder/ldp_ldw_creater/ldp_ldw_base.h"
#include "planning/localview/lane_line_builder/ldp_ldw_creater/ldw_creater/ldw_creater.h"
#include "planning/localview/local_view.h"
#include "planning/proto/navigation_hdmap_config.pb.h"
#include "proto/common/vehicle_state.pb.h"

namespace TL {
namespace planning {
using TL::common::Status;

class LdpLdwCore {
 public:
  LdpLdwCore();

  /**
   * @brief module name
   */
  static std::string Name() { return "LdpLdwCore"; }

  /**
   * @brief module initialization function
   * @return initialization status
   */
  Status Init(const PerceptionMapConfig& config);
  Status Init();

  /**
   * @brief module start function
   * @return start status
   */
  Status Start();

  /**
   * @brief module stop function
   */
  void Stop();

  /**
   * @brief main logic of the navigation_hdmap module, runs periodically
   * triggered by timer.
   */
  bool Process(const std::shared_ptr<LocalView>& local_view,
               functionmanager::FunctionManagerOut* to_fct);
  void LdwLdpDebugInfo(const std::shared_ptr<LocalView>& local_view);
  double GetLeftTireDistance2Line(double line_markers_offset_last);
  double GetRightTireDistance2Line(double line_markers_offset_last);
  double LineMarkerEUation(double x, double c0, double c1, double c2,
                           double c3);

  void SetLanemarkerDebug(
      const std::shared_ptr<LanemarkersLaneLine>& lanemarkerdebug) {
    lanemarkerdebug_ = lanemarkerdebug;
  }

  void PreLanemarkers(const ::TL::perception::LaneMarkers& lane_markers,
                      double v_spd, double yaw_rate);
  void LeftPreLanemarker(const ::TL::perception::LaneMarker& ori_lane_marker,
                         double speed, double yaw_rate, double pre_time);
  void RightPreLanemarker(
      const ::TL::perception::LaneMarker& ori_lane_marker, double speed,
      double yaw_rate, double pre_time);

 private:
  // void ProcessLaneMarkers(const std::shared_ptr<LocalView>& local_view);
  void SetMg(const std::shared_ptr<LocalView>& local_view);
  common::VehicleParam vehicle_param_;
  PerceptionMapConfig navi_hdmap_config_;
  std::unique_ptr<LdwCreater> ldw_creater_{nullptr};
  std::unique_ptr<LdpCreater> ldp_creater_{nullptr};
  std::unique_ptr<LaneLineMarkerDetection> lane_line_marker_detection_{nullptr};
  std::shared_ptr<LanemarkersLaneLine> lanemarkerdebug_{nullptr};
  LdpLdwData ldp_ldw_data_;
};

}  // namespace planning
}  // namespace TL
