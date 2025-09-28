/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */

#pragma once

#include <deque>
#include <list>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "common/file/log.h"
#include "common/filters/digital_filter.h"
#include "common/filters/digital_filter_coefficients.h"
#include "common/filters/mean_filter.h"
#include "common/vehicle_state/vehicle_state_provider.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/coortrans_and_copy.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/decider_data.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/lane_change_observer.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/lane_line_delay.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/lane_width_predictor.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/lanemarker_change_decider.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/lanemarker_filter.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/lanemarker_new_decider.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/lanemarker_new_decider/reset_and_quality_monitor.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/navigation_lanecentral_constructor.h"
#include "planning/localview/local_view.h"

#include "planning/proto/navigation_hdmap_config.pb.h"
#include "proto/common/vehicle_state.pb.h"
#include "proto/localization/localization.pb.h"
#include "proto/map/navigation.pb.h"
#include "proto/perception/perception_obstacle.pb.h"

/**
 * @namespace TL::navigation_hdmap
 * @brief TL::navigation_hdmap
 */
namespace TL {
namespace planning {
namespace lanelineprocess {
using TL::common::math::Vec2d;
using TL::perception::LaneMarker;
using TL::perception::LaneMarkers;
using LanePointsMap = std::unordered_map<std::string, std::vector<Vec2d>>;
using TL::functionmanager::FctToNnpInput;
using TL::soc::Chassis;

/**
 * @class LanemarkerNewDecider
 * @brief LanemarkerNewDecider generates a real-time relative map based on
 * navagation lines.
 */
class LanemarkerNewDecider {
 public:
  LanemarkerNewDecider() = default;  //  NOLINT
  explicit LanemarkerNewDecider(const planning::PerceptionMapConfig& config);
  ~LanemarkerNewDecider() = default;

  /**
   * @brief Set the configuration information required by the
   * `LanemarkerNewDecider`.
   * @param config Configuration object.
   * @return None.
   */
  TL::common::Status Init();
  /**
   * @brief set vehicle_state , localization ,histpry_sub_state
   * and perception_obstacles
   * @param localview licalview
   * @return None.
   */
  void SetDecisionMg(const std::shared_ptr<LocalView>& local_view,
                     functionmanager::FunctionManagerOut* to_fct,
                     bool laneline_status);

  bool Decision(LaneMarkers* lane_marker, LanePointsMap* lane_points_map,
                const std::shared_ptr<LocalView>& local_view,
                std::pair<bool, bool>* lane_markers_points_copy);

  LneWdeValid GetLaneWidth() { return lane_width_; }

  std::shared_ptr<LanemarkersLaneLine> GetLanemarkerDebugPtr() {
    return lanemarker_laneline_;
  }

 private:
  void DoEgolaneUpdate(DeciderData* decider_data);
  void DoNextlaneUpdate(DeciderData* decider_data);
  void LeftlaneDecider(DeciderData* decider_data);
  void RightlaneDecider(DeciderData* decider_data);
  void NLeftlaneDecider(DeciderData* decider_data);
  void NRightlaneDecider(DeciderData* decider_data);
  void LogLanemarkerErr(const LaneMarkers& lane_marker,
                        const LanePointsMap& lane_points);
  static bool GetMeasurePoints(const LaneMarker& lanemarker,
                               const std::vector<Vec2d>& filter_points,
                               std::vector<Vec2d>* lanemarker_points);
  void TranslatedPoints(std::vector<Vec2d>* lanemarker_points);
  static Vec2d PointEarth2Bus(const common::PointENU& point,
                              const common::Pose& pose);
  bool LeftLaneLineJump(const DeciderData& decider_data) const;
  bool RightLaneLineJump(const DeciderData& decider_data) const;
  static std::pair<bool, bool> LineStartIsGood(double left_start,
                                               double right_start);
  /**
   * @brief Judge whether in lane changing state
   *
   * @param decider_data
   */
  void IsLaneChange(DeciderData* decider_data);
  /**
   * @brief Judge all lanemerkers State
   *
   * @param decider_data
   */
  void JudgeLaneMarkersState(DeciderData* decider_data);
  /**
   * @brief Judge single lanemerker State, next lanemarker no need rise_time and
   * down_time for changlane
   *
   * @param lane_marker
   * @param state 1.bad state 2.good state 3.better state
   * @param time 0:rise time 1:down time
   * @param rise_time lanemarker input time
   * @param dowm_time lanemarker output time
   */
  void JudgeLaneMarkerState(const LaneMarker& lane_marker,
                            LaneMarkerState* state,
                            std::pair<double, double>* time,
                            double rise_time = 0.0, double down_time = 0.0,
                            bool is_jump_line = false,
                            bool line_start_is_good = true);
  void OutDebugInfo(DeciderData* decider_data);
  /**
   * @brief 将原始的车道线信息预推一段时间以应对视觉等带来的延迟
   *
   * @param decider_data
   * @param lane_markers 原始车道线
   * @param v_spd speed
   * @param yaw_rate 车辆横摆角速度，滤波后的
   */
  void PreLanemarkers(DeciderData* decider_data,
                      const LaneMarkers& lane_markers, double v_spd,
                      double yaw_rate);
  /**
   * @brief 单个车道线处理
   *
   * @param decider_data
   * @param ori_lane_marker 原始车道线
   * @param pre_lanemarker 预推后的车道线
   * @param speed
   * @param yaw_rate 车辆横摆角速度，滤波后的
   * @param pre_time 预推时间
   */
  static void PreLanemarker(const LaneMarker& ori_lane_marker,
                            LaneMarker* pre_lanemarker, double speed,
                            double yaw_rate, double pre_time);
  void DeciderObsBeforeVehicle(const LaneMarkers& lane_markers);
  static double CalculateObsY(const LaneMarker& lane_marker, double obs_x,
                              double obs_y);
  // the configuration information required by the `LanemarkerNewDecider`
  const planning::PerceptionMapConfig config_;
  double better_quality_threshold_ = 0.6;
  double good_quality_threshold_ = 0.3;
  double main_loop_time_ = 0.1;

  std::shared_ptr<const common::VehicleState> vehicle_state_{nullptr};
  std::shared_ptr<const localization::Localization> localization_{nullptr};
  std::shared_ptr<const perception::PerceptionObstacles> perception_obstacles_{
      nullptr};
  lanelineprocess::DeciderData decider_data_;
  std::unique_ptr<CoorTransAndCopy> coortrans_and_copy_;
  std::unique_ptr<LaneChangeDecider> lanechange_decider_;
  std::unique_ptr<LaneWidthPredictor> lanewidth_predictor_;
  std::unique_ptr<LaneMarkerFilter> lanemarker_filter_;
  std::unique_ptr<ResetAndQualityMonitor> reset_and_quality_monitor_;
  TL::planning::lanelineprocess::LaneChangeObserver lanechange_observer_;
  TL::common::DigitalFilter speed_digital_filter_;
  TL::common::DigitalFilter yawrate_digital_filter_;
  LaneMarkersState lane_markers_state_{BAD_LANEMARKER, BAD_LANEMARKER,
                                       BAD_LANEMARKER, BAD_LANEMARKER};
  std::array<std::pair<double, double>, 4> lane_markers_state_limit_;
  LneWdeValid lane_width_;
  std::shared_ptr<LanemarkersLaneLine> lanemarker_laneline_;
  functionmanager::PerceptionSubState history_perception_sub_state_{
      functionmanager::SUB_INITIAL_TYPE};
  functionmanager::NNPSysState history_nnp_state_{
      functionmanager::NNPS_PASSIVE};
  FctToNnpInput::NPILOT_State history_pilot_state_{FctToNnpInput::PILOT_OFF};
  FctToNnpInput::ADCS8_ACCState history_acc_sytate_{FctToNnpInput::ACC_OFF};
  Chassis::DrivingMode history_drive_mode_{Chassis::COMPLETE_MANUAL};
  bool has_obs_flag_{false};
  bool history_laneline_status_{false};
  bool history_noline_status_{false};
  bool is_nolane_ok_{false};
  double left_line_c0_old_{0.0};
  double right_line_c0_old_{0.0};
};
}  // namespace lanelineprocess
}  // namespace planning
}  // namespace TL
