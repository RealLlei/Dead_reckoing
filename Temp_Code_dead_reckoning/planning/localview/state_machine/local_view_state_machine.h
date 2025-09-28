
//  Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.

#pragma once

#define BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#define BOOST_MPL_LIMIT_VECTOR_SIZE 40  // or whatever you need

#include <memory>
#include <string>
#include <boost/msm/front/functor_row.hpp>

#include "common/status/status.h"
#include "map/hdmap/hdmap.h"
#include "planning/common/dependency_injector.h"
#include "planning/localview/hdmap_avp_state/hdmap_avp_state.h"
#include "planning/localview/hdmap_lane_line_state/hdmap_lane_line_state.h"
#include "planning/localview/hdmap_perception_fused_state/hdmap_perception_fused_state.h"
#include "planning/localview/history_trace_lane_line_state/history_trace_lane_line_state.h"
#include "planning/localview/lane_line_builder/adaptive_cruise_lane_line/adaptive_cruise_lane_line.h"
#include "planning/localview/lane_line_builder/ldp_ldw_creater/ldp_ldw_core.h"
#include "planning/localview/lane_line_builder/local_hdmap_lane_line/local_hdmap_lane_line.h"
#include "planning/localview/lane_line_builder/map_fusion_lane_line/map_fusion_lane_line.h"
#include "planning/localview/lane_line_builder/navigation_hdmap_lane_line/navigation_hdmap_lane_line.h"
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/missile_mode/missile_mode_lane_line.h"
#include "planning/localview/lane_line_builder/real_hdmap_lane_line/real_hdmap_lane_line.h"
#include "planning/localview/local_view.h"
#include "planning/localview/localview_comdata_manager.h"
#include "planning/localview/parking_lane_line_state/parking_lane_line_state.h"
#include "planning/localview/perception_lane_line_state/perception_lane_line_state.h"
#include "planning/localview/state_machine/event.h"
#include "planning/localview/state_machine/initial_state.h"
#include "proto/fsm/function_manager.pb.h"

namespace TL {
namespace planning {
using TL::common::ErrorCode;  // NOLINT
using TL::common::Status;

/**
 * @class LocalViewStateFront
 * @brief local_view状态机
 */
class LocalViewStateFront
    : public boost::msm::front::state_machine_def<LocalViewStateFront> {
 public:
  using initial_state = InitialState;
  LocalViewStateFront() = default;
  ~LocalViewStateFront() = default;
  bool Init();
  bool Stop();
  void SetState(const Status& status);
  void SetLocalView(const std::shared_ptr<LocalView>& local_view);
  void SetMachineStateType(const functionmanager::MachineStateType& state_type);
  void SetHistoryMachineStateType(
      const functionmanager::MachineStateType& state_type);

  void SetPerceptionSubState(
      const functionmanager::PerceptionSubState sub_state) {
    cur_perception_sub_state_ = sub_state;
  }

  void SetHdmapSubState(const functionmanager::HdmapSubState sub_state) {
    cur_hdmap_sub_state_ = sub_state;
  }

  const common::Status& GetState() const;
  const LocalView& GetLocalView() const;
  const std::shared_ptr<LocalView>& GetMutableLocalView();
  /**
   * @brief build local view
   *
   * @param local_view local view information
   * @return common::Status errorcode and error message
   */
  common::Status BuildLocalView(const std::shared_ptr<LocalView>& local_view);

  const functionmanager::MachineStateType& GetMachineStateType() const;

  const functionmanager::PerceptionSubState& GetPerceptionSubState() const {
    return cur_perception_sub_state_;
  }

  const functionmanager::HdmapSubState& GetHdmapSubState() const {
    return cur_hdmap_sub_state_;
  }

  std::shared_ptr<functionmanager::FunctionManagerOut> GetEhpFctOut() const {
    return nnp_fct_out_;
  }

  const std::shared_ptr<LocalViewData>& GetMutableLocalViewData();

  void VehicleFunctionStateUpdata(const std::shared_ptr<LocalView>& local_view);

  const functionmanager::MachineStateType& GetHistoryMachineStateType() const;

  void APALightUpNTP(
      const std::shared_ptr<LocalView>& local_view,
      const std::shared_ptr<functionmanager::FunctionManagerOut>& to_fct);

  bool BuildNTPMap(const std::shared_ptr<LocalView>& local_view);

  bool UpdateEHPData(const std::shared_ptr<TL::ehp::EHP>& ehp_message,
                     int* received_ehp_count);
  void SetFctAndLaneLineDebug(functionmanager::FunctionManagerOut* to_fct,
                              const std::shared_ptr<LocalView>& local_view);
  void DealMapFusionAction(functionmanager::FunctionManagerOut* to_fct,
                           const std::shared_ptr<LocalView>& local_view);

  // 状态转移表
  struct transition_table
      : boost::mpl::vector<
            //    source state        event         target state      action
            // +---------------+-----------------+----------------+-----------------+

            boost::msm::front::Row<InitialState, BuildLocalViewEvent,
                                   InitialState, BuildInitialLocalViewAction,
                                   boost::msm::front::none>,
            boost::msm::front::Row<InitialState, InitialToMapEvent,
                                   HDMapLaneLineState, MapFromInitialAction,
                                   boost::msm::front::none>,
            boost::msm::front::Row<
                InitialState, InitialToPerceptionEvent, PerceptionLaneLineState,
                PerceptionFromInitialAction, boost::msm::front::none>,
            boost::msm::front::Row<
                InitialState, InitialToFusionEvent, HDMapPerceptionFusedState,
                FusionFromInitialAction, boost::msm::front::none>,
            boost::msm::front::Row<
                InitialState, InitialToParkingEvent, ParkingLanelineState,
                ParkingFromInitialAction, boost::msm::front::none>,
            boost::msm::front::Row<InitialState, InitialToHistoryTraceEvent,
                                   HistoryTraceLanelineState,
                                   HistoryTraceFromInitialAction,
                                   boost::msm::front::none>,
            boost::msm::front::Row<InitialState, InitialToHDMapAVPEvent,
                                   HDMapAVPState, HDMapAVPFromInitialAction,
                                   boost::msm::front::none>,
            boost::msm::front::Row<PerceptionLaneLineState, BuildLocalViewEvent,
                                   PerceptionLaneLineState,
                                   BuildPerceptionLocalViewAction,
                                   boost::msm::front::none>,
            boost::msm::front::Row<
                HDMapPerceptionFusedState, BuildLocalViewEvent,
                HDMapPerceptionFusedState, BuildFusedLocalViewAction,
                boost::msm::front::none>,
            boost::msm::front::Row<HDMapLaneLineState, BuildLocalViewEvent,
                                   HDMapLaneLineState, BuildMapLocalViewAction,
                                   boost::msm::front::none>,
            boost::msm::front::Row<
                ParkingLanelineState, BuildLocalViewEvent, ParkingLanelineState,
                BuildParkingLocalViewAction, boost::msm::front::none>,
            boost::msm::front::Row<
                HistoryTraceLanelineState, BuildLocalViewEvent,
                HistoryTraceLanelineState, BuildHistoryTraceLocalViewAction,
                boost::msm::front::none>,
            boost::msm::front::Row<HDMapAVPState, BuildLocalViewEvent,
                                   HDMapAVPState, BuildHDMapAVPLocalViewAction,
                                   boost::msm::front::none>,
            boost::msm::front::Row<
                PerceptionLaneLineState, PerceptionToMapWithFlagEvent,
                HDMapLaneLineState, MapWithFlagAction, boost::msm::front::none>,
            boost::msm::front::Row<HDMapLaneLineState, MapToPerceptionEvent,
                                   PerceptionLaneLineState,
                                   PerceptionFromMapAction,
                                   boost::msm::front::none>,
            boost::msm::front::Row<
                PerceptionLaneLineState, PerceptionToFusionEvent,
                HDMapPerceptionFusedState, FusionFromPerceptionAction,
                boost::msm::front::none>,
            boost::msm::front::Row<
                HDMapPerceptionFusedState, FusionToPerceptionEvent,
                PerceptionLaneLineState, PerceptionFromFusionAction,
                boost::msm::front::none>,
            boost::msm::front::Row<HDMapLaneLineState, NnpToInitialEvent,
                                   InitialState, InitialFromNnpAction,
                                   boost::msm::front::none>,
            boost::msm::front::Row<PerceptionLaneLineState, NnpToInitialEvent,
                                   InitialState, InitialFromNnpAction,
                                   boost::msm::front::none>,
            boost::msm::front::Row<HDMapPerceptionFusedState, NnpToInitialEvent,
                                   InitialState, InitialFromNnpAction,
                                   boost::msm::front::none>,
            boost::msm::front::Row<ParkingLanelineState, ParkingToInitialEvent,
                                   InitialState, InitialFromParkingAction,
                                   boost::msm::front::none>,
            boost::msm::front::Row<HistoryTraceLanelineState,
                                   HistoryTraceToInitialEvent, InitialState,
                                   InitialFromHistoryTraceAction,
                                   boost::msm::front::none>,
            boost::msm::front::Row<HDMapAVPState, HDMapAVPToInitialEvent,
                                   InitialState, InitialFromHDMapAVPAction,
                                   boost::msm::front::none>> {};

 private:
  std::shared_ptr<LocalView> local_view_ = nullptr;
  std::shared_ptr<LocalViewData> local_view_data_ = nullptr;
  Status mode_status_ = Status::OK();
  functionmanager::MachineStateType history_state_machine_ =
      functionmanager::MachineStateType::INITIAL_TYPE;
  functionmanager::MachineStateType cur_state_machine_ =
      functionmanager::MachineStateType::INITIAL_TYPE;
  //   routing::ChangeLaneType change_lane_type_fused_ =
  //       routing::ChangeLaneType::FORWARD;
  std::unique_ptr<NavigationHdmap> perception_lane_line_;
  std::unique_ptr<RealHDMapLaneLine> real_hdmap_lane_line_;
  std::unique_ptr<LocalHDMapLaneLine> local_hdmap_lane_line_;
  std::unique_ptr<MapFusionLaneLine> map_fusion_lane_line_;
  //   std::unique_ptr<ObstacleFollowingLaneLine> without_lane_line_;
  std::unique_ptr<AdaptiveCruise> adaptive_cruise_;
  std::unique_ptr<LdpLdwCore> ldw_ldp_core_{nullptr};
  std::unique_ptr<missilelane::MissileMode> missile_mode_{nullptr};
  functionmanager::PerceptionSubState cur_perception_sub_state_ =
      functionmanager::SUB_INITIAL_TYPE;
  functionmanager::HdmapSubState cur_hdmap_sub_state_ =
      functionmanager::HD_INITIAL_TYPE;
  std::shared_ptr<functionmanager::FunctionManagerOut> nnp_fct_out_{nullptr};
  functionmanager::MachineStateType previous_state_machine_ =
      functionmanager::MachineStateType::INITIAL_TYPE;
  uint64_t fsm_sequence_num_ = 0;
  TL::hdmap::AVPSlamMap map_from_slam_;
  bool is_slam_map_build_ = false;
  double distance_outof_odd_last_ = 0.0;
  double distance_downramp_last_ = 0.0;
  hdmap::RoadSection::Type road_type_last_{hdmap::RoadSection_Type_UNKNOWN};
};

using LocalViewStateMachine =
    boost::msm::back::state_machine<LocalViewStateFront>;

}  // namespace planning
}  // namespace TL
