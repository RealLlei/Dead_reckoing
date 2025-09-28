/*
 * Copyright (c) TL auto Co., Ltd. 2022-2023. All rights reserved.
 */
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include "common/file/log.h"
#include "common/math/double_type.h"
#include "common/math/vec2d.h"
#include "common/util/macros.h"
#include "map/hdmap/hdmap_common.h"
#include "planning/common/planning_gflags.h"
#include "planning/localview/lane_line_builder/real_hdmap_lane_line/failure_location_checker.h"
#include "proto/fsm/function_manager.pb.h"

namespace TL {
namespace planning {
namespace {
constexpr uint64_t kLocationIsFault = 12;
constexpr uint64_t kLocationFaultStart = 101;
constexpr uint64_t kLocationFatalFault = 111;
constexpr uint64_t kLocationSeriouslFault = 121;
constexpr uint64_t kLocationNormalFault = 151;
constexpr int kErrStep = 21;
constexpr int kShiftStep = 11;
constexpr int kInitToShiftCount = 10;
constexpr int kInitToErrCount = 20;
constexpr int kToInitCount = 50;
constexpr int kErrToInitCount = 350;
constexpr int kShiftToErrCount = 3;
constexpr double kLocationXerrMin = 2.0;
constexpr double kLocationYerrMin = 0.6;
constexpr double kLanemarkerMinJumpErr = 1.5;
constexpr double kRiseTime = 0.5;
constexpr double kDownTime = 2.0;
constexpr double kMainLoopTime = 0.1;
constexpr double kMinDisToDownRamp = 500;
constexpr double kMinSplitMergeDis = 200;
constexpr double kInitWithDiff = 0.25;
// lane map matching check
constexpr double kMaxLanemarkerTm = 2.1;
constexpr double kValLanemarkerTm = 1.1;
constexpr double kBetterLanemarkerTm = 0.6;
constexpr double kDeltaTm = 0.1;
constexpr double kCheckErr = 0.12;
constexpr double kValErr = 0.28;
constexpr double kMaxErr = 0.4;
constexpr double kMinDeltaTm = 0.3;

const double kSegmentationEpsilon = 0.2;
const std::size_t kLanePointMinSize = 2U;

void RemoveDuplicates(std::vector<Vec2d>* points) {
  RETURN_IF_NULL(points);

  size_t points_size = points->size();

  if (points_size <= kLanePointMinSize) {
    return;
  }

  int count = 0;
  // 3个条件：
  // 1.第1个点和最后一个点必要
  // 2.中间点距离>limit
  // 3.最后两个点如果距离太近只要最后一个点
  for (size_t i = 0; i < points_size; ++i) {
    if (count == 0 || i == points_size - 1) {
      (*points)[count++] = points->at(i);
    } else {
      if (points->at(i).DistanceTo(points->at(count - 1)) >
          kSegmentationEpsilon) {
        (*points)[count++] = points->at(i);
      }
    }
  }
  if (count > 2) {
    if (points->at(count - 2).DistanceTo(points->at(count - 1)) <=
        kSegmentationEpsilon) {
      points->at(count - 2) = points->at(count - 1);
      --count;
    }
  }
  points->resize(count);
}
}  // namespace

using TL::functionmanager::NNPSysState;

void FailureLocationChecker::Init() {
  per_lanechange_observer_.Init();
  map_lanechange_observer_.Init();
  left_lanemarker_quality_debounce_.ResetTime(kRiseTime, kDownTime,
                                              kMainLoopTime);
  right_lanemarker_quality_debounce_.ResetTime(kRiseTime, kDownTime,
                                               kMainLoopTime);
  left_lanemarker_viewrange_debounce_.ResetTime(kRiseTime, kDownTime,
                                                kMainLoopTime);
  right_lanemarker_viewrange_debounce_.ResetTime(kRiseTime, kDownTime,
                                                 kMainLoopTime);
  is_in_main_road_debounce_.ResetTime(5.0, 0.0, kMainLoopTime);
  is_good_lanemarker_debounce_.ResetTime(1.0, 3.0, kMainLoopTime);
  is_good_init_lanemarker_debounce_.ResetTime(2.0, 0.0, kMainLoopTime);
  is_in_overlaplane_debounce_.ResetTime(0.3, 0.0, 0.1);
}

void FailureLocationChecker::Process(
    const std::shared_ptr<LocalView>& local_view,
    const std::shared_ptr<TL::hdmap::PncMap>& pnc_map,
    const std::shared_ptr<TL::hdmap::HDMap>& hdmap,
    functionmanager::FunctionManagerOut* const to_fct) {
  vehicle_state_ = local_view->GetVehicleState();
  is_change_mode_by_odd_type_ = to_fct->has_nnp_fct_out()
                                    ? to_fct->nnp_fct_out()
                                          .nnp_statechange_conditions()
                                          .is_change_mode_by_odd_type()
                                    : false;
  bool not_in_main_road = true;
  // loc not checker in ramp、dis_down_ramp < 1000、split lane、merge lane
  if (pnc_map != nullptr) {
    const auto dis_down_ramp =
        pnc_map
            ->GetFrontLaneRangeInfo(std::numeric_limits<double>::max(),
                                    TL::hdmap::LaneType::RAMP)
            .start_s;
    auto adc_lane = pnc_map->GetADCWaypoint().lane;
    err_not_main_road_ = GetErrStatebl(pnc_map, hdmap);
    to_fct->mutable_real_hdmap_debug()->set_err_not_main_road(
        err_not_main_road_);
    const auto lane_transition =
        (adc_lane != nullptr && adc_lane->lane().has_lane_transition())
            ? adc_lane->lane().lane_transition()
            : TL::hdmap::Lane_LaneTransition_UNKONOW;
    int predecessor_lane_id_size = 0;
    double check_length = kMinSplitMergeDis + 1;
    if (adc_lane != nullptr) {
      check_length = adc_lane->total_length() - pnc_map->GetADCWaypoint().s;
      predecessor_lane_id_size =
          std::max(adc_lane->lane().successor_id_size(),
                   adc_lane->lane().predecessor_id_size());
    }
    while (check_length < kMinSplitMergeDis) {
      if (adc_lane != nullptr && adc_lane->lane().successor_id_size() == 1 &&
          predecessor_lane_id_size < 2) {
        const auto success_lane =
            hdmap->GetLaneById(adc_lane->lane().successor_id(0));
        if (success_lane != nullptr) {
          predecessor_lane_id_size =
              std::max(success_lane->lane().predecessor_id_size(),
                       success_lane->lane().successor_id_size());
          adc_lane = success_lane;
          check_length += success_lane->total_length();
        } else {
          predecessor_lane_id_size = 0;
          break;
        }
      } else {
        break;
      }
    }
    ADEBUG << "dis_down_ramp: " << dis_down_ramp
           << " , lane_transition: " << lane_transition
           << " , predecessor_lane_id_size: " << predecessor_lane_id_size;
    ADEBUG << "ADCIsInLaneSplit: " << pnc_map->ADCIsInLaneSplit()
           << " , IsInOverLapLane size: " << pnc_map->IsInOverLapLane()
           << " , pnc_map->AdcInMainRoad(): " << pnc_map->AdcInMainRoad();
    auto is_in_over_lap =
        is_in_overlaplane_debounce_.DealDebounce(pnc_map->IsInOverLapLane());
    // const bool is_down_ramp =
    //     dis_down_ramp > 0.1 && dis_down_ramp < kMinDisToDownRamp;
    not_in_main_road = !is_in_main_road_debounce_.DealDebounce(
        pnc_map->AdcInMainRoad() &&
        !(dis_down_ramp > 0.1 &&                 // NOLINT
          dis_down_ramp < kMinDisToDownRamp) &&  // NOLINT
        lane_transition == TL::hdmap::Lane_LaneTransition_CONTINUE &&
        !pnc_map->ADCIsInLaneSplit() && predecessor_lane_id_size < 2 &&
        !(to_fct->fsm_state() == TL::functionmanager::PERCEPTION_TYPE
              ? false
              : is_in_over_lap));
  }
  TL::common::Pose pose;
  if (local_view->HasLocalization() &&
      local_view->GetLocalization()->has_pose()) {
    pose = local_view->GetLocalization()->pose();
  } else {
    pose.set_heading(0.0);
    pose.mutable_position()->set_x(0.0);
    pose.mutable_position()->set_y(0.0);
  }

  bool is_bad_lanemarkers = true;
  if (local_view->HasLaneMarkers()) {
    lanemarkers_ = *local_view->GetLaneMarkers();
    is_bad_lanemarkers = DeciderLaneMarkerAndWidth(pose);
  }

  int ori_location_err_state = LocationErrDecider(local_view, to_fct);

  bool match_prerequisite =
      !is_bad_lanemarkers && !not_in_main_road && ori_location_err_state == 0;
  bool map_checker_bl = MapLaneChecker(pnc_map, hdmap, to_fct);
  to_fct->mutable_real_hdmap_debug()->mutable_map_checker_debug()->set_is_match(
      map_checker_bl);
  bool is_match =
      MatchingLaneAndMap(pnc_map, pose, hdmap, to_fct, match_prerequisite) &&
      map_checker_bl;
  ADEBUG << " *****map_checker_bl******* = " << map_checker_bl
         << " , is_match: " << is_match;
  double left_width = 0.0;
  double right_width = 0.0;
  double s = 0.0;
  // 定位点在车道中心线左边l为正，右边l为负
  double l = 0.0;
  bool has_no_width_diff = false;
  double left_heading_diff{0.0};
  double right_heading_diff{0.0};
  std::pair<double, double> width_diff = std::make_pair(0.0, 0.0);
  if (pnc_map != nullptr && pnc_map->GetADCWaypoint().lane != nullptr) {
    const auto adc_lane = pnc_map->GetADCWaypoint().lane;
    s = pnc_map->GetADCWaypoint().s;
    l = pnc_map->GetADCWaypoint().l;
    adc_lane->GetWidth(s, &left_width, &right_width);
    const auto map_heading = adc_lane->Heading(s);
    const auto map_veh_heading = map_heading - pose.heading();
    const auto left_lane_c1 =
        std::atan(lanemarkers_.front_left_lane_marker().c1_heading_angle());
    const auto right_lane_c1 =
        std::atan(lanemarkers_.front_right_lane_marker().c1_heading_angle());
    left_heading_diff = (map_veh_heading - left_lane_c1) / M_PI * 360;
    right_heading_diff = (map_veh_heading - right_lane_c1) / M_PI * 360;
    left_map_c0_ = left_width - l;
    right_map_c0_ = -(right_width + l);
    width_diff = CalculateValidLocErrHasMap(adc_lane);
    const auto is_lane_change_per =
        per_lanechange_observer_.Observer(lanemarkers_);
    const auto is_lane_change_map =
        map_lanechange_observer_.Observer(GenerateVirtualMapLanemarkers());
    LanechangeRiseDecider(is_lane_change_per, is_lane_change_map);
    // AERROR << "left_map_c0: " << left_map_c0_
    //        << " , right_map_c0: " << right_map_c0_ << " , left_lane_c0: "
    //        << lanemarkers_.front_left_lane_marker().c0_position()
    //        << " , right lane_c0: "
    //        << lanemarkers_.front_right_lane_marker().c0_position();
    // AERROR << "width_diff left: " << width_diff.first
    //        << " , width_diff right: " << width_diff.second;
    // AERROR << " , is_lane_change_per left: " << is_lane_change_per.first
    //        << " , is_lane_change_per right: " << is_lane_change_per.second
    //        << " , is_lane_change_map left: " << is_lane_change_map.first
    //        << " , is_lane_change_map right: " << is_lane_change_map.second
    //        << " , is_lane_change: " << is_lane_change_;

  } else {
    has_no_width_diff = true;
  }
  ADEBUG << "ori_location_err_state = " << ori_location_err_state
         << ", is_bad_lanemarkers = " << is_bad_lanemarkers
         << ", has_no_width_diff = " << has_no_width_diff;
  if (is_lane_change_) {
    width_diff.first = width_diff_history_.first;
    width_diff.second = width_diff_history_.second;
  }

  auto* loc_err_debug =
      to_fct->mutable_real_hdmap_debug()->mutable_location_err_debug();
  // loc_err_debug->set_loc_err_reason("loc err normal and has map");
  loc_err_debug->set_left_lanemarker_err(width_diff.first);    // NOLINT
  loc_err_debug->set_right_lanemarker_err(width_diff.second);  // NOLINT
  loc_err_debug->set_is_failure_location(ori_location_err_state);
  loc_err_debug->set_is_bad_lanemarkers(is_bad_lanemarkers);
  loc_err_debug->set_not_in_main_road(not_in_main_road);
  loc_err_debug->set_left_heading_err(left_heading_diff);    // NOLINT
  loc_err_debug->set_right_heading_err(right_heading_diff);  // NOLINT
  ADEBUG << "left_width = " << left_width << ", right_width = " << right_width
         << ", s = " << s << ", l = " << l;
  ADEBUG << "left_c0 = " << lanemarkers_.front_left_lane_marker().c0_position()
         << ", left_map_c0 = " << left_map_c0_ << ", right_c0 = "
         << lanemarkers_.front_right_lane_marker().c0_position()
         << ", right_map_c0 = " << right_map_c0_;
  ADEBUG << "left_lane_err = " << width_diff.first
         << ", right_lane_err = " << width_diff.second;
  // 仅仅使用定位自己的故障进行降级
  if (FLAGS_only_using_locationself_err) {
    width_diff.first = 0.0;
    width_diff.second = 0.0;
    is_bad_lanemarkers = false;
    not_in_main_road = false;
    has_no_width_diff = false;
    left_map_c0_ = kDfaultLaneWidth;
    right_map_c0_ = kDfaultLaneWidth;
  }
  // 地图一直未消失的定位失效和恢复判断
  DealPerceptionLocErr(width_diff, is_bad_lanemarkers, has_no_width_diff,
                       not_in_main_road, is_match);
  switch (loc_err_state_) {
    case 0:
      if (ori_location_err_state == 2 ||
          (ori_location_err_state == 1 &&
           (not_in_main_road || is_bad_lanemarkers || has_no_width_diff)) ||
          per_loc_err_state_ == LocErrState::Err) {
        loc_err_state_ = 2;
      } else if (ori_location_err_state == 1 ||
                 per_loc_err_state_ == LocErrState::Shift) {
        loc_err_state_ = 1;
      }
      break;
    case 1:
      if (ori_location_err_state == 2 ||
          per_loc_err_state_ == LocErrState::Err || not_in_main_road ||
          is_bad_lanemarkers || has_no_width_diff) {
        loc_err_state_ = 2;
      } else if (ori_location_err_state == 0 &&
                 per_loc_err_state_ == LocErrState::Loc_Init) {
        loc_err_state_ = 0;
      }
      break;
    case 2:
      if (ori_location_err_state == 0 &&
          per_loc_err_state_ == LocErrState::Loc_Init) {
        loc_err_state_ = 0;
      }
      break;
    default:
      break;
  }
  loc_err_debug->set_perception_err_state(per_loc_err_state_);
  width_diff_history_ = width_diff;
  auto loc_err_state = (FLAGS_enable_odd_area_internal_to_perception &&
                        FLAGS_enable_hdmap_nnp_mode)
                           ? loc_err_state_
                           : 0;
  // 将定位问题都变成内部降级，没有故障外部退出
  // loc_err_state = loc_err_state != LocErrState::Loc_Init
  //                     ? LocErrState::Shift
  //                     : LocErrState::Loc_Init;
  ADEBUG << "LOC_ERR_STATE: " << ori_location_err_state
         << ", perception loc_err: " << per_loc_err_state_
         << " , loc_pose_step_jump: " << is_loc_pose_jump_
         << " , is_match: " << is_match << " , loc_err: " << loc_err_state;
  loc_err_debug->set_hasmap_err_state(loc_err_state);
  to_fct->mutable_nnp_fct_out()
      ->mutable_nnp_statechange_conditions()
      ->set_location_err_state(loc_err_state);
  ADEBUG << "---------loc_err_state----- " << loc_err_state_;
}

void FailureLocationChecker::DealPerceptionLocErr(
    const std::pair<double, double>& width_diff, bool is_bad_lanemarkers,
    bool has_no_width_diff, bool not_in_main_road, bool is_match) {
  // 地图一直未消失的定位失效和恢复判断
  switch (per_loc_err_state_) {
    case LocErrState::Loc_Init:
      DealLocInitState(width_diff, is_bad_lanemarkers, has_no_width_diff,
                       not_in_main_road, is_match);
      break;
    case LocErrState::Shift:
      DealLocShiftState(width_diff, is_bad_lanemarkers, has_no_width_diff,
                        not_in_main_road, is_match);
      break;
    case LocErrState::Err:
      if (is_bad_lanemarkers || has_no_width_diff || not_in_main_road) {
        err_state_count_++;
        width_same_count_ = 0;
      } else if (std::fabs(width_diff.first) < kInitWithDiff &&
                 std::fabs(width_diff.second) < kInitWithDiff && is_match) {
        width_same_count_++;
        err_state_count_ = 0;
      } else {
        width_same_count_ = 0;
        err_state_count_ = 0;
      }
      if (err_state_count_ > kErrToInitCount ||
          width_same_count_ > kToInitCount) {
        per_loc_err_state_ = LocErrState::Loc_Init;
        left_map_c0_ = kDfaultLaneWidth;
        right_map_c0_ = kDfaultLaneWidth;
        width_diff_count_ = 0;
        width_same_count_ = 0;
        err_state_count_ = 0;
      }
      break;
    default:
      break;
  }
}

bool FailureLocationChecker::MatchingLaneAndMap(
    const std::shared_ptr<TL::hdmap::PncMap>& pnc_map,
    const common::Pose& pose, const std::shared_ptr<TL::hdmap::HDMap>& hdmap,
    functionmanager::FunctionManagerOut* const to_fct,
    bool match_prerequisite) {
  RETURN_VAL_IF_NULL(to_fct, false);
  if (pnc_map == nullptr || pnc_map->GetADCWaypoint().lane == nullptr) {
    ResetMatchState();
    return true;
  }

  auto* lanemap_debug =
      to_fct->mutable_real_hdmap_debug()->mutable_maplane_checker();
  std::vector<Vec2d> left_map_bound_points;
  std::vector<Vec2d> right_map_bound_points;
  const auto speed = std::max(3.0, vehicle_state_->linear_velocity());
  double max_lanemarker_dis = speed * kMaxLanemarkerTm;
  double val_dis = speed * kValLanemarkerTm;
  double check_dis = speed * kBetterLanemarkerTm;
  double delta_dis = speed * kDeltaTm;
  ADEBUG << "max_lanemarker_dis: " << max_lanemarker_dis
         << " , val_dis: " << val_dis << " , check_dis: " << check_dis
         << ", delta_dis: " << delta_dis;
  ADEBUG << "-------------start-----------";
  lanemap_debug->set_max_lanemarker_dis(max_lanemarker_dis);
  lanemap_debug->set_val_dis(val_dis);
  lanemap_debug->set_checker_dis(check_dis);

  const auto& left_lanemarker = lanemarkers_.front_left_lane_marker();
  const auto& right_lanemarker = lanemarkers_.front_right_lane_marker();
  double view_range =
      std::max(left_lanemarker.view_range(), right_lanemarker.view_range());
  auto adc_lane = pnc_map->GetADCWaypoint().lane;
  const double start_s = pnc_map->GetADCWaypoint().s;
  double lane_length = adc_lane->total_length() - start_s;
  ADEBUG << "lane_length: " << lane_length << " , view_range: " << view_range
         << " , lane id: " << adc_lane->lane().id().id();
  AddMapPoints(&left_map_bound_points, &right_map_bound_points, adc_lane, pose);
  int add_lane_size = 1;
  while (lane_length < view_range) {
    if (adc_lane->lane().successor_id().empty()) {
      break;
    }
    if (adc_lane->lane().successor_id().size() > 1) {
      ResetMatchState();
      return true;
    }
    auto lane_id = adc_lane->lane().successor_id().at(0);
    ADEBUG << " success lane id: " << lane_id.id();
    adc_lane = hdmap->GetLaneById(lane_id);
    if (adc_lane == nullptr) {
      break;
    }
    AddMapPoints(&left_map_bound_points, &right_map_bound_points, adc_lane,
                 pose);
    lane_length += adc_lane->total_length();
    ADEBUG << "add lane length:" << adc_lane->total_length()
           << " , total lane length: " << lane_length;
    add_lane_size++;
  }
  ADEBUG << "left_point_size: " << left_map_bound_points.size()
         << " , add_lane_size: " << add_lane_size;

  if (left_map_bound_points.size() < 2 || right_map_bound_points.size() < 2) {
    ResetMatchState();
    return true;
  }
  std::vector<Vec2d> left_bus_bound_points =
      InterPolateVec2dPoints(left_map_bound_points, 0.0, view_range, delta_dis);
  std::vector<Vec2d> right_bus_bound_points = InterPolateVec2dPoints(
      right_map_bound_points, 0.0, view_range, delta_dis);

  if (left_bus_bound_points.size() < 2 || right_bus_bound_points.size() < 2) {
    ResetMatchState();
    return true;
  }

  const auto left_dis = CheckerDistance(left_bus_bound_points, left_lanemarker);
  const auto right_dis =
      CheckerDistance(right_bus_bound_points, right_lanemarker);

  // speed * kMinDeltaTm 容忍车道线最多比需求的短0.3s
  const bool left_match_prerequisite =
      match_prerequisite && speed > 10.0 &&
      left_lanemarker.view_range() + speed * kMinDeltaTm > max_lanemarker_dis &&
      left_map_bound_points.back().x() > left_lanemarker.view_range();
  const bool right_match_prerequisite =
      match_prerequisite && speed > 10.0 &&
      right_lanemarker.view_range() + speed * kMinDeltaTm >
          max_lanemarker_dis &&
      right_map_bound_points.back().x() > right_lanemarker.view_range();
  ADEBUG << "left_match_prerequisite: " << left_match_prerequisite
         << " , right_match_prerequisite: " << right_match_prerequisite
         << " , match_prerequisite: " << match_prerequisite;
  // left match state
  if (left_match_state_ == 0) {
    if (left_match_prerequisite && std::get<0>(left_dis) > check_dis &&
        std::get<1>(left_dis) < max_lanemarker_dis) {
      left_match_state_ = 1;
    }
  } else if (left_match_state_ == 1) {
    left_match_state_count_++;
    if (std::get<0>(left_dis) < check_dis) {
      left_check_dis_count_++;
    }
    ADEBUG << " left_match_state_count: " << left_match_state_count_;
    if (!left_match_prerequisite || left_check_dis_count_ > 3 ||
        std::get<1>(left_dis) > max_lanemarker_dis ||
        std::get<1>(left_dis) <
            max_lanemarker_dis - (left_match_state_count_ + 3) * delta_dis) {
      left_check_dis_count_ = 0;
      left_match_state_count_ = 0;
      left_match_state_ = 0;
    } else if (left_match_prerequisite && left_check_dis_count_ <= 3 &&
               std::get<1>(left_dis) < val_dis && left_match_state_count_ > 3) {
      left_check_dis_count_ = 0;
      left_match_state_count_ = 0;
      left_match_state_ = 2;
    }
  } else if (left_match_state_ == 2) {
    left_match_state_count_++;
    if ((std::get<0>(left_dis) > check_dis &&
         std::get<1>(left_dis) > max_lanemarker_dis) ||
        !left_match_prerequisite || left_match_state_count_ > 50) {
      left_match_state_count_ = 0;
      left_match_state_ = 0;
    }
  }
  // right match state
  if (right_match_state_ == 0) {
    if (right_match_prerequisite && std::get<0>(right_dis) > check_dis &&
        std::get<1>(right_dis) < max_lanemarker_dis) {
      right_match_state_ = 1;
    }
  } else if (right_match_state_ == 1) {
    if (std::get<0>(right_dis) < check_dis) {
      right_check_dis_count_++;
    }
    right_match_state_count_++;
    ADEBUG << " right_match_state_count: " << right_match_state_count_;
    if (!right_match_prerequisite || right_check_dis_count_ > 3 ||
        std::get<1>(right_dis) > max_lanemarker_dis ||
        std::get<1>(right_dis) <
            max_lanemarker_dis - (right_match_state_count_ + 3) * delta_dis) {
      right_check_dis_count_ = 0;
      right_match_state_count_ = 0;
      right_match_state_ = 0;
    } else if (right_match_prerequisite && right_check_dis_count_ <= 3 &&
               std::get<1>(right_dis) < val_dis &&
               right_match_state_count_ > 3) {
      right_check_dis_count_ = 0;
      right_match_state_count_ = 0;
      right_match_state_ = 2;
    }
  } else if (right_match_state_ == 2) {
    right_match_state_count_++;
    if ((std::get<0>(right_dis) > check_dis &&
         std::get<1>(right_dis) > max_lanemarker_dis) ||
        !right_match_prerequisite || right_match_state_count_ > 50) {
      right_match_state_count_ = 0;
      right_match_state_ = 0;
    }
  }

  auto* left_debug = lanemap_debug->mutable_left_check();
  left_debug->set_match_state(left_match_state_);
  left_debug->set_match_state_dis_count(left_check_dis_count_);
  left_debug->set_match_x(std::get<0>(left_dis));
  left_debug->set_diff_x(std::get<1>(left_dis));
  auto* right_debug = lanemap_debug->mutable_right_check();
  right_debug->set_match_state(right_match_state_);
  right_debug->set_match_state_dis_count(right_check_dis_count_);
  right_debug->set_match_x(std::get<0>(right_dis));
  right_debug->set_diff_x(std::get<1>(right_dis));

  ADEBUG << "--------left_match_state: " << left_match_state_
         << " , right_match_state: " << right_match_state_;
  return left_match_state_ != 2 && right_match_state_ != 2;
}

bool FailureLocationChecker::GetErrStatebl(
    const std::shared_ptr<TL::hdmap::PncMap>& pnc_map,
    const std::shared_ptr<TL::hdmap::HDMap>& hdmap) {
  constexpr double checker_err_length = 300.0;
  auto adc_lane = pnc_map->GetADCWaypoint().lane;
  const auto& passage_lane_ids = pnc_map->GetAdcPassageAllLaneId();
  if (passage_lane_ids.empty() || hdmap == nullptr || adc_lane == nullptr) {
    return false;
  }
  auto lane_transition = adc_lane->lane().has_lane_transition()
                             ? adc_lane->lane().lane_transition()
                             : TL::hdmap::Lane_LaneTransition_UNKONOW;
  auto is_main_road = adc_lane->IsMainRoad();
  if (lane_transition == TL::hdmap::Lane_LaneTransition_SPLITING ||
      !is_main_road) {
    return true;
  }
  double lane_length = adc_lane->total_length() - pnc_map->GetADCWaypoint().s;
  if (adc_lane->id().id() != passage_lane_ids.front() ||
      lane_length > checker_err_length) {
    return false;
  }
  TL::hdmap::Id lane_id;
  for (const auto& id : passage_lane_ids) {
    if (id != adc_lane->id().id()) {
      lane_id.set_id(id);
      const auto& lane = hdmap->GetLaneById(lane_id);
      if (lane == nullptr) {
        return false;
      }
      lane_transition = lane->lane().has_lane_transition()
                            ? lane->lane().lane_transition()
                            : TL::hdmap::Lane_LaneTransition_UNKONOW;
      if (lane_transition == TL::hdmap::Lane_LaneTransition_SPLITING ||
          !lane->IsMainRoad()) {
        return true;
      }
      lane_length += lane->total_length();
      if (lane_length > checker_err_length) {
        return false;
      }
    }
  }
  return false;
}

bool FailureLocationChecker::MapLaneChecker(  // NOLINT
    const std::shared_ptr<TL::hdmap::PncMap>& pnc_map,
    const std::shared_ptr<TL::hdmap::HDMap>& hdmap,
    functionmanager::FunctionManagerOut* to_fct) {
  constexpr double break_length = 150.0;
  auto adc_lane = pnc_map->GetADCWaypoint().lane;
  const auto& passage_lane_ids = pnc_map->GetAdcPassageAllLaneId();
  if (passage_lane_ids.empty() || hdmap == nullptr || adc_lane == nullptr) {
    return true;
  }

  auto lane_curvature_type = adc_lane->lane().curvature_type();
  if (lane_curvature_type == TL::hdmap::Lane_LaneCurvature_ABNORMAL) {
    auto* debug =
        to_fct->mutable_real_hdmap_debug()->mutable_map_checker_debug();
    debug->set_id(adc_lane->id().id());
    debug->set_curvature_type(lane_curvature_type);
    return false;
  }
  double lane_length = adc_lane->total_length() - pnc_map->GetADCWaypoint().s;
  if (adc_lane->id().id() != passage_lane_ids.front() ||
      lane_length > break_length) {
    return true;
  }
  TL::hdmap::Id lane_id;
  for (const auto& id : passage_lane_ids) {
    if (id != adc_lane->id().id()) {
      lane_id.set_id(id);
      const auto& lane = hdmap->GetLaneById(lane_id);
      if (lane == nullptr) {
        return true;
      }
      if (lane->lane().curvature_type() ==
          TL::hdmap::Lane_LaneCurvature_ABNORMAL) {
        auto* debug =
            to_fct->mutable_real_hdmap_debug()->mutable_map_checker_debug();
        debug->set_id(adc_lane->id().id());
        debug->set_curvature_type(lane_curvature_type);
        return false;
      }
      lane_length += lane->total_length();
      if (lane_length > break_length) {
        return true;
      }
    }
  }
  return true;
}

std::tuple<double, double, double> FailureLocationChecker::CheckerDistance(
    const std::vector<Vec2d>& map_bound_points,
    const TL::perception::LaneMarker& lane_marker) {
  int index = 0;
  double left_map_match_x = map_bound_points.back().x();
  bool is_check_match = true;
  bool is_check_diff = true;
  bool is_checker_recorve = true;
  double left_diff_x = map_bound_points.back().x();
  double left_check_start_x = map_bound_points.back().x();
  // double left_check_recorve_x = map_bound_points.back().x();
  for (uint64_t i = 0; i < map_bound_points.size() - 1; i++) {
    auto bus_point = map_bound_points[i];
    auto y = CalculateLanemarkerY(bus_point.x(), lane_marker);
    double delta_y = std::fabs(bus_point.y() - y);
    ADEBUG << " left_bus_bound_points[" << index << "] x: " << bus_point.x()
           << " , map_y: " << bus_point.y() << " , left_lane_y: " << y
           << " , left_delta_y: " << delta_y;
    if (delta_y > kCheckErr && is_check_match) {
      is_check_match = false;
      left_map_match_x = bus_point.x();
    }
    if (delta_y > kValErr && is_check_diff) {
      is_check_diff = false;
      left_diff_x = bus_point.x();
    }
    if (delta_y > 0.3 && is_checker_recorve) {
      is_checker_recorve = false;
      // left_check_recorve_x = bus_point.x();
    }
    if (delta_y > kMaxErr) {
      left_check_start_x = bus_point.x();
      break;
    }
    index++;
  }
  ADEBUG << "left_map_match_x: " << left_map_match_x
         << " , left_diff_x: " << left_diff_x
         << " , left_check_start_x: " << left_check_start_x;
  return {left_map_match_x, left_diff_x, left_check_start_x};
}

void FailureLocationChecker::AddMapPoints(
    std::vector<Vec2d>* const left_map_points,
    std::vector<Vec2d>* const right_map_points,
    const TL::hdmap::LaneInfoConstPtr& lane, const common::Pose& pose) {
  RETURN_IF_NULL(lane);
  TL::common::PointENU prev_p;
  prev_p.set_x(std::numeric_limits<double>::infinity());
  prev_p.set_y(std::numeric_limits<double>::infinity());
  for (const auto& seg : lane->lane().left_boundary().curve().segment()) {
    if (!seg.has_line_segment()) {
      continue;
    }
    for (const auto& p : seg.line_segment().point()) {
      if (!std::isinf(prev_p.x()) && !std::isinf(prev_p.y()) &&
          common::math::double_type::SeemsEqual(prev_p.x(), p.x()) &&
          common::math::double_type::SeemsEqual(prev_p.y(), p.y())) {
        continue;
      }
      left_map_points->emplace_back(PointEarth2Bus({p.x(), p.y()}, pose));
      prev_p.set_x(p.x());
      prev_p.set_y(p.y());
    }
  }
  prev_p.set_x(std::numeric_limits<double>::infinity());
  prev_p.set_y(std::numeric_limits<double>::infinity());
  for (const auto& seg : lane->lane().right_boundary().curve().segment()) {
    if (!seg.has_line_segment()) {
      continue;
    }
    for (const auto& p : seg.line_segment().point()) {
      if (!std::isinf(prev_p.x()) && !std::isinf(prev_p.y()) &&
          common::math::double_type::SeemsEqual(prev_p.x(), p.x()) &&
          common::math::double_type::SeemsEqual(prev_p.y(), p.y())) {
        continue;
      }
      right_map_points->emplace_back(PointEarth2Bus({p.x(), p.y()}, pose));
      prev_p.set_x(p.x());
      prev_p.set_y(p.y());
    }
  }
  RemoveDuplicates(left_map_points);
  RemoveDuplicates(right_map_points);
}

Vec2d FailureLocationChecker::PointEarth2Bus(const Vec2d& point,  // NOLINT
                                             const common::Pose& pose) {
  Vec2d bus_point;
  double x = point.x() - pose.position().x();
  double y = point.y() - pose.position().y();
  bus_point.set_x(x * std::cos(pose.heading()) + y * std::sin(pose.heading()));
  bus_point.set_y(-x * std::sin(pose.heading()) + y * std::cos(pose.heading()));
  return bus_point;
}

std::vector<Vec2d> FailureLocationChecker::InterPolateVec2dPoints(
    const std::vector<Vec2d>& map_points, double start_x, double end_x,
    double delta) {
  std::vector<Vec2d> out_points;
  for (uint64_t i = 0; i < map_points.size() - 1; i++) {
    auto bus_point_next = map_points[i + 1];
    if (map_points[i].x() < 0 && map_points[i + 1].x() > 0) {
      auto point = InterPolateVec2dPoint(map_points[i], map_points[i + 1],
                                         start_x - map_points[i].x());
      out_points.emplace_back(point);
      while (bus_point_next.x() - point.x() > 2 * delta && point.x() < end_x) {
        point = InterPolateVec2dPoint(point, bus_point_next, delta);
        out_points.emplace_back(point);
      }
    } else if (map_points[i].x() > 0) {
      auto point = map_points[i];
      out_points.emplace_back(point);
      while (bus_point_next.x() - point.x() > 2 * delta && point.x() < end_x) {
        point = InterPolateVec2dPoint(point, bus_point_next, delta);
        out_points.emplace_back(point);
      }
    }
  }
  return out_points;
}

Vec2d FailureLocationChecker::InterPolateVec2dPoint(  // NOLINT
    const Vec2d& point_start,                         // NOLINT
    const Vec2d& point_end, double dis) {
  return {point_start.x() + dis,
          point_start.y() + dis / (point_end.x() - point_start.x()) *
                                (point_end.y() - point_start.y())};
}

int FailureLocationChecker::LocationErrDecider(
    const std::shared_ptr<LocalView>& local_view,
    functionmanager::FunctionManagerOut* const to_fct) {
  // 0:no err 1:normal err 2: serious err 3: fatal err
  // 1 will reduce to PerceptionMode, 2 or 3 will reduce to PILOT
  int original_location_err_status = 0;
  is_loc_pose_jump_ = false;
  not_change_lane_ = false;
  if (!local_view->HasLocalization() ||
      !local_view->GetLocalization()->has_pose() ||
      !local_view->GetLocalization()->has_header() ||
      !local_view->HasValidLocalizationHeader()) {
    is_location_init_ = false;
    history_pose_time_ = 0.0;
    original_location_err_status = 1;
    return original_location_err_status;
  }
  const auto localization_estimate = local_view->GetLocalization();
  uint64_t location_state = localization_estimate->location_state();
  // 低速下前方有障碍物时，屏蔽123和130故障
  if (DeciderObsBeforeVehicle(local_view) &&
      (location_state == 130 || location_state == 123)) {
    location_state = 2;
  }
  if (is_change_mode_by_odd_type_) {
    location_state = 2;
  }
  ADEBUG << "loc err: " << location_state;
  if (location_state >= kLocationFaultStart &&
      location_state < kLocationFatalFault) {
    original_location_err_status = 3;
  } else if (location_state >= kLocationFatalFault &&
             location_state < kLocationSeriouslFault) {
    original_location_err_status = 2;
  } else if (location_state >= kLocationSeriouslFault &&
             location_state < kLocationNormalFault) {
    original_location_err_status = 1;
  }
  const auto nnp_sys_state =
      (local_view->HasFunctionManagerIn() &&
       local_view->GetFunctionManagerIn()->has_fct_nnp_in())
          ? local_view->GetFunctionManagerIn()->fct_nnp_in().nnp_sysstate()
          : functionmanager::NNPSysState::NNPS_PASSIVE;
  const bool is_drive_auto = (nnp_sys_state == NNPSysState::NNPS_ACTIVE ||
                              nnp_sys_state == NNPSysState::NNPS_OVERRIDE ||
                              nnp_sys_state == NNPSysState::NNPS_LAT_OVERRIDE ||
                              nnp_sys_state == NNPSysState::NNPS_LON_OVERRIDE);
  // 3s延迟debounce
  if (is_drive_auto) {
    original_location_err_status = original_location_err_debounce_.DealDebounce(
                                       original_location_err_status == 1)
                                       ? 1
                                       : 0;
  } else {
    original_location_err_debounce_.Reset();
  }

  not_change_lane_ =
      original_location_err_status == 0 && location_state == kLocationIsFault;
  to_fct->set_no_change_lane(not_change_lane_);
  if (original_location_err_status > 0) {
    is_location_init_ = false;
    history_pose_time_ = 0.0;
    return original_location_err_status;
  }
  const auto pose = localization_estimate->pose();
  const bool is_same_utm_zone = localization_zone_id_ == pose.using_utm_zone();
  localization_zone_id_ = pose.using_utm_zone();
  auto* loc_err_debug =
      to_fct->mutable_real_hdmap_debug()->mutable_location_err_debug();
  if (history_pose_time_ > kMainLoopTime) {
    double time_diff =
        localization_estimate->header().data_stamp() - history_pose_time_;
    double x_diff =
        std::fabs(history_location_point_.x() - pose.position().x() +
                  pose.linear_velocity().x() * time_diff);
    double y_diff =
        std::fabs(history_location_point_.y() - pose.position().y() +
                  pose.linear_velocity().y() * time_diff);
    double vehicle_pose_x =
        x_diff * std::cos(pose.heading()) + y_diff * std::sin(pose.heading());
    double vehicle_pose_y = -(-x_diff * std::sin(pose.heading()) +
                              y_diff * std::cos(pose.heading()));
    if (is_same_utm_zone && !FLAGS_only_using_locationself_err &&
        (vehicle_pose_x > kLocationXerrMin ||
         vehicle_pose_y > kLocationYerrMin)) {
      is_loc_pose_jump_ = is_location_init_;
    } else {
      is_location_init_ = true;
    }
    loc_err_debug->set_pose_err_x(vehicle_pose_x);  // NOLINT
    loc_err_debug->set_pose_err_y(vehicle_pose_y);  // NOLINT
    ADEBUG << "history_location_point x = " << history_location_point_.x()
           << ", y = " << history_location_point_.y()
           << ", now_location_point x = " << pose.position().x()
           << ", y = " << pose.position().y()
           << ", x_diff = " << pose.linear_velocity().x() * time_diff
           << ", pose_diff_x = "
           << history_location_point_.x() - pose.position().x()
           << ", pose_diff_y = "
           << history_location_point_.y() - pose.position().y()
           << ", y_diff = " << pose.linear_velocity().y() * time_diff
           << ", x_err = " << x_diff << ", y_err = " << y_diff
           << ", vehicle_x_err = " << vehicle_pose_x
           << ", vehicle_y_err = " << vehicle_pose_y;
  }
  loc_err_debug->set_is_loc_pose_jump(is_loc_pose_jump_);
  history_pose_time_ = localization_estimate->header().data_stamp();
  history_location_point_.set_x(pose.position().x());
  history_location_point_.set_y(pose.position().y());
  return original_location_err_status;
}

std::pair<double, double> FailureLocationChecker::CalculateValidLocErrHasMap(
    const TL::hdmap::LaneInfoConstPtr& nearestlane) {
  UNUSED(nearestlane);
  // 视觉车道线对地图的偏差，左正右负
  double left_err =
      lanemarkers_.front_left_lane_marker().c0_position() - left_map_c0_;
  double right_err =
      lanemarkers_.front_right_lane_marker().c0_position() - right_map_c0_;
  return std::make_pair(left_err, right_err);
}

bool FailureLocationChecker::DeciderLaneMarkerAndWidth(
    const TL::common ::Pose& pose) {
  const auto vehicle_spd =
      (pose.has_linear_velocity_vrf() && pose.linear_velocity_vrf().has_y())
          ? pose.linear_velocity_vrf().y()
          : 0.0;
  const double good_quality_value = 0.6;
  const double min_lane_width = 3.2;
  const double max_lane_width = 4.5;
  const double lane_a0_coff_var_thd = 0.16;
  const double lane_a1_coff_var_thd = 0.00035;
  const double min_length_view_range = std::max(vehicle_spd * 1.5, 20.0);
  const double min_view_range_bad = std::min(10.0, std::max(vehicle_spd, 1.0));
  const double max_heading_angle = 0.05;
  const double min_heading_angle = 0.01;
  const double k_lane_change_delta = 8.0;
  const auto left_lanemarker = lanemarkers_.front_left_lane_marker();
  const auto right_lanemarker = lanemarkers_.front_right_lane_marker();
  const auto left_c1 = std::fabs(left_lanemarker.c1_heading_angle());
  const auto right_c1 = std::fabs(right_lanemarker.c1_heading_angle());
  ADEBUG << " left_c1 = " << left_c1 << ", right_c1 = " << right_c1;
  const bool is_lanemarker_jump =
      ((std::pow(left_lanemarker_a0_delay_.GetAverageValue() -
                     left_lanemarker.c0_position(),
                 2) > lane_a0_coff_var_thd ||
        std::pow(left_lanemarker_a1_delay_.GetAverageValue() -
                     left_lanemarker.c1_heading_angle(),
                 2) > lane_a1_coff_var_thd ||
        std::pow(right_lanemarker_a0_delay_.GetAverageValue() -
                     right_lanemarker.c0_position(),
                 2) > lane_a0_coff_var_thd ||
        std::pow(right_lanemarker_a1_delay_.GetAverageValue() -
                     right_lanemarker.c1_heading_angle(),
                 2) > lane_a1_coff_var_thd) &&
       (left_c1 < min_heading_angle && right_c1 < min_heading_angle)) ||
      (left_c1 > max_heading_angle || right_c1 > max_heading_angle);
  left_lanemarker_a0_delay_.Deal(left_lanemarker.c0_position());
  left_lanemarker_a1_delay_.Deal(left_lanemarker.c1_heading_angle());
  right_lanemarker_a0_delay_.Deal(right_lanemarker.c0_position());
  right_lanemarker_a1_delay_.Deal(right_lanemarker.c1_heading_angle());

  const auto good_left_lanemarker =
      left_lanemarker_quality_debounce_.DealDebounce(left_lanemarker.quality() >
                                                     good_quality_value);
  const auto good_right_lanemarker =
      right_lanemarker_quality_debounce_.DealDebounce(
          right_lanemarker.quality() > good_quality_value);
  const bool is_good_quality = left_lanemarker.quality() > 0.3 &&
                               right_lanemarker.quality() > 0.3 &&
                               good_left_lanemarker && good_right_lanemarker;
  const auto view_range_deb =
      (left_c1 < min_heading_angle && right_c1 < min_heading_angle)
          ? min_view_range_bad
          : min_length_view_range;
  const auto good_left_view_range =
      left_lanemarker_viewrange_debounce_.DealDebounce(
          left_lanemarker.view_range() > view_range_deb);
  const auto good_right_view_range =
      right_lanemarker_viewrange_debounce_.DealDebounce(
          right_lanemarker.view_range() > view_range_deb);
  const bool good_viewrange =
      left_lanemarker.view_range() > min_view_range_bad &&
      right_lanemarker.view_range() > min_view_range_bad &&
      (good_left_view_range || good_right_view_range);
  double lane_width =
      left_lanemarker.c0_position() - right_lanemarker.c0_position();
  double view_lane_width =
      CalculateLanemarkerY(min_view_range_bad, left_lanemarker) -
      CalculateLanemarkerY(min_view_range_bad, right_lanemarker);
  const bool is_suitable_lane_width = lane_width > min_lane_width &&
                                      lane_width < max_lane_width &&
                                      view_lane_width > min_lane_width - 0.2 &&
                                      view_lane_width < max_lane_width;
  ADEBUG << " is_lanemarker_jump = " << is_lanemarker_jump
         << ", is_good_quality = " << is_good_quality
         << ", good_viewrange = " << good_viewrange
         << ", is_suitable_lane_width = " << is_suitable_lane_width;
  // 只使用quality和viewrange进行判断 降级前判断严格些，不然会出现无车道线判断偏差过大的情况
  const bool is_good_lane = is_good_quality && good_viewrange &&
                            !is_lanemarker_jump && is_suitable_lane_width;
  const bool is_init_good_lane =
      is_good_init_lanemarker_debounce_.DealDebounce(is_good_lane);
  const bool is_break_good_lane =
      is_good_lanemarker_debounce_.DealDebounce(is_good_lane);

  left_lanemarker_c1_delta_ = left_c1 * k_lane_change_delta;
  right_lanemarker_c1_delta_ = right_c1 * k_lane_change_delta;
  return !(loc_err_state_ == 1 ? is_break_good_lane : is_init_good_lane);
}

void FailureLocationChecker::DealLocInitState(
    const std::pair<double, double> width_diff, const bool is_bad_lanemarkers,
    const bool has_no_width_diff, const bool not_in_main_road,
    const bool is_match) {
  const double kShiftWithDiff = FLAGS_min_map_lane_diff;
  const double left_c0 = lanemarkers_.front_left_lane_marker().c0_position();
  const double right_c0 = lanemarkers_.front_right_lane_marker().c0_position();
  const double left_delta = left_c0 < 0.6 ? 0.2 : 0.0;
  const double right_delta = right_c0 > -0.6 ? 0.2 : 0.0;
  // const bool is_step_delta_err =
  //     std::fabs(width_diff.first - width_diff_history_.first) > 0.25 ||
  //     std::fabs(width_diff.second - width_diff_history_.second) > 0.25;
  const bool is_width_diff =
      (std::fabs(width_diff.first) >
           kShiftWithDiff + left_delta + left_lanemarker_c1_delta_ &&
       std::fabs(width_diff.second) > kInitWithDiff &&
       width_diff.first * width_diff.second > 0.0) ||
      (std::fabs(width_diff.first) > kInitWithDiff &&
       std::fabs(width_diff.second) >
           kShiftWithDiff + right_delta + right_lanemarker_c1_delta_ &&
       width_diff.first * width_diff.second > 0.0);
  if (is_bad_lanemarkers || not_in_main_road || is_change_mode_by_odd_type_) {
    width_diff_count_ = 0;
    left_map_c0_ = kDfaultLaneWidth;
    right_map_c0_ = kDfaultLaneWidth;
  } else if (!is_match) {
    width_diff_count_ += kShiftStep;
  } else if (left_c0 < 0.1 || right_c0 > -0.1) {  // NOLINT
    width_diff_count_ = 0;
  } else if ((std::fabs(width_diff.first) > kLanemarkerMinJumpErr &&
              std::fabs(width_diff.second) > kLanemarkerMinJumpErr) ||
             has_no_width_diff) {
    width_diff_count_ += kErrStep;
  } else if (is_width_diff) {
    width_diff_count_++;
    ADEBUG << "width_diff_count: " << width_diff_count_;
  } else {
    width_diff_count_ = 0;
  }
  if (width_diff_count_ > kInitToErrCount) {
    per_loc_err_state_ = LocErrState::Err;
    width_diff_count_ = 0;
  } else if (width_diff_count_ > kInitToShiftCount) {
    per_loc_err_state_ = LocErrState::Shift;
    width_diff_count_ = 0;
  }
}

void FailureLocationChecker::DealLocShiftState(
    const std::pair<double, double> width_diff, const bool is_bad_lanemarkers,
    const bool has_no_width_diff, const bool not_in_main_road,
    const bool is_match) {
  UNUSED(not_in_main_road);
  if ((has_no_width_diff || is_bad_lanemarkers || err_not_main_road_) &&
      !is_change_mode_by_odd_type_) {
    width_diff_count_ += kErrStep;
    left_map_c0_ = kDfaultLaneWidth;
    right_map_c0_ = kDfaultLaneWidth;
  } else if ((std::fabs(width_diff.first - width_diff_history_.first) >
                  kLanemarkerMinJumpErr &&
              std::fabs(width_diff.second - width_diff_history_.second) >
                  kLanemarkerMinJumpErr) &&
             !is_change_mode_by_odd_type_) {
    width_diff_count_ += kErrStep;
  } else if ((std::fabs(width_diff.first) > 1.2 ||
              std::fabs(width_diff.second) > 1.2) &&
             !is_change_mode_by_odd_type_) {
    width_diff_count_++;
    width_same_count_ = 0;
  } else if (std::fabs(width_diff.first) < kInitWithDiff &&
             std::fabs(width_diff.second) < kInitWithDiff && is_match) {
    width_same_count_++;
    width_diff_count_ = 0;
  } else {
    width_diff_count_ = 0;
    width_same_count_ = 0;
  }
  if (width_diff_count_ > kShiftToErrCount) {
    per_loc_err_state_ = LocErrState::Err;
    width_diff_count_ = 0;
  }
  if (width_same_count_ > kToInitCount) {
    per_loc_err_state_ = LocErrState::Loc_Init;
    width_same_count_ = 0;
  }
}

void FailureLocationChecker::Reset() {
  left_map_c0_ = kDfaultLaneWidth;
  right_map_c0_ = kDfaultLaneWidth;
  is_left_lane_change_ = false;
  is_right_lane_change_ = false;
  is_lane_change_ = false;
  double_left_lane_change_rise_decider_.Reset();
  double_right_lane_change_rise_decider_.Reset();
  map_lanechange_observer_.Reset();
  per_lanechange_observer_.Reset();
  left_match_state_ = 0;
  right_match_state_ = 0;
  left_match_state_count_ = 0;
  right_match_state_count_ = 0;
  left_check_dis_count_ = 0;
  right_check_dis_count_ = 0;
  per_loc_err_state_ = LocErrState::Loc_Init;
  loc_err_state_ = 0;
  width_same_count_ = 0;
  err_state_count_ = 0;
}

void FailureLocationChecker::ResetMatchState() {
  left_match_state_ = 0;
  right_match_state_ = 0;
  left_match_state_count_ = 0;
  right_match_state_count_ = 0;
  left_check_dis_count_ = 0;
  right_check_dis_count_ = 0;
}

TL::perception::LaneMarkers
FailureLocationChecker::GenerateVirtualMapLanemarkers() const {
  TL::perception::LaneMarkers lanemarkers;
  lanemarkers.mutable_front_left_lane_marker()->set_c0_position(left_map_c0_);
  lanemarkers.mutable_front_left_lane_marker()->set_quality(1.0);
  lanemarkers.mutable_front_right_lane_marker()->set_c0_position(right_map_c0_);
  lanemarkers.mutable_front_right_lane_marker()->set_quality(1.0);
  return lanemarkers;
}

void FailureLocationChecker::LanechangeRiseDecider(
    const std::pair<bool, bool> perception, const std::pair<bool, bool> map) {
  is_left_lane_change_ = double_left_lane_change_rise_decider_.Decider(
      perception.first, map.first);
  is_right_lane_change_ = double_right_lane_change_rise_decider_.Decider(
      perception.second, map.second);
  is_lane_change_ = is_left_lane_change_ || is_right_lane_change_;
}

double FailureLocationChecker::CalculateLanemarkerY(
    const double distance, const TL::perception::LaneMarker& lane_marker) {
  double x2 = distance * distance;
  double x3 = x2 * distance;
  return lane_marker.c0_position() + lane_marker.c1_heading_angle() * distance +
         lane_marker.c2_curvature() * x2 +
         lane_marker.c3_curvature_derivative() * x3;
}

bool FailureLocationChecker::DeciderObsBeforeVehicle(
    const std::shared_ptr<LocalView>& local_view) {
  const double vehicle_speed = vehicle_state_->linear_velocity();
  const double yaw_rate = vehicle_state_->angular_velocity();
  constexpr double kMinVehicleSpeed = 5.0;
  constexpr double kMaxFrontDistance = kMinVehicleSpeed * 4;
  constexpr double delta_width = 0.3;
  bool has_obs_flag{false};

  if (vehicle_speed > kMinVehicleSpeed) {
    return has_obs_flag;
  }
  const auto& left_lanemarker = lanemarkers_.front_left_lane_marker();
  const auto& right_lanemarker = lanemarkers_.front_right_lane_marker();
  const bool left_quality_good =
      left_lanemarker.quality() > 0.6 && left_lanemarker.view_range() > 20.0;
  const bool right_quality_good =
      right_lanemarker.quality() > 0.6 && right_lanemarker.view_range() > 20.0;
  TL::perception::LaneMarker lanemarker;
  double half_lane_width{2.15};
  if (left_quality_good && right_quality_good) {
    lanemarker.set_c0_position(
        (left_lanemarker.c0_position() + right_lanemarker.c0_position()) / 2);
    lanemarker.set_c1_heading_angle((left_lanemarker.c1_heading_angle() +
                                     right_lanemarker.c1_heading_angle()) /
                                    2);
    lanemarker.set_c2_curvature(
        (left_lanemarker.c2_curvature() + right_lanemarker.c2_curvature()) / 2);
    lanemarker.set_c3_curvature_derivative(
        (left_lanemarker.c3_curvature_derivative() +
         right_lanemarker.c3_curvature_derivative()) /
        2);
    half_lane_width =
        (left_lanemarker.c0_position() - right_lanemarker.c0_position()) / 2 +
        delta_width;
  } else if (left_quality_good && !right_quality_good) {
    lanemarker = left_lanemarker;
    lanemarker.set_c0_position(0.0);
  } else if (!left_quality_good && right_quality_good) {
    lanemarker = right_lanemarker;
    lanemarker.set_c0_position(0.0);
  } else if (!left_quality_good && !right_quality_good) {
    auto curvature =
        common::math::Clamp((yaw_rate / vehicle_speed) / 2, -0.01, 0.01);
    lanemarker.set_c0_position(0.0);
    lanemarker.set_c1_heading_angle(0.0);
    lanemarker.set_c2_curvature(curvature);
    lanemarker.set_c3_curvature_derivative(0.0);
  }
  if (local_view->HasPerceptionObstacles() &&
      local_view->GetPerceptionObstacles() != nullptr) {
    const auto perception_obstacles = local_view->GetPerceptionObstacles();
    for (const auto& obs : perception_obstacles->perception_obstacle()) {
      const auto& obs_y = obs.position_flu().y();
      const auto& obs_x = obs.position_flu().x();
      if (obs_x < 0.0 || obs_x > kMaxFrontDistance) {
        continue;
      }
      auto obs_width_dis = CalculateObsY(lanemarker, obs_x, obs_y);
      if (obs_width_dis < half_lane_width) {
        has_obs_flag = true;
        break;
      }
    }
  }
  ADEBUG << "has_obs_flag: " << has_obs_flag;
  return has_obs_flag;
}

double FailureLocationChecker::CalculateObsY(
    const TL::perception::LaneMarker& lane_marker, double obs_x,
    double obs_y) {
  auto c0 = lane_marker.c0_position();
  auto c1 = lane_marker.c1_heading_angle();
  auto c2 = lane_marker.c2_curvature();
  auto c3 = lane_marker.c3_curvature_derivative();
  auto obs_x2 = obs_x * obs_x;
  auto obs_x3 = obs_x2 * obs_x;
  double obs_c0 = obs_y - c1 * obs_x - c2 * obs_x2 - c3 * obs_x3;
  double angle = c1 + c2 * obs_x + c3 * obs_x2;
  return std::fabs(obs_c0 - c0) * cos(angle);
}

}  // namespace planning
}  // namespace TL
