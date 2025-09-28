/*
 * Copyright (c) TL auto Co., Ltd. 2023-2024. All rights reserved.
 */
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/missile_mode/obs_history_points_generator.h"
#include <sys/types.h>

#include <algorithm>
#include <any>
#include <optional>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <tuple>
#include <utility>
#include <variant>
#include <vector>

#include "Eigen/src/Core/Stride.h"
#include "common/math/curve_fitting.h"
#include "common/math/math_utils.h"
#include "common/math/vec2d.h"
#include "common/status/status.h"
#include "planning/common/planning_gflags.h"
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/missile_mode/obstacles_state.h"
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/missile_mode/vehicle_state_decider.h"
#include "planning/localview/local_view.h"
#include "third_party/x86/protobuf/include/google/protobuf/util/type_resolver.h"
#include "proto/fsm/function_manager.pb.h"
#include "proto/local_mapping/local_map.pb.h"

namespace TL {
namespace planning {
namespace missilelane {
using TL::common::math::FitPolynomial;
using LanemarkerIndex = std::pair<int, const TL::perception::LaneMarker*>;

constexpr uint8_t kODPRnVehTrajBuffSize = 20;
// 纵向最近障碍物距离，小于1m不做处理
constexpr int kTJATvdpGminDistFol = 1;
constexpr double kTJAMainLoopTimeSec = 0.1;
constexpr double kODPRtPntsFixPrvwTimeYaxs = 0.5;
constexpr double kODPRvObjStdStillEnSpd = 1;
constexpr double kODPRvObjStdStillDisenSpd = 1.5;
constexpr double kODPRdHistTrajLngthThd = 1;
constexpr double kODPRkAvgSlopStdDvton = 0.04;  // 目标轨迹点的斜率的标准差
constexpr double kODPRfTrgtVehCrvCrsFrq = 0.5;
constexpr int kTimerSchedulerStep = 1;
constexpr double kODPRtTmAbNrmlPathRise = 0.2;
constexpr double kODPRtTmAbNrmlPathFall = 0.8;
constexpr double kODPRtTrgtVehMoveRiseTime = 0.5;
constexpr double kODPRtrgtVehMoveFallTime = 0.5;
constexpr int kMagicNumber5 = 5;
constexpr double kCurvatureNum = 1.5;

template <typename T>
google::protobuf::internal::RepeatedPtrIterator<const T> FindById(
    const google::protobuf::RepeatedPtrField<T>& input, int id) {
  return std::find_if(input.begin(), input.end(),
                      [&](const T& in) { return in.track_id() == id; });
}

template <typename T>
mapping::LaneCubicCurve FindParam(
    const google::protobuf::RepeatedPtrField<T>& input, int id) {
  const auto it = std::find_if(input.begin(), input.end(), [&](const T& in) {
    return in.track_id() == id;
  });
  return (it != input.end() && !it->lane_param().cubic_curve_set().empty())
             ? it->lane_param().cubic_curve_set().at(0)
             : mapping::LaneCubicCurve();
}

template <typename T>
void Trans2Lanemarker(const T& input, LaneMarker* lanemarker) {
  lanemarker->set_line_seq(input.track_id());
  const auto param = input.lane_param().cubic_curve_set().at(0);
  lanemarker->set_c0_position(param.c0());
  lanemarker->set_c1_heading_angle(param.c1());
  lanemarker->set_c2_curvature(param.c2());
  lanemarker->set_c3_curvature_derivative(param.c3());
  lanemarker->set_longitude_start(param.start_point_x());
  lanemarker->set_longitude_end(param.end_point_x());
  lanemarker->set_view_range(param.end_point_x() - param.start_point_x());
  lanemarker->set_quality(input.confidence());
}

Status ObsHisPointsGenerator::Init(
    const std::shared_ptr<ObstaclesState>& obstacles_state_ptr) {
  obstacles_state_ = obstacles_state_ptr;

  vehicle_state_ = obstacles_state_ptr->vehicle_state();

  avg_trgveh_move_curvature_filter_.set_fs_and_fc(1 / kTJAMainLoopTimeSec,
                                                  kODPRfTrgtVehCrvCrsFrq);
  avg_trgveh_move_heading_filter_.set_fs_and_fc(1 / kTJAMainLoopTimeSec,
                                                kODPRfTrgtVehCrvCrsFrq);
  k_standard_deviation_filter_.set_fs_and_fc(1 / kTJAMainLoopTimeSec,
                                             kODPRfTrgtVehCrvCrsFrq);
  trgveh_curve_ddy_filter_.set_fs_and_fc(1 / kTJAMainLoopTimeSec,
                                         kODPRfTrgtVehCrvCrsFrq);
  obs_lat_y_filter_.SetCoefficientAndFlag(0.5, false);
  c1_heading_delay_ = lanelineprocess::Delay<double>(kMagicNumber5);

  est_trgmovetrend_not_relible_debounce_.ResetTime(
      kODPRtTmAbNrmlPathRise, kODPRtTmAbNrmlPathFall, kTJAMainLoopTimeSec);
  // 预瞄距离插值表
  TJAdistEndPointTableCreater();
  TJA_ODPR_dist_EndPoint_.Init(xy1_);
  return Status::OK();
}

// 默认第一个障碍物为目标障碍物，其它的可能也需要记录历史轨迹点,可以使用perception_obstacles_先测试
bool ObsHisPointsGenerator::Process(
    const std::shared_ptr<LocalView>& local_view) {
  // 处理障碍物生成轨迹点
  bool is_deal_obs_ok = DealObs(local_view);
  // 检测到车道线且能生成地图后用车道线地图取代跟车地图
  bool is_deal_find_lane_ok = DealLaneLine(local_view);
  bool is_deal_no_obs_lane_ok = DealNoObsLaneLine(local_view);
  ADEBUG << "missile_mode_state: " << missile_mode_state_
         << " ,is_deal_obs_ok: " << is_deal_obs_ok
         << " ,is_deal_find_lane_ok: " << is_deal_find_lane_ok
         << " , is_deal_no_obs_lane_ok: " << is_deal_no_obs_lane_ok;
  // 0:init  1:using lane(no obs)  2:follow obs  3:find lane
  switch (missile_mode_state_) {
    case 0:
      if (is_deal_obs_ok) {
        missile_mode_state_ = 2;
      } else if (is_deal_no_obs_lane_ok) {
        missile_mode_state_ = 1;
        obs_lanemarker_ = no_obs_lanemarker_;
      }
      break;
    case 1:
      if (is_deal_obs_ok) {
        missile_mode_state_ = 2;
      } else if (is_deal_find_lane_ok) {
        missile_mode_state_ = 3;
        obs_lanemarker_ = laneline_lanemarker_;
      } else if (is_deal_no_obs_lane_ok) {
        obs_lanemarker_ = no_obs_lanemarker_;
      } else {
        missile_mode_state_ = 0;
      }
      break;
    case 2:
      if (is_deal_find_lane_ok) {
        missile_mode_state_ = 3;
        obs_lanemarker_ = laneline_lanemarker_;
      } else if (!is_deal_obs_ok && is_deal_no_obs_lane_ok) {
        missile_mode_state_ = 1;
        obs_lanemarker_ = no_obs_lanemarker_;
      } else if (!is_deal_obs_ok) {
        missile_mode_state_ = 0;
      }
      break;
    case 3:
      if (!is_deal_find_lane_ok && is_deal_obs_ok) {
        missile_mode_state_ = 2;
      } else if (!is_deal_find_lane_ok && !is_deal_obs_ok) {
        missile_mode_state_ = 0;
      }
      obs_lanemarker_ = laneline_lanemarker_;
      break;
    default:
      break;
  }
  CreatInitLaneMarkerPoints(obs_lanemarker_, &obs_points_);
  // for (auto& point : obs_points_) {
  //   double lane_y = coff[0] + point.x() * coff[1] +
  //                   point.x() * point.x() * coff[2] +
  //                   point.x() * point.x() * point.x() * coff[3];

  // AERROR << "out x: " << point.x() << " , y: " << point.y()
  //        << " ,lane_y: " << lane_y;
  // point.set_y(lane_y);
  // }
  vehicle_state_->local_view_data()->update_data()->set_missile_mode_state(
      missile_mode_state_);
  // 检查未激活前车辆和地图的夹角和位置
  bool is_in_control = false;
  if (local_view->HasFunctionManagerIn() &&
      local_view->GetFunctionManagerIn()->ta_pilot_mode() ==
          TL::functionmanager::TaPilotMode::NO_CONTROL) {
    is_in_control = true;
  }
  bool is_check_c0_c1_fit =
      (std::fabs(obs_lanemarker_.c0_position()) < 2) &&
      (std::fabs(obs_lanemarker_.c1_heading_angle()) < 0.18);
  ADEBUG << "******end missile_mode_state******: " << missile_mode_state_;
  return (missile_mode_state_ != 0) &&
         (is_in_control ? is_check_c0_c1_fit : true);
}

bool ObsHisPointsGenerator::DealObs(
    const std::shared_ptr<LocalView>& local_view) {
  const auto& missile_obs =
      obstacles_state_->TargetObs(obs_lanemarker_, local_view);
  const auto& obs = missile_obs.deq_obs.front();
  // 数据预处理
  DataPreprocess(obs);
  obs_heading_ = missile_obs.average_heading;
  if (missile_mode_state_ != 2 || !vehicle_state_->is_missile_mode_active()) {
    current_points_state_ = Obs_Out;
  }
  bool is_deal_obs_ok{false};
  if (ObsVaildDecider(missile_obs)) {
    // 跟车目标消失或切换判断，目标消失需要车辆行驶到目标消失处，障碍物无
    // 法切换时也需要沿着原来的障碍物轨迹继续行驶直到满足切换条件
    bool need_using_history_points = NeedUsingHisPoints(missile_obs);
    ADEBUG << " need_using_history: " << need_using_history_points
           << " , time: " << using_history_points_time_
           << " , length: " << using_history_points_length_;
    if (need_using_history_points) {
      ADEBUG << "need_using_history_points: " << using_history_points_time_;
      TranslateHisPoints(&obs_lanemarker_, &using_history_points_length_,
                         &using_history_points_time_);
    } else {
      // 求取目标车辆后方历史轨迹点--20个点
      GenerateObsBackPoints(obs);

      if (!hist_points_buffer_update_.empty() &&
          hist_points_buffer_update_.back().x() < 0.1) {
        ADEBUG << " HAS NO points_buffer_update !";
        return false;
      }
      // for (const auto& point : hist_points_buffer_update_) {
      //   AERROR << "hist back points_update x: " << point.x()
      //          << " , y: " << point.y();
      // }
      // 打印历史轨迹
      // for (const auto& point : hist_points_buffer_) {
      //   AERROR << "hist back points x: " << point.x() << " , y: " << point.y();
      // }

      // 求取heading和curve
      GenerateObsHeadingAndCurve(obs);
      if (((vehicle_state_ != nullptr &&
            !vehicle_state_->is_missile_mode_active()) ||
           missile_mode_state_ == 1) &&
          std::fabs(average_curve_) > 0.004 && is_stand_deviation_nok_) {
        ADEBUG << " average_curve greater than 0.004 or "
                  "target_mvtrnd_not_reliable !!!!";
        return false;
      }
      // 最终生成目标前方轨迹点并存储在pre_points_x，pre_points_y
      Eigen::MatrixXd pre_points_x;  // 缩减后的目标前方轨迹点19->9
      Eigen::MatrixXd pre_points_y;  // 计算奇数项和偶数项的平均值
      is_prepoint_generate_ =
          GenerateTrgtVehPreTrajPnts(&pre_points_x, &pre_points_y);
      // 打印预测轨迹
      // for (int i = 0; i < pre_points_x.cols(); ++i) {
      //   AERROR << " pre_points_[" << i << "]X: " << pre_points_x(0, i)
      //          << " , y: " << pre_points_y(0, i);
      // }
      // 最终历史轨迹点和预测轨迹点的merge，前车后方轨迹点x：10，前车前方预测轨迹点x：9，第20个点x 填充0
      // 此处输出最终的轨迹点
      if (is_prepoint_generate_) {
        TrgtVehTrajPntsMergeFix(pre_points_x, pre_points_y);
        ADEBUG << "obs_points SIZE: " << obs_points_.size();
      }
      // 打印输出轨迹
      // for (const auto& point : obs_points_) {
      //   AERROR << "out x: " << point.x() << " , y: " << point.y();
      // }
      if (obs_points_.size() < 3) {
        return false;
      }
      DealBackPoints();
      // for (const auto& point : obs_points_) {
      //   AERROR << "out x: " << point.x() << " , y: " << point.y();
      // }
      const int N = 3;
      // std::vector<Vec2d> fit_points;
      // fit_points.reserve(hist_points_buffer_.size());
      // for (auto const& point : hist_points_buffer_) {
      //   fit_points.emplace_back(point);
      // }
      std::vector<double> coff = FitPolynomial<N>(obs_points_);
      obs_lanemarker_.set_c0_position(coff[0]);
      obs_lanemarker_.set_c1_heading_angle(coff[1]);
      obs_lanemarker_.set_c2_curvature(coff[2]);
      obs_lanemarker_.set_c3_curvature_derivative(coff[3]);
      obs_lanemarker_.set_longitude_start(obs_points_.front().x());
      obs_lanemarker_.set_longitude_end(obs_points_.back().x());
      obs_lanemarker_.set_view_range(obs_points_.back().x());
      max_using_history_points_length_ = obs_x_;
      using_history_points_time_ = 0.0;
      using_history_points_length_ = 0.0;
      obs_id_last_ = obs_id_;
      ADEBUG << " fit C0: " << coff[0] << " , C1: " << coff[1]
             << " , C2: " << coff[2] << " , C3: " << coff[3];
    }
    is_deal_obs_ok = obstacles_state_->IsObsLanemarkerValid(obs_lanemarker_);
  }
  return is_deal_obs_ok;
}

bool ObsHisPointsGenerator::DealLaneLine(
    const std::shared_ptr<LocalView>& local_view) {
  ADEBUG << "---find laneline---"
         << " ,is_missile_mode_active: "
         << vehicle_state_->is_missile_mode_active();
  ADEBUG << "DealLaneLine"
         << " history_perception_sub_state_:" << history_perception_sub_state_;

  if (!vehicle_state_->is_missile_mode_active()) {
    lane_line_state_ = 0;
    laneline_state_one_count_ = 0;
    has_lanemarker_debounce_.Reset();
    return false;
  }
  TL::perception::LaneMarker left_lanemarker{};
  TL::perception::LaneMarker right_lanemarker{};
  auto [has_right_lanemarker_bl, has_left_lanemarker_bl] =
      CheckerLanemarker(local_view, &left_lanemarker, &right_lanemarker);
  ADEBUG << " has_right_lanemarker_bl: " << has_right_lanemarker_bl
         << " , has_left_lanemarker_bl: " << has_left_lanemarker_bl;
  bool has_lanemarker = false;
  bool has_left_lanemarker{false};
  bool has_right_lanemarker{false};
  bool has_next_left_lanemarker{};
  bool has_next_right_lanemarker{};
  bool is_left_c0_near_ok{false};
  bool is_right_c0_near_ok{false};
  if (has_left_lanemarker_bl) {
    double left_c0{0.0};
    double longitude_start = left_lanemarker.longitude_start();
    bool is_viewrange_fit =
        left_lanemarker.longitude_end() - longitude_start > 20.0 &&
        longitude_start < 50.0 && left_lanemarker.longitude_end() > 15.0 &&
        (missile_mode_state_ == 1 ? longitude_start > -5.0 : true);
    if (longitude_start > 5.0) {
      left_c0 = CalculateLanemarkerY(longitude_start, left_lanemarker);
    } else {
      left_c0 = left_lanemarker.c0_position();
    }
    double left_c1 = left_lanemarker.c1_heading_angle() +
                     2 * left_lanemarker.c2_curvature() * longitude_start +
                     3 * left_lanemarker.c3_curvature_derivative() *
                         longitude_start * longitude_start;
    double left_c2 =
        2 * left_lanemarker.c2_curvature() +
        6 * left_lanemarker.c3_curvature_derivative() * longitude_start;
    bool is_c0_heading_fit =
        left_c0 > -0.6 && left_c0 < 4.3 && std::fabs(left_c1) < 0.1;

    has_left_lanemarker = is_viewrange_fit && is_c0_heading_fit;
    // double left_c2 = RoadCurvatureCalculate(left_lanemarker, 0);
    is_left_c0_near_ok =
        left_c0 < 1.6 && std::fabs(left_lanemarker.c2_curvature()) < 0.001;
    ADEBUG << " left longitude_start: " << longitude_start
           << " ,longitude_end: " << left_lanemarker.longitude_end()
           << " ,left_c0: " << left_c0
           << " ,c1: " << left_lanemarker.c1_heading_angle()
           << " , longitude_start c1: " << left_c1
           << " ,is_left_c0_near_ok: " << is_left_c0_near_ok
           << " ,left_c2: " << left_c2;
  } else {
    ADEBUG << "has_no_front_left_lane_marker";
  }
  if (has_right_lanemarker_bl) {
    double longitude_start = right_lanemarker.longitude_start();
    bool is_viewrange_fit =
        right_lanemarker.longitude_end() - longitude_start > 20.0 &&
        longitude_start < 50.0 && right_lanemarker.longitude_end() > 15.0 &&
        (missile_mode_state_ == 1 ? longitude_start > -5.0 : true);
    double right_c0{0.0};
    if (longitude_start > 5.0) {
      right_c0 = CalculateLanemarkerY(longitude_start, right_lanemarker);
    } else {
      right_c0 = right_lanemarker.c0_position();
    }
    double right_c1 = right_lanemarker.c1_heading_angle() +
                      2 * right_lanemarker.c2_curvature() * longitude_start +
                      3 * right_lanemarker.c3_curvature_derivative() *
                          longitude_start * longitude_start;
    bool is_c0_heading_fit =
        right_c0 < 0.6 && right_c0 > -4.3 && std::fabs(right_c1) < 0.1;

    has_right_lanemarker = is_viewrange_fit && is_c0_heading_fit;
    // double right_c2 = RoadCurvatureCalculate(right_lanemarker, 0);
    is_right_c0_near_ok = abs(right_c0) < 1.6 &&
                          std::fabs(right_lanemarker.c2_curvature()) < 0.001;
    ADEBUG << " right longitude_start: " << longitude_start
           << " ,longitude_end: " << right_lanemarker.longitude_end()
           << " ,right_c0: " << right_c0
           << " ,c1: " << right_lanemarker.c1_heading_angle()
           << " , longitude_start c1: " << right_c1
           << " ,is_right_c0_near_ok: " << is_right_c0_near_ok;
  } else {
    ADEBUG << "has_no_front_right_lane_marker";
  }
  if (lane_line_state_ == 0) {
    has_lanemarker = has_lanemarker_debounce_.DealDebounce(
        has_left_lanemarker && has_right_lanemarker);
  } else {
    has_lanemarker_debounce_.Reset();
  }
  half_lane_width_ = 1.875;
  ADEBUG << "has_left_lanemarker: " << has_left_lanemarker
         << " , has_right_lanemarker: " << has_right_lanemarker
         << " , has_next_left_lanemarker: " << has_next_left_lanemarker
         << " , has_next_right_lanemarker: " << has_next_right_lanemarker
         << " , lane_line_state: " << lane_line_state_
         << " , has_host_passable_lane: "
         << (has_left_lanemarker && has_right_lanemarker)
         << " , has_left_passable_lane: "
         << (has_next_left_lanemarker && has_left_lanemarker)
         << " , has_right_passable_lane: "
         << (has_right_lanemarker && has_next_right_lanemarker);

  switch (lane_line_state_) {
    case 0:
      if (has_lanemarker) {
        lane_line_state_ = 1;
        GenerateAllLanemarker(left_lanemarker, right_lanemarker);
      } else if (no_obs_lane_line_state_ == 2 &&  // 增加单边线
                 ((!has_right_lanemarker && has_left_lanemarker &&
                   is_left_c0_near_ok) ||
                  (!has_left_lanemarker && has_right_lanemarker &&
                   is_right_c0_near_ok))) {
        lane_line_state_ = 1;
        if (has_left_lanemarker) {
          GenerateOneLanemarker(left_lanemarker, 1);
        } else if (has_right_lanemarker) {
          GenerateOneLanemarker(right_lanemarker, -1);
        }
      }
      break;

    case 1:
      if (has_left_lanemarker && has_right_lanemarker) {
        GenerateAllLanemarker(left_lanemarker, right_lanemarker);
      } else if (has_left_lanemarker) {
        GenerateOneLanemarker(left_lanemarker, 1);
      } else if (has_right_lanemarker) {
        GenerateOneLanemarker(right_lanemarker, -1);
      } else if (laneline_state_one_count_ < 3) {
        laneline_state_one_count_++;
      } else {
        lane_line_state_ = 2;
      }
      break;
    case 2:
      lane_line_state_ = 0;
      break;
    default:
      break;
  }
  return lane_line_state_ == 1;
}

std::pair<bool, bool> ObsHisPointsGenerator::CheckerLanemarker(
    const std::shared_ptr<LocalView>& local_view,
    TL::perception::LaneMarker* left_lanemarker,
    TL::perception::LaneMarker* right_lanemarker) {
  DealLocalMap(local_view);

  if (pre_lanes_.empty()) {
    ADEBUG << " pre_lanes_.empty(),!!!";
    return {false, false};
  }
  const auto& local_map = local_view->GetLocalMap();
  const auto& lane_lines = local_map->lane_lines();
  const auto& road_edges = local_map->road_edges();

  // Define a variant to hold either LaneLine or RoadEdge iterators
  using LaneOrEdgeIt =
      std::variant<google::protobuf::internal::RepeatedPtrIterator<
                       const TL::mapping::LaneLine>,
                   google::protobuf::internal::RepeatedPtrIterator<
                       const TL::mapping::RoadEdge>>;

  // Lambda to find a track ID in lane_lines or road_edges
  auto find_track = [&](int track_id) -> std::optional<LaneOrEdgeIt> {
    auto lane_it = FindById(lane_lines, track_id);
    if (lane_it != lane_lines.end()) {
      return lane_it;
    }

    auto edge_it = FindById(road_edges, track_id);
    if (edge_it != road_edges.end()) {
      return edge_it;
    }

    return std::nullopt;
  };

  // Lambda to process and transform lane markers
  auto process_lanemarker =
      [&](int track_id, TL::perception::LaneMarker* lanemarker) -> bool {
    if (auto it = find_track(track_id)) {
      std::visit([&](auto&& arg) { Trans2Lanemarker(*arg, lanemarker); }, *it);
      ADEBUG << "Lanemarker line_seq(): " << lanemarker->line_seq();
      return true;
    }
    return false;
  };
  // if (!pre_lanes_.empty()) {
  //   for (auto& lane : pre_lanes_) {
  //     AERROR << ", left_track_id: " << lane.left_track_id
  //            << ", right_track_id: " << lane.right_track_id
  //            << ", central_lanemarker.c0: "
  //            << lane.central_lanemarker.c0_position()
  //            << ", lane_width/2: " << lane.lane_width / 2;
  //   }
  // }

  // Missile mode: Process specific states
  if (vehicle_state_->is_missile_mode_active() && lane_line_state_ != 1) {
    // Calculate obstacle width and sort pre_lanes
    if (missile_mode_state_ == 2) {
      auto calculate_obs_width = [&](const PreLanes& lane) {
        return CalculateObsY(lane.central_lanemarker, obs_x_, obs_y_);
      };

      // std::sort(pre_lanes_.begin(), pre_lanes_.end(),
      //           [&](const PreLanes& lhs, const PreLanes& rhs) {
      //             return calculate_obs_width(lhs) < calculate_obs_width(rhs);
      //           }); // 暂时屏蔽，针对于目标车骑线可能存在误选车道
      auto it = std::find_if(
          pre_lanes_.begin(), pre_lanes_.end(), [&](const PreLanes& input) {
            bool right_exists = find_track(input.right_track_id).has_value();
            bool left_exists = find_track(input.left_track_id).has_value();
            bool width_condition =
                calculate_obs_width(input) < input.lane_width / 2;
            bool central_line_condition =
                abs(input.central_lanemarker.c0_position()) <
                input.lane_width / 2;
            return right_exists && left_exists && central_line_condition &&
                   width_condition;  // central_line_condition  has a higher priority than width_condition
          });

      if (it == pre_lanes_.end()) {
        ADEBUG << " it == pre_lanes_.end()!!!";
        return {false, false};
      }

      his_pre_lane_ = *it;
      bool left_result = process_lanemarker(it->left_track_id, left_lanemarker);
      bool right_result =
          process_lanemarker(it->right_track_id, right_lanemarker);

      ADEBUG << " ****first find lane by follow obs, left id: "
             << his_pre_lane_.left_track_id
             << " , right id: " << his_pre_lane_.right_track_id;
      return {right_result, left_result};
    }
    //  Sort pre_lanes and find by central line position
    if (missile_mode_state_ == 1) {
      std::sort(pre_lanes_.begin(), pre_lanes_.end(),
                [&](const PreLanes& lhs, const PreLanes& rhs) {
                  return lhs.central_lanemarker.c0_position() <
                         rhs.central_lanemarker.c0_position();
                });

      auto it = std::find_if(
          pre_lanes_.begin(), pre_lanes_.end(), [&](const PreLanes& input) {
            bool right_exists = find_track(input.right_track_id).has_value();
            bool left_exists = find_track(input.left_track_id).has_value();
            bool central_line_condition =
                abs(input.central_lanemarker.c0_position()) <
                input.lane_width / 2;
            return right_exists && left_exists && central_line_condition;
          });

      if (it == pre_lanes_.end()) {
        ADEBUG << " it == pre_lanes_.end()!!!";
        return {false, false};
      }

      his_pre_lane_ = *it;
      bool left_result = process_lanemarker(it->left_track_id, left_lanemarker);
      bool right_result =
          process_lanemarker(it->right_track_id, right_lanemarker);

      ADEBUG << " ****first find lane by central line, left id: "
             << his_pre_lane_.left_track_id
             << " , right id: " << his_pre_lane_.right_track_id;
      return {right_result, left_result};
    }
  }

  // Standard lane marker assignment
  bool left_lanemarker_bl =
      process_lanemarker(his_pre_lane_.left_track_id, left_lanemarker);
  bool right_lanemarker_bl =
      process_lanemarker(his_pre_lane_.right_track_id, right_lanemarker);

  ADEBUG << " left_it: " << his_pre_lane_.left_track_id
         << " , right_it: " << his_pre_lane_.right_track_id;

  return {right_lanemarker_bl, left_lanemarker_bl};
}

void ObsHisPointsGenerator::DealLocalMap(
    const std::shared_ptr<LocalView>& local_view) {
  if (!local_view->HasLocalMap()) {
    return;
  }
  const auto& local_map = local_view->GetLocalMap();
  if (local_map->lane_lines_size() < 1) {
    return;
  }
  GetTargetLanes(local_view);
}

void ObsHisPointsGenerator::GetTargetLanes(
    const std::shared_ptr<LocalView>& local_view) {
  const auto& local_map = local_view->GetLocalMap();
  const auto& lane_lines = local_map->lane_lines();
  const auto& road_edges = local_map->road_edges();
  // const auto& arrows = local_map->arrows();
  // const auto& stop_lines = local_map->stop_lines();
  // 收集可用的lane id，并查找是否有左侧边界lane
  ADEBUG << "----------pre lane-----------";
  pre_lanes_.clear();
  int left_pre_road_laneline_id{0};
  std::vector<std::pair<int, double>> pre_laneline_ids;
  double max_start_x{0.0};
  // Step 1: Collect lane line IDs and find left boundary lane
  for (const auto& lane_line : lane_lines) {
    if (!lane_line.points().empty()) {
      const auto& points = lane_line.points();
      const auto start_point_x =
          lane_line.lane_param().cubic_curve_set().at(0).start_point_x();
      const auto end_point_x =
          lane_line.lane_param().cubic_curve_set().at(0).end_point_x();
      // 如果lane的成熟度大于10且丢失的帧数小于10，且vieWRange>20m，也认为是有效的lane
      // 需要区分消亡车道
      if ((points.begin()->x() > 0.0 && points.rbegin()->x() > 40.0) ||
          (lane_line.tracked_age() > 5 && lane_line.lost_age() < 10 &&
           end_point_x > 0.0 && (end_point_x - start_point_x) > 20.0)) {
        if (points.begin()->x() > max_start_x) {
          max_start_x = points.begin()->x();
        }
        ADEBUG << " pre_laneline_id: " << lane_line.track_id();
        pre_laneline_ids.emplace_back(lane_line.track_id(), 0.0);
        if (lane_line.color() == mapping::YELLOW &&
            (lane_line.lanetype() == mapping::LaneType_SOLID ||
             lane_line.lanetype() == mapping::LaneType_DOUBLE_SOLID)) {
          left_pre_road_laneline_id = lane_line.track_id();
          ADEBUG << " left_pre_road_laneline_id: " << left_pre_road_laneline_id;
        }
      }
    }
  }
  if (pre_laneline_ids.empty()) {
    ADEBUG << " pre_laneline_ids.empty()!!!";
    return;
  }
  for (auto& id : pre_laneline_ids) {
    auto param = FindParam(lane_lines, id.first);
    double x2 = max_start_x * max_start_x;
    double x3 = x2 * max_start_x;
    id.second = param.c0() + param.c1() * max_start_x + param.c2() * x2 +
                param.c3() * x3;
  }

  //  Sort lane lines by c0对收集的自车前的lane按照c0大小排序
  std::sort(pre_laneline_ids.begin(), pre_laneline_ids.end(),
            [&](const std::pair<int, double>& input_fir,
                const std::pair<int, double>& input_sec) {
              return input_fir.second > input_sec.second;
            });
  // for (int i = 0; i < pre_laneline_ids.size(); ++i) {
  //   ADEBUG << " pre_laneline index[" << i
  //          << "],id : " << pre_laneline_ids[i].first
  //          << " ,c0: " << pre_laneline_ids[i].second;
  // }
  // Step 2: Find left and right road edges
  // Todo： 如果有多个，需要选择左侧c0最小的
  std::vector<std::pair<int, double>> pre_edge_id{};
  for (const auto& road_edge : road_edges) {
    auto point = road_edge.points().at(0);
    const auto road_edge_start_point_x =
        road_edge.lane_param().cubic_curve_set().at(0).start_point_x();
    const auto road_edge_end_point_x =
        road_edge.lane_param().cubic_curve_set().at(0).end_point_x();
    const auto road_edge_start_c0 =
        road_edge.lane_param().cubic_curve_set().at(0).c0();
    if ((!road_edge.points().empty() && road_edge.points().begin()->x() > 0.0 &&
         road_edge.points().rbegin()->x() > 20.0) ||
        (road_edge.tracked_age() > 10 && road_edge.lost_age() < 10 &&
         abs(road_edge_start_c0) < 10.0 && road_edge_end_point_x > 0.0 &&
         (road_edge_end_point_x - road_edge_start_point_x) > 20.0)) {
      auto it =
          std::find_if(road_edge.points().begin(), road_edge.points().end(),
                       [&](const common::Point3D& input) {
                         return input.x() > max_start_x;
                       });
      double pre_c0 = it != road_edge.points().end()
                          ? it->y()
                          : road_edge.points().rbegin()->y();
      pre_edge_id.emplace_back(road_edge.track_id(), pre_c0);
    }
  }
  if (pre_edge_id.empty()) {
    ADEBUG << " pre_edge_id.empty()!!!";
    return;
  }
  std::sort(pre_edge_id.begin(), pre_edge_id.end(),
            [&](const std::pair<int, double>& input_fir,
                const std::pair<int, double>& input_sec) {
              return input_fir.second < input_sec.second;
            });
  // for (int i = 0; i < pre_edge_id.size(); ++i) {
  //   ADEBUG << " pre_edge index[" << i << "],id : " << pre_edge_id[i].first
  //          << " ,c0: " << pre_edge_id[i].second;
  // }
  auto left_road_it = std::find_if(
      pre_edge_id.begin(), pre_edge_id.end(),
      [&](const std::pair<int, double>& input) { return input.second > 0.0; });
  ADEBUG << " left_road_it: " << left_road_it->first;
  // Step 3: Determine start_left_lanemarker and end_right_lanemarker
  perception::LaneMarker start_left_lanemarker;
  perception::LaneMarker end_right_lanemarker;
  double start_pre_c0 = 0.0;
  double end_pre_c0 = 0.0;
  if (left_pre_road_laneline_id != 0) {  // 存在左侧边界黄实线lane
    Trans2Lanemarker(*FindById(lane_lines, left_pre_road_laneline_id),
                     &start_left_lanemarker);
    auto it = std::find_if(pre_laneline_ids.begin(), pre_laneline_ids.end(),
                           [&](const std::pair<int, double>& input) {
                             return input.first == left_pre_road_laneline_id;
                           });
    start_pre_c0 = it->second;
    ADEBUG << " start_pre_c0------determined by  yellow......... "
           << start_left_lanemarker.line_seq();
  } else if (left_road_it != pre_edge_id.end()) {  // 根据左侧道路边缘确定
    auto it = std::find_if(pre_laneline_ids.begin(), pre_laneline_ids.end(),
                           [&](const std::pair<int, double>& input) {
                             return left_road_it->second - input.second > 0.0 &&
                                    left_road_it->second - input.second < 1.2;
                           });
    if (it != pre_laneline_ids.end()) {
      Trans2Lanemarker(*FindById(lane_lines, it->first),
                       &start_left_lanemarker);
      start_pre_c0 = it->second;
    } else {
      Trans2Lanemarker(*FindById(road_edges, left_road_it->first),
                       &start_left_lanemarker);
      start_pre_c0 = left_road_it->second - 0.3;
    }
    ADEBUG << " start_pre_c0 ----determined by left roadedge. "
           << start_left_lanemarker.line_seq();
  } else {  // 根据最左侧车道线确定：
    Trans2Lanemarker(*FindById(lane_lines, pre_laneline_ids.begin()->first),
                     &start_left_lanemarker);
    start_pre_c0 = pre_laneline_ids.begin()->second;
    ADEBUG << " start_pre_c0-----determined by left lane. "
           << start_left_lanemarker.line_seq();
  }
  std::sort(pre_edge_id.begin(), pre_edge_id.end(),
            [&](const std::pair<int, double>& input_fir,
                const std::pair<int, double>& input_sec) {
              return input_fir.second > input_sec.second;
            });
  // for (int i = 0; i < pre_edge_id.size(); ++i) {
  //   ADEBUG << "  pre_edge index[" << i << "],id : " << pre_edge_id[i].first
  //          << ",c0: " << pre_edge_id[i].second;
  // }

  auto right_road_it = std::find_if(
      pre_edge_id.begin(), pre_edge_id.end(),
      [&](const std::pair<int, double>& input) { return input.second < 0.0; });
  ADEBUG << " right_road_it: " << right_road_it->first;

  if (right_road_it != pre_edge_id.end()) {
    Trans2Lanemarker(*FindById(road_edges, right_road_it->first),
                     &end_right_lanemarker);
    end_pre_c0 = right_road_it->second + 0.3;
    ADEBUG << " end_pre_c0............determined by right roadedge. "
           << end_right_lanemarker.line_seq();
  } else {
    Trans2Lanemarker(*FindById(lane_lines, pre_laneline_ids.back().first),
                     &end_right_lanemarker);
    end_pre_c0 = pre_laneline_ids.back().second;
    ADEBUG << " end_pre_c0............determined by right lane. "
           << end_right_lanemarker.line_seq();
  }

  // Step 4: Calculate lane width and generate lanes
  double lane_width{3.75};
  int lane_size =
      static_cast<int>(std::round((start_pre_c0 - end_pre_c0) / lane_width));
  double lane_width_init = (start_pre_c0 - end_pre_c0) / lane_size;
  if (lane_width_init > 2.5 && lane_width_init < 4.5) {
    lane_width = lane_width_init;
  }
  ADEBUG << " view_range: " << max_start_x << " ,lane_width: " << lane_width
         << " ,start_pre_c0: " << start_pre_c0 << " ,end_pre_c0: " << end_pre_c0
         << " ,lane_size: " << lane_size;
  u_int32_t count = 0;
  PreLanes pre_lane{};  // 用于存储每次迭代生成的车道struct信息。
  while (start_pre_c0 > end_pre_c0 + 2.0 && count < 6) {
    ++count;
    auto it = std::find_if(pre_laneline_ids.begin(), pre_laneline_ids.end(),
                           [&](const std::pair<int, double>& input) {
                             return start_pre_c0 - input.second > 2.0;
                           });
    ADEBUG << "while------------it: " << it->first;
    perception::LaneMarker right_lanemarker;  // 虚拟车道线
    if (it == pre_laneline_ids.end() || start_pre_c0 - it->second > 4.5) {
      if (start_pre_c0 - end_pre_c0 < 4.5) {
        right_lanemarker = end_right_lanemarker;
        lane_width = start_pre_c0 - end_pre_c0;
        ADEBUG << " ............determined by  end_right_lanemarker. "
               << right_lanemarker.line_seq() << " ,lane_width: " << lane_width;
      } else {
        right_lanemarker = start_left_lanemarker;
        right_lanemarker.set_line_seq(0);
        // lane_width = 3.75;                 // Default lane width
        ADEBUG << " ............determined by  start_left_lanemarker:   "
               << right_lanemarker.line_seq() << " ,lane_width: " << lane_width;
      }

    } else {
      Trans2Lanemarker(*FindById(lane_lines, it->first), &right_lanemarker);
      lane_width = start_pre_c0 - it->second;
      ADEBUG << " ............determined by  it->first. "
             << right_lanemarker.line_seq() << " ,lane_width: " << lane_width;
    }
    pre_lane.central_lanemarker = right_lanemarker;
    pre_lane.central_lanemarker.set_c0_position(
        start_left_lanemarker.c0_position() - lane_width / 2);
    ADEBUG << "Central_lanemarker C0_position:"
           << start_left_lanemarker.c0_position() - lane_width / 2;
    pre_lane.pre_c0 = start_pre_c0 - lane_width / 2;
    ADEBUG << "pre_c0: " << pre_lane.pre_c0;
    pre_lane.left_track_id = start_left_lanemarker.line_seq();
    pre_lane.right_track_id = right_lanemarker.line_seq();
    pre_lane.lane_width = lane_width;
    pre_lane.view_range = max_start_x;
    pre_lanes_.emplace_back(pre_lane);

    start_pre_c0 -= lane_width;
    ADEBUG << " ............start_pre_c0: " << start_pre_c0;
    start_left_lanemarker = right_lanemarker;
  }

  // for (int i = 0; i < pre_lanes_.size(); ++i) {
  //   ADEBUG << " pre_laneline index[" << i
  //          << "],left_track_id : " << pre_lanes_[i].left_track_id
  //          << " , right_track_id :" << pre_lanes_[i].right_track_id
  //          << " ,c0: " << pre_lanes_[i].pre_c0;
  // }
}

bool ObsHisPointsGenerator::DealNoObsLaneLine(
    const std::shared_ptr<LocalView>& local_view) {
  bool has_lanemarker = false;
  if (local_view->HasLaneMarkers() &&
      local_view->GetLaneMarkers()->has_front_right_lane_marker() &&
      local_view->HasLaneMarkers() &&
      local_view->GetLaneMarkers()->has_front_left_lane_marker()) {
    const auto& left_lanemarker =
        local_view->GetLaneMarkers()->front_left_lane_marker();
    const auto& right_lanemarker =
        local_view->GetLaneMarkers()->front_right_lane_marker();
    bool is_viewrange_fit = left_lanemarker.longitude_end() < 35.0 &&
                            right_lanemarker.longitude_end() < 35.0 &&
                            (left_lanemarker.longitude_end() > 5.0 ||
                             right_lanemarker.longitude_end() > 5.0) &&
                            left_lanemarker.longitude_start() < 5.0 &&
                            right_lanemarker.longitude_start() < 5.0;
    double right_c0 = right_lanemarker.c0_position();
    double left_c0 = left_lanemarker.c0_position();
    double right_c1 = left_lanemarker.c1_heading_angle();
    double left_c1 = right_lanemarker.c1_heading_angle();
    bool is_c0_fit = left_c0 > 1.0 && right_c0 < -1.0 &&
                     std::fabs(left_c1) < 0.05 && std::fabs(right_c1) < 0.05 &&
                     std::fabs(right_c1 - left_c1) < 0.05;
    has_lanemarker = is_viewrange_fit && is_c0_fit;
  }
  // 0:init  1: start using laneline 2. using history for 20m
  ADEBUG << " no_obs_lane_line_state: " << no_obs_lane_line_state_
         << " ,has_lanemarker: " << has_lanemarker
         << " ,length: " << no_obs_total_length_;
  switch (no_obs_lane_line_state_) {
    case 0:
      if (has_lanemarker) {
        no_obs_lane_line_state_ = 1;
        GenerateNoObsLanemarker(local_view);
      }
      no_obs_total_length_ = 0.0;
      no_obs_total_time_ = 0.0;
      break;

    case 1:
      if (has_lanemarker) {
        GenerateNoObsLanemarker(local_view);
      } else {
        TranslateHisPoints(&no_obs_lanemarker_, &no_obs_total_length_,
                           &no_obs_total_time_);
        no_obs_lane_line_state_ = 2;
      }
      break;

    case 2:
      if (no_obs_total_length_ > 25.0 || missile_mode_state_ != 1 ||
          vehicle_state_->is_only_acc_active() ||
          !vehicle_state_->is_missile_mode_active()) {
        no_obs_total_length_ = 0.0;
        no_obs_total_time_ = 0.0;
        no_obs_lane_line_state_ = 0;
      }
      TranslateHisPoints(&no_obs_lanemarker_, &no_obs_total_length_,
                         &no_obs_total_time_);
      break;
    default:
      break;
  }
  return no_obs_lane_line_state_ != 0;
}

void ObsHisPointsGenerator::TranslateHisPoints(
    TL::perception::LaneMarker* lanemarker, double* totle_length,
    double* totle_time) {
  *totle_time += 0.1;
  *totle_length += vehicle_state_->spd() * 0.1;
  std::vector<Vec2d> input_points;
  input_points.reserve(obs_points_.size());
  for (const auto& point : obs_points_) {
    input_points.emplace_back(point.x(),
                              CalculateLanemarkerY(point.x(), *lanemarker));
  }
  std::vector<Vec2d> out_points;
  CoordTranslate2d(&out_points, input_points, vehicle_state_->dphi(),
                   vehicle_state_->dx(), vehicle_state_->dy_ctrv(), true);
  const int N = 3;
  std::vector<double> coff = FitPolynomial<N>(out_points);
  lanemarker->set_c0_position(coff[0]);
  lanemarker->set_c1_heading_angle(coff[1]);
  lanemarker->set_c2_curvature(coff[2]);
  lanemarker->set_c3_curvature_derivative(coff[3]);
}

void ObsHisPointsGenerator::GenerateNoObsLanemarker(
    const std::shared_ptr<LocalView>& local_view) {
  const auto& left_lanemarker =
      local_view->GetLaneMarkers()->front_left_lane_marker();
  const auto& right_lanemarker =
      local_view->GetLaneMarkers()->front_right_lane_marker();
  double half_lane_width =
      (left_lanemarker.c0_position() - right_lanemarker.c0_position()) / 2;
  double delta_c0 = left_lanemarker.c0_position() - half_lane_width;
  double c0 =
      std::fabs(delta_c0) > 0.35 ? std::copysign(0.35, delta_c0) : delta_c0;
  double c1 = (left_lanemarker.c1_heading_angle() +
               right_lanemarker.c1_heading_angle()) /
              2;
  c1_heading_delay_.Deal(c1);
  no_obs_lanemarker_.set_c0_position(c0);
  no_obs_lanemarker_.set_c1_heading_angle(c1_heading_delay_.GetAverageValue());
  no_obs_lanemarker_.set_c2_curvature(
      (left_lanemarker.c2_curvature() + right_lanemarker.c2_curvature()) / 2);
  no_obs_lanemarker_.set_c3_curvature_derivative(
      (left_lanemarker.c3_curvature_derivative() +
       right_lanemarker.c3_curvature_derivative()) /
      2);
  no_obs_lanemarker_.set_longitude_start(left_lanemarker.longitude_start());
  no_obs_lanemarker_.set_longitude_end(left_lanemarker.longitude_end());
  half_lane_width_ = half_lane_width;
}

void ObsHisPointsGenerator::GenerateAllLanemarker(
    const TL::perception::LaneMarker& left_lane_marker,
    const TL::perception::LaneMarker& right_lane_marker) {
  auto left_lanemarker = left_lane_marker;
  auto right_lanemarker = right_lane_marker;
  double half_lane_width{3.2};
  std::vector<Vec2d> left_points;
  std::vector<Vec2d> right_points;
  if (left_lanemarker.longitude_start() > 5.0 &&
      right_lanemarker.longitude_start() > 5.0) {
    double min_longitude_start = std::max(left_lanemarker.longitude_start(),
                                          right_lanemarker.longitude_start());
    double left_longitude_start_c0 =
        CalculateLanemarkerY(min_longitude_start, left_lanemarker);
    double right_longitude_start_c0 =
        CalculateLanemarkerY(min_longitude_start, right_lanemarker);
    half_lane_width = (left_longitude_start_c0 - right_longitude_start_c0) / 2;
    double left_c0 = left_longitude_start_c0;
    double right_c0 = right_longitude_start_c0;
    if (left_longitude_start_c0 - half_lane_width > 0.3 &&
        right_longitude_start_c0 + half_lane_width > 0.3) {
      left_c0 = half_lane_width + 0.15;
      right_c0 = -half_lane_width + 0.15;
    } else if (left_longitude_start_c0 - half_lane_width < -0.3 &&
               right_longitude_start_c0 + half_lane_width < -0.3) {
      left_c0 = half_lane_width - 0.15;
      right_c0 = -half_lane_width - 0.15;
    }

    for (int i = -20; i < 0; i++) {
      left_points.emplace_back(i * 1.0, left_c0);
      right_points.emplace_back(i * 1.0, right_c0);
    }
    int left_length = std::ceil(left_lanemarker.longitude_end() -
                                left_lanemarker.longitude_start());
    for (int i = 0; i < left_length; i++) {
      double x = left_lanemarker.longitude_start() + i * 1.0;
      left_points.emplace_back(x, CalculateLanemarkerY(x, left_lanemarker));
    }
    int right_length = std::ceil(right_lanemarker.longitude_end() -
                                 right_lanemarker.longitude_start());
    for (int i = 0; i < right_length; i++) {
      double x = right_lanemarker.longitude_start() + i * 1.0;
      right_points.emplace_back(x, CalculateLanemarkerY(x, right_lanemarker));
    }
    const int N = 3;
    std::vector<double> left_coff = FitPolynomial<N>(left_points);
    left_lanemarker.set_c0_position(left_coff[0]);
    left_lanemarker.set_c1_heading_angle(left_coff[1]);
    left_lanemarker.set_c2_curvature(left_coff[2]);
    left_lanemarker.set_c3_curvature_derivative(left_coff[3]);
    ADEBUG << " left_coff[0]: " << left_coff[0] << " , [1]: " << left_coff[1]
           << " , [2]: " << left_coff[2] << " , [3]: " << left_coff[3];
    std::vector<double> right_coff = FitPolynomial<N>(right_points);
    right_lanemarker.set_c0_position(right_coff[0]);
    right_lanemarker.set_c1_heading_angle(right_coff[1]);
    right_lanemarker.set_c2_curvature(right_coff[2]);
    right_lanemarker.set_c3_curvature_derivative(right_coff[3]);
    ADEBUG << " right_coff[0]: " << right_coff[0] << " , [1]: " << right_coff[1]
           << " , [2]: " << right_coff[2] << " , [3]: " << right_coff[3];
  } else {
    half_lane_width =
        (left_lanemarker.c0_position() - right_lanemarker.c0_position()) / 2;
  }
  double delta_c0 = left_lanemarker.c0_position() - half_lane_width;
  double c0 =
      std::fabs(delta_c0) > 0.35 ? std::copysign(0.35, delta_c0) : delta_c0;
  laneline_lanemarker_.set_c0_position(c0);
  laneline_lanemarker_.set_c1_heading_angle(
      (left_lanemarker.c1_heading_angle() +
       right_lanemarker.c1_heading_angle()) /
      2);
  laneline_lanemarker_.set_c2_curvature(
      (left_lanemarker.c2_curvature() + right_lanemarker.c2_curvature()) / 2);
  laneline_lanemarker_.set_c3_curvature_derivative(
      (left_lanemarker.c3_curvature_derivative() +
       right_lanemarker.c3_curvature_derivative()) /
      2);
  laneline_lanemarker_.set_longitude_start(-20.0);
  laneline_lanemarker_.set_longitude_end(left_lanemarker.longitude_end());
  ADEBUG << " END_coff[0]: " << laneline_lanemarker_.c0_position()
         << " , [1]: " << laneline_lanemarker_.c1_heading_angle()
         << " , [2]: " << laneline_lanemarker_.c2_curvature()
         << " , [3]: " << laneline_lanemarker_.c3_curvature_derivative();
  half_lane_width_ = half_lane_width;
}

void ObsHisPointsGenerator::GenerateOneLanemarker(
    const TL::perception::LaneMarker& lanemarker, int is_left_lanemarker) {
  double longitude_start = lanemarker.longitude_start();
  if (longitude_start > 5) {
    double longitude_start_c0 =
        CalculateLanemarkerY(longitude_start, lanemarker);
    double c0 = longitude_start_c0;
    if (longitude_start_c0 - half_lane_width_ * is_left_lanemarker > 0.3) {
      c0 = half_lane_width_ * is_left_lanemarker + 0.15;
    } else if (longitude_start_c0 - half_lane_width_ * is_left_lanemarker <
               -0.3) {
      c0 = half_lane_width_ * is_left_lanemarker - 0.15;
    }
    std::vector<Vec2d> points;
    for (int i = -20; i < 0; i++) {
      points.emplace_back(i * 1.0, c0);
    }
    int left_length =
        std::ceil(lanemarker.longitude_end() - lanemarker.longitude_start());
    for (int i = 0; i < left_length; i++) {
      double x = longitude_start + i * 1.0;
      points.emplace_back(x, CalculateLanemarkerY(x, lanemarker));
    }
    const int N = 3;
    std::vector<double> left_coff = FitPolynomial<N>(points);
    double delta_c0 = left_coff[0] - half_lane_width_ * is_left_lanemarker;
    double c0_out =
        std::fabs(delta_c0) > 0.35 ? std::copysign(0.35, delta_c0) : delta_c0;
    laneline_lanemarker_.set_c0_position(c0_out);
    laneline_lanemarker_.set_c1_heading_angle(left_coff[1]);
    laneline_lanemarker_.set_c2_curvature(left_coff[2]);
    laneline_lanemarker_.set_c3_curvature_derivative(left_coff[3]);
    laneline_lanemarker_.set_longitude_start(lanemarker.longitude_start());
    laneline_lanemarker_.set_longitude_end(lanemarker.longitude_end());
  } else {
    double delta_c0 =
        lanemarker.c0_position() - half_lane_width_ * is_left_lanemarker;
    double c0 =
        std::fabs(delta_c0) > 0.35 ? std::copysign(0.35, delta_c0) : delta_c0;
    laneline_lanemarker_.set_c0_position(c0);
    laneline_lanemarker_.set_c1_heading_angle(lanemarker.c1_heading_angle());
    laneline_lanemarker_.set_c2_curvature(lanemarker.c2_curvature());
    laneline_lanemarker_.set_c3_curvature_derivative(
        lanemarker.c3_curvature_derivative());
    laneline_lanemarker_.set_longitude_start(lanemarker.longitude_start());
    laneline_lanemarker_.set_longitude_end(lanemarker.longitude_end());
  }
}

void ObsHisPointsGenerator::DataPreprocess(
    const TL::perception::PerceptionObstacle& obs) {
  obs_x_ = obs.position_flu().x();
  obs_vx_ = obs.velocity_flu().x();
  ego_vehspd_ = vehicle_state_->spd();
  ego_yawtate_ = vehicle_state_->yaw_rate();
  obs_id_ = obs.id();
  // obs_y目标横向距离滤波
  obs_lat_y_filter_.SetCoefficientAndFlag(0.5, obs_id_ != obs_id_last_);
  obs_y_ = obs_lat_y_filter_.Filter(obs.position_flu().y());
  // 判断跟车目标是否在区域内
  is_target_in_ = JudgeTrgtVehIN(obs);
  double obs_theta = std::atan2(obs_y_, obs_x_);
  vehicle_state_->local_view_data()
      ->update_data()
      ->set_adc_target_obs_flupos_heading(obs_theta);
  ADEBUG << "is_target_in: " << is_target_in_ << " ,obs_x: " << obs_x_
         << " , obs_y: " << obs_y_ << " , obs_vx: " << obs_vx_
         << " , ego_vehspd: " << ego_vehspd_
         << " , ego_yawtate: " << ego_yawtate_ << " obs_theta: " << obs_theta;
}

void ObsHisPointsGenerator::DealBackPoints() {
  int obs_points_size = obs_points_.size();  // NOLINT
  int index{obs_points_size};
  for (int i = obs_points_size - 1; i > 0; i--) {
    if (obs_points_[i].x() < 0.1) {
      index = i;
    } else {
      break;
    }
  }
  obs_points_.erase(obs_points_.begin(), obs_points_.end() - index);
  while (obs_points_.front().x() > -20) {
    obs_points_.insert(obs_points_.begin(), {obs_points_.front().x() - 1.0,
                                             obs_points_.front().y()});
  }

  ADEBUG << "end obs_y_ / obs_x_: " << obs_y_ / obs_x_
         << " ,obs_points_.front().y(): " << std::fabs(obs_points_.front().y())
         << " , obs_y: " << std::fabs(obs_y_);
  if (std::fabs(obs_points_.front().y()) > 0.36 &&
      (obs_y_ / obs_x_ > 0.067 || std::fabs(obs_y_) > 2.0 ||
       std::fabs(obs_points_.front().y()) > 1.5)) {
    if (obs_points_.front().y() > 0.01 && obs_y_ > -0.8 &&
        obs_points_.front().y() - obs_y_ > 1.0) {
      for (auto& point : obs_points_) {
        point.set_y(0.36);
      }
      return;
    }
    if (obs_points_.front().y() < -0.01 && obs_y_ < 0.8 &&
        obs_points_.front().y() - obs_y_ < -1.0) {
      for (auto& point : obs_points_) {
        point.set_y(-0.36);
      }
      return;
    }

    double delta_y =
        obs_points_.front().y() - std::copysign(0.36, obs_points_.front().y());
    // AERROR << "delta_y: " << delta_y;
    for (auto& point : obs_points_) {
      double y = point.y() - delta_y;
      point.set_y(y);
    }
  }
}

void ObsHisPointsGenerator::GenerateObsBackPoints(
    const TL::perception::PerceptionObstacle& obs) {
  // step1:第一个调度模块TrgtVehTrajCaptureEvtGen()，更新目标后方的历史轨迹点
  // tuple(0:dx_out, 1:dphi_out, 2:buff_rest_flag, 3:do_TrgtVehHistPath_Calc,4.do_TrgtVehHistPath_RTU);
  UNUSED(obs);
  static double dphi_out = 0;
  static double dx_out = 0;
  static double obs_in_lon_dis = 0;
  static double obs_dx = 0.0;
  bool buff_rest_flag = false;
  // double delta_d = 0.4;
  // 初始化 数组
  static std::deque<double> init_delta_arr(19, 0);
  // AERROR "init_delta_arr[0]:" << init_delta_arr[0]

  double speed = std::fabs(ego_vehspd_) < 0.2 ? 0 : ego_vehspd_;
  double yaw_rate = std::fabs(ego_vehspd_) < 0.2 ? 0 : ego_yawtate_;
  dx_out += speed * kTJAMainLoopTimeSec;
  dphi_out += yaw_rate * kTJAMainLoopTimeSec;
  obs_dx += obs_vx_ * kTJAMainLoopTimeSec;
  ADEBUG << "current_points_state: " << current_points_state_;
  ADEBUG << "dx_out : " << dx_out << " ,min_dis: " << obs_x_ / 19
         << " ,obs_dx: " << obs_dx;
  // auto pilot_state = fct_in_->fct_nnp_in().npilot_state();
  // bool is_nolane_active =
  //     history_perception_sub_state_ == functionmanager::NOLANE_TYPE &&
  //     pilot_state == FctToNnpInput::PILOT_SUSPEND &&
  //     pilot_state == FctToNnpInput::PILOT_ACTIVE;
  // 根据当前状态进行相应的操作
  switch (current_points_state_) {
    case Obs_Out:
      // 状态1：OBJ_OUT1
      if (obs_x_ > kTJATvdpGminDistFol) {
        obs_dx = 0.0;
        ObsInStateAction(&init_delta_arr, &dx_out, &dphi_out, &obs_in_lon_dis,
                         is_target_in_);
      } else {
        init_delta_arr.resize(19, 0);
        dx_out = 0;
        dphi_out = 0;
        obs_dx = 0.0;
        buff_rest_flag = false;
      }
      break;

    case Obs_In_LowSpd:
      // 状态2的子状态 a:integral_staticobj
      ADEBUG << "----LowSpd update---";
      obs_dx = 0.0;
      ObsInStateAction(&init_delta_arr, &dx_out, &dphi_out, &obs_in_lon_dis,
                       is_target_in_);
      if (obs_vx_ <= 1.5) {
        current_points_state_ = Obs_In_LowSpd;
      }
      break;
    case Obs_In_HighSpd:
      // 状态2的子状态 b:integral
      if (obs_x_ <= 0.1) {
        current_points_state_ = Obs_Out;
        init_delta_arr.resize(19, 0);
        dx_out = 0;
        dphi_out = 0;
        obs_dx = 0.0;
        buff_rest_flag = false;
        break;
      }
      if (obs_vx_ <= 1) {
        current_points_state_ = Obs_In_LowSpd;
        dphi_out = 0;
        dx_out = 0;
        obs_dx = 0.0;
        break;
      }
      if (is_obj_id_changed_bl_) {
        ObsInStateAction(&init_delta_arr, &dx_out, &dphi_out, &obs_in_lon_dis,
                         is_target_in_);
        break;
      }
      if (obs_dx > (obs_x_ / 19) * 0.7) {
        ADEBUG << "----HighSpd update---";
        double delta_d =
            std::max((obs_x_ - obs_in_lon_dis + init_delta_arr.front()), 0.4);
        // AERROR << "delta_d: " << delta_d
        //        << " dx_out_highspd_update: " << dx_out;
        init_delta_arr.pop_front();
        init_delta_arr.emplace_back(delta_d);
        obs_in_lon_dis = obs_x_;
        DoVehHistPntsBuffUpdate(dx_out, vehicle_state_->dy_ctrv(), dphi_out,
                                buff_rest_flag, is_target_in_);
        dphi_out = 0;
        dx_out = 0;
        obs_dx = 0.0;
      } else {
        ADEBUG << "----HighSpd--CoordTranslate2d---";
        // AERROR << "dx_out_highSpd_RTU: " << dx_out
        //        << " ,dphi_out_highSpd_RTU: " << dphi_out;
        CoordTranslate2d(&hist_points_buffer_, hist_points_buffer_update_,
                         dphi_out, dx_out, vehicle_state_->dy_ctrv(), true);
      }
      break;
  }
  // AERROR << "dx_out: " << dx_out << " ,dphi_out: " << dphi_out
  //        << "obs_x_ / 19:" << obs_x_ / 19;
}

void ObsHisPointsGenerator::ObsInStateAction(std::deque<double>* init_delta,
                                             double* dx, double* dphi,
                                             double* dis, bool is_target_in) {
  current_points_state_ = Obs_In_HighSpd;
  *dx = 0.0;
  *dphi = 0.0;
  *dis = obs_x_;
  init_delta->resize(19, *dis / 19);
  DoVehHistPntsBuffUpdate(0.0, 0.0, 0.0, true, is_target_in);
}

//  do_TrgtVehHistPath_Calc 此处为第一个调度模块的triger1，目标历史轨迹点buffer更新
bool ObsHisPointsGenerator::DoVehHistPntsBuffUpdate(double dx, double dy,
                                                    double dphi, bool rest_flag,
                                                    bool is_target_in) {
  if (is_target_in) {
    if (rest_flag) {
      hist_points_buffer_.clear();
      BufferReset(&hist_points_buffer_update_, obs_x_, obs_y_,
                  kODPRnVehTrajBuffSize, average_heading_);  // 初始化
      hist_points_buffer_ = hist_points_buffer_update_;
      // AERROR << "back_point_buffer_has_reset!!!";
      return true;
    }
    CoordTranslate2d(&hist_points_buffer_, hist_points_buffer_update_, dphi, dx,
                     dy, true);
    hist_points_buffer_.pop_front();
    hist_points_buffer_.emplace_back(obs_x_, obs_y_);
    hist_points_buffer_update_ = hist_points_buffer_;
  } else {
    current_points_state_ = Obs_Out;
    std::deque<Vec2d> tmp_points{20, Vec2d(0, 0)};
    hist_points_buffer_.swap(tmp_points);
    hist_points_buffer_update_ = hist_points_buffer_;
  }
  return true;
}

// bufferreset Linspace求初始点
void ObsHisPointsGenerator::BufferReset(std::deque<Vec2d>* hist_points_buffer,
                                        double obs_x, double obs_y,
                                        int num_history_buff, double heading) {
  double delta_x = std::cos(heading);
  double delta_y = std::sin(heading);
  std::vector<Vec2d> init_points;
  init_points.reserve(40);
  for (int i = -20; i < 0; i++) {
    init_points.emplace_back(i * 1.0, 0.0);
  }
  double init_x{obs_x};
  double init_y{obs_y};
  for (int i = 0; i < 20; i++) {
    init_points.emplace_back(init_x, init_y);
    init_x += delta_x;
    init_y += delta_y;
  }
  const int N = 3;
  std::vector<double> coff = FitPolynomial<N>(init_points);
  double interval = obs_x / (num_history_buff - 1);
  hist_points_buffer->clear();  // 预分配足够的内存， 20个点
  double x_2{0.0};
  for (int i = 0; i < num_history_buff; i++) {
    double x = interval * i;
    x_2 = x * x;
    double y = coff[0] + coff[1] * x + coff[2] * x_2 + coff[3] * x_2 * x;
    hist_points_buffer->emplace_back(x, y);
  }
}

// bufferreset Linspace求初始点
void ObsHisPointsGenerator::BufferReset(std::deque<Vec2d>* hist_points_buffer,
                                        double obs_x, double obs_y,
                                        int num_history_buff) {
  double interval = obs_x / (num_history_buff - 1);
  hist_points_buffer->clear();  // 预分配足够的内存， 20个点
  for (int i = 0; i < num_history_buff; i++) {
    hist_points_buffer->emplace_back(interval * i, obs_y);
  }
}

void ObsHisPointsGenerator::GenerateObsHeadingAndCurve(
    const TL::perception::PerceptionObstacle& obs) {
  UNUSED(obs);
  const auto& [dx_out, dphi_out, reset_flag, is_do_vehicletrajpredict_bl] =
      Timer_Scheduler();
  // AERROR << "dx_out: " << dx_out << " , dphi_out: " << dphi_out
  //        << " , reset_flag: " << reset_flag
  //        << " , is_do_vehicletrajpredict_bl: " << is_do_vehicletrajpredict_bl
  //        << " , is_target_in_: " << is_target_in_;
  if (is_do_vehicletrajpredict_bl) {
    is_avgtrnd_buff_full_bl_ =
        DoVehicleTrajectoryPredict(dx_out, dphi_out, reset_flag, is_target_in_);
  }
  // AERROR << " hist_points_buffer_for_heading_.size: "
  //        << hist_points_buffer_for_heading_.size()
  //        << " , is_buff_full_bl: " << is_avgtrnd_buff_full_bl;
  int avge_size = hist_points_buffer_for_heading_.size();  // NOLINT
  Matrix average_move_trend_points_x = Matrix::Zero(1, std::max(avge_size, 20));
  Matrix average_move_trend_points_y = Matrix::Zero(1, std::max(avge_size, 20));
  for (int i = 0; i < static_cast<int>(hist_points_buffer_for_heading_.size());
       i++) {
    (average_move_trend_points_x)(0, i) =
        hist_points_buffer_for_heading_.at(i).x();
    (average_move_trend_points_y)(0, i) =
        hist_points_buffer_for_heading_.at(i).y();
    // 顺便打印buffer_for_heading轨迹点
    ADEBUG << "hist_points_buffer_for_heading[" << i
           << "]_x: " << average_move_trend_points_x(0, i)
           << " y: " << average_move_trend_points_y(0, i);
  }
  // 目标运动合理性判断（主要对目标是否静止/轨迹长度/heading标准差等做校验）
  bool is_est_trgt_mvtrnd_not_reliable = TrgtVehMvTrdStsReliable(
      obs_vx_, average_move_trend_points_x, average_move_trend_points_y);
  //   Func_Gain*  对heading/curve 做ramp、 limit
  double ramp_risetime = 1 / kODPRtTrgtVehMoveRiseTime;
  double ramp_falltime = -1 / kODPRtrgtVehMoveFallTime;

  int func_status = is_est_trgt_mvtrnd_not_reliable ? 0 : 1;
  double func_gain = 0;
  // AERROR << "is_est_trgt_mvtrnd_not_reliable: "
  // << is_est_trgt_mvtrnd_not_reliable;
  EnableDynamicRateLimiter(func_status, 0, ramp_risetime, ramp_falltime,
                           kTJAMainLoopTimeSec, &func_gain);
  // AERROR << "func_gain: " << func_gain;

  if (fLB::FLAGS_not_using_nolane_func_gain) {
    func_gain = 1;
  }
  // heading
  average_heading_ = CalculateAverageHeading(&average_move_trend_points_x,
                                             &average_move_trend_points_y) *
                     func_gain;
  // curve
  average_curve_ = CalculateAvgMoveCurvature(average_move_trend_points_x,
                                             average_move_trend_points_y) *
                   func_gain;
  if (!is_avgtrnd_buff_full_bl_) {
    average_heading_ = 0.0;
    average_curve_ = 0.0;
  }

  ADEBUG << "average_heading_after_ramp: "
         << average_heading_
         //  << " , history heading: " << history_heading_ << " ,"
         << "average_curve_" << average_curve_;
}

std::tuple<double, double, bool, bool>
ObsHisPointsGenerator::Timer_Scheduler() {
  double dx_out = 0;
  double dphi_out = 0;
  static double dx = 0;
  static double dphi = 0;
  static int i = 0;
  static bool reset_flag = false;
  bool is_do_vehicletrajpredict_bl{false};
  // // 为了解决长时间跟同一前车坐标转换问题，修改调度，每3s，buffer reset
  // int max_i = std::fabs(ego_vehspd_) > 7 ? 20 : 30;
  // || i > max_i
  if (is_obj_id_changed_bl_) {
    reset_flag = true;
    i = 0;
    dx = 0;
    dphi = 0;
    dx_out = dx;
    dphi_out = dphi;
    is_do_vehicletrajpredict_bl = true;
  } else {
    reset_flag = false;
    i = i + 1;
    dx += vehicle_state_->dx();
    dphi += vehicle_state_->dphi();
  }

  if (i == 255) {
    i = 2;
  }

  if (i >= kTimerSchedulerStep && dx > 0.5) {
    dx_out = dx;
    dphi_out = dphi;
    // i = 0;
    dx = 0;
    dphi = 0;
    is_do_vehicletrajpredict_bl = true;
  }
  return std::make_tuple(dx_out, dphi_out, reset_flag,
                         is_do_vehicletrajpredict_bl);
}

// 更新轨迹点buffer，后续计算heading做预测使用。得到的轨迹点方法与step1一致，不同的是更新的时间间隔
bool ObsHisPointsGenerator::DoVehicleTrajectoryPredict(double dx, double dphi,
                                                       bool reset_flag,
                                                       bool is_target_in) {
  Matrix buffer_point = Matrix::Zero(3, 20);  // 初始轨迹点buffer
  bool is_buffer_full{true};
  if (is_target_in) {
    if (reset_flag) {
      BufferReset(&hist_points_buffer_for_heading_, obs_x_, obs_y_,
                  kODPRnVehTrajBuffSize, average_heading_);
      return is_buffer_full;
    }
    // AERROR << "Predict 111 hist_points_buffer_for_heading_.size: "
    //        << hist_points_buffer_for_heading_.size();
    for (int i = 0;
         i < static_cast<int>(hist_points_buffer_for_heading_.size()); i++) {
      buffer_point(0, i) = hist_points_buffer_for_heading_.at(i).x();
      buffer_point(1, i) = hist_points_buffer_for_heading_.at(i).y();
      buffer_point(2, i) = 1;
    }
    // 转换为当前时刻的点
    buffer_point = vehicle_state_->TransMatrix(dx, dphi) * buffer_point;
    // AERROR << "buffer_point: " << buffer_point.cols();
    // hist_points_buffer_for_heading_.reserve(
    //     NumHistBuff);  // 预分配足够的内存， 20个点
    for (int i = 0;
         i < static_cast<int>(hist_points_buffer_for_heading_.size()); i++) {
      hist_points_buffer_for_heading_[i].set_x(buffer_point(0, i));
      hist_points_buffer_for_heading_[i].set_y(buffer_point(1, i));
    }
    hist_points_buffer_for_heading_.pop_front();
    hist_points_buffer_for_heading_.emplace_back(obs_x_, obs_y_);
  } else {
    std::deque<Vec2d> tmp_points{20, Vec2d(0, 0)};
    hist_points_buffer_for_heading_.swap(tmp_points);
    is_buffer_full = false;
  }
  return is_buffer_full;
}

// 目标运动有效性检查 返回true表示不通过
bool ObsHisPointsGenerator::TrgtVehMvTrdStsReliable(
    double obs_lon_spd, const Eigen::MatrixXd& hist_points_x,
    const Eigen::MatrixXd& hist_points_y) {
  static bool is_trgtveh_stdstill = true;  // 默认目标静止
  if (abs(obs_lon_spd) >= kODPRvObjStdStillDisenSpd) {
    is_trgtveh_stdstill = false;
  }
  if (abs(obs_lon_spd) <= kODPRvObjStdStillEnSpd) {
    is_trgtveh_stdstill = true;
  }

  bool is_histtrajlength_ok =
      hist_points_x(0, 19) - hist_points_x.minCoeff() < kODPRdHistTrajLngthThd;
  // X[20] - min_x >k_ODPR_d_HistTrajLngthThd_sg = 1m;
  is_stand_deviation_nok_ =
      k_standard_deviation_filter_.Filter(CalculateStandDeviation(
          hist_points_x, hist_points_y)) >= kODPRkAvgSlopStdDvton;
  ADEBUG << "is_stand_deviation_nok_" << is_stand_deviation_nok_;
  return est_trgmovetrend_not_relible_debounce_.DealDebounce(
      is_histtrajlength_ok || is_trgtveh_stdstill || is_stand_deviation_nok_);
}

// 最小二乘法计算目标历史轨迹平均heading(rad)
double ObsHisPointsGenerator::CalculateAverageHeading(
    const Eigen::MatrixXd* hist_points_x,
    const Eigen::MatrixXd* hist_points_y) {
  if (hist_points_x->size() == 0) {
    return 0;
  }
  // 计算 x_m 和 y_m
  double x_m = hist_points_x->mean();
  double y_m = hist_points_y->mean();

  // 计算 b 和 a
  double sum_XY = (hist_points_x->array() * hist_points_y->array()).sum();
  double sum_X2 = (hist_points_x->array() * hist_points_x->array()).sum();
  if (std::abs(sum_X2 - kODPRnVehTrajBuffSize * x_m * x_m) < 1e-6) {
    // Return a default value or handle the error as appropriate
    return 0;  //  return 0.0 as a default slope
  }

  double b = (sum_XY - kODPRnVehTrajBuffSize * x_m * y_m) /
             (sum_X2 - kODPRnVehTrajBuffSize * x_m * x_m);
  double a = y_m - b * x_m;
  UNUSED(a);
  // AERROR << "heading:" << b;
  return SaturationDynamicLimit(avg_trgveh_move_heading_filter_.Filter(b), -1,
                                1);
}

// Curvature计算：k = ddy/((1+dy^2)^1.5) 待加低通滤波 ,最终也需要根据is_EstTrgtMvTrndNotRlby_bl结果进行ramp和限幅
double ObsHisPointsGenerator::CalculateAvgMoveCurvature(
    const Eigen::MatrixXd& hist_points_x,
    const Eigen::MatrixXd& hist_points_y) {
  const double kDenomDiffThreshold = 0.001;
  auto size = hist_points_x.size();
  Eigen::VectorXd hist_points_yy(size);
  Eigen::VectorXd hist_points_xx(size);
  for (int i = 0; i < size; ++i) {
    hist_points_xx[i] = hist_points_x(0, i);
    hist_points_yy[i] = hist_points_y(0, i);
  }
  // 计算一阶导数
  Eigen::ArrayXd denom_diff =
      (hist_points_xx.tail(size - 1) - hist_points_xx.head(size - 1)).array();
  denom_diff = (denom_diff.abs() < 0.01).select(0.01, denom_diff);
  Eigen::VectorXd num =
      (hist_points_yy.tail(size - 1) - hist_points_yy.head(size - 1)).array() /
      denom_diff;
  // for (int i = 0; i < denom_diff.size(); ++i) {
  //   AERROR << denom_diff[i] << " ";
  // }

  // for (int i = 0; i < num.size(); ++i) {
  //   AERROR << " [" << i << "]: " << num[i];
  // }
  double dy = num.sum() / (size - 1);  // NOLINT
  // AERROR << "dy: " << dy << " , sum: " << num.sum() << " , size: " << size;
  // 计算二阶导数
  Eigen::ArrayXd den =
      ((hist_points_xx.tail(size - 1) + hist_points_xx.head(size - 1)) * 0.5)
          .array();
  Eigen::ArrayXd den_diff =
      (den.tail(den.size() - 1) - den.head(den.size() - 1)).array();
  double ddy_denom = den_diff.array().sum() / (size - 2);  // NOLINT
  double ddy_den =
      (num.tail(num.size() - 1) - num.head(num.size() - 1)).array().sum() /
      (size - 2);  // NOLINT
  double ddy =
      trgveh_curve_ddy_filter_.Filter(ddy_den) /
      (abs(ddy_denom) > kDenomDiffThreshold ? ddy_denom : kDenomDiffThreshold);
  // 计算曲率
  double curvature = ddy / std::pow((1 + dy * dy), 1.5);
  // AERROR << ", dy: " << ddy_den << " ,ddy_den_diff: " << ddy_denom
  //        << " ,ddy_den_filter:" << (trgveh_curve_ddy_filter_.Filter(ddy_den))
  //        << " , ddy: " << ddy << " , curvature: " << curvature;
  //  AvgMoveTrnd_Curvature
  return SaturationDynamicLimit(
      avg_trgveh_move_curvature_filter_.Filter(curvature), -0.1, 0.1);
}

// standard_deviation_k计算历史轨迹点标准差
// Caculate the Stand Devation Value of the Points slopes sets current
// if Stand Devation > 0.02,the move trend is not reliable.
double ObsHisPointsGenerator::CalculateStandDeviation(
    const Eigen::MatrixXd& hist_points_x,
    const Eigen::MatrixXd& hist_points_y) {

  auto size = hist_points_x.size();
  Eigen::VectorXd hist_points_yy(size);
  Eigen::VectorXd hist_points_xx(size);
  for (int i = 0; i < size; ++i) {
    hist_points_xx[i] = hist_points_x(0, i);
    hist_points_yy[i] = hist_points_y(0, i);
  }
  // 计算斜率
  Eigen::VectorXd slopes(size - 1);
  for (int m = 0; m < (size - 1); ++m) {
    slopes[m] = (hist_points_yy[m + 1] - hist_points_yy[m]) /
                std::max((hist_points_xx[m + 1] - hist_points_xx[m]), 0.01);
  }
  // for (int i = 0; i < slopes.size(); ++i) {
  //   AERROR << "slopes[" << i << "]: " << slopes[i];
  // }
  double dy = slopes.sum() / (size - 1);  // NOLINT
  Eigen::VectorXd diff_slopes = slopes.array() - dy;
  // 计算斜率的均值
  // double mean_slope = slopes.mean();
  // AERROR << "mean_slope: " << dy;
  // 计算斜率的平方差
  Eigen::VectorXd diff_slope = slopes.array() - dy;
  double var_slope = (diff_slope.array() * diff_slope.array()).sum() /
                     static_cast<double>(slopes.size() - 1);
  // AERROR << "var_slope: " << var_slope;
  // 计算标准差
  double std_dev_slope = std::sqrt(var_slope);
  // AERROR << "std_dev_slope: " << std_dev_slope;
  return std_dev_slope;
}

// 计算目标未来轨迹点，预测1s
bool ObsHisPointsGenerator::GenerateTrgtVehPreTrajPnts(
    Eigen::MatrixXd* pre_points_x, Eigen::MatrixXd* pre_points_y) {
  // double TJA_distEndPoint_m = 30;  已经做插值表
  if (!is_target_in_) {
    return false;
  }
  const int num_points_part1 = 11;
  const int num_points_part2 = 7;
  const int num_points_part3 = 4;
  double delta_s1 = obs_vx_ * kODPRtPntsFixPrvwTimeYaxs;
  double delta_s2 = delta_s1;
  double delta_s3 = fmax((2 * delta_s1 + obs_x_),
                         TJA_ODPR_dist_EndPoint_.Interpolate(ego_vehspd_)) +
                    3 - obs_x_ - 2 * delta_s1;
  // AERROR << "delta_s1:" << delta_s1 << ",delta_s2" << delta_s2
  //        << ",delta_s3:" << delta_s3;
  Matrix Preview_LinearPnts_S_X1{Matrix::Zero(1, 10)};
  Matrix Preview_LinearPnts_S_Y1{Matrix::Zero(1, 10)};
  Matrix Preview_LinearPnts_S_X2{Matrix::Zero(1, 6)};
  Matrix Preview_LinearPnts_S_Y2{Matrix::Zero(1, 6)};
  Matrix Preview_LinearPnts_S_X3{Matrix::Zero(1, 3)};
  Matrix Preview_LinearPnts_S_Y3{Matrix::Zero(1, 3)};
  Matrix Preview_LinearPnts_S_X1_2{Matrix::Zero(1, 10)};

  LinSpace(&Preview_LinearPnts_S_X1, delta_s1, num_points_part1);
  LinSpace(&Preview_LinearPnts_S_X2, delta_s2, num_points_part2);
  LinSpace(&Preview_LinearPnts_S_X3, delta_s3, num_points_part3);

  // AERROR << "Preview_LinearPnts_S_X1:" << Preview_LinearPnts_S_X1
  //        << ",S_X2:" << Preview_LinearPnts_S_X2
  //        << ",S_X3:" << Preview_LinearPnts_S_X3;
  // Calculate Y values for different parts
  for (int i = 0; i < Preview_LinearPnts_S_X1.cols(); ++i) {
    Preview_LinearPnts_S_X1_2(0, i) =
        Preview_LinearPnts_S_X1(0, i) * Preview_LinearPnts_S_X1(0, i);
  }
  Preview_LinearPnts_S_Y1 = Preview_LinearPnts_S_X1 * average_heading_ +
                            0.5 * average_curve_ * Preview_LinearPnts_S_X1_2;
  Preview_LinearPnts_S_Y2 = 0.5 * average_heading_ * Preview_LinearPnts_S_X2;
  Preview_LinearPnts_S_Y3 = 0.25 * average_heading_ * Preview_LinearPnts_S_X3;
  // AERROR << "Preview_LinearPnts_S_Y1:" << Preview_LinearPnts_S_Y1 << ",S_Y2"
  //        << Preview_LinearPnts_S_Y2 << ",S_Y3:" << Preview_LinearPnts_S_Y3;
  // Y 部分 Concatenate Y values of different parts
  Eigen::MatrixXd part1_y = Preview_LinearPnts_S_Y1.array() + obs_y_;
  Eigen::MatrixXd part2_y =
      part1_y(num_points_part1 - 2) + Preview_LinearPnts_S_Y2.array();
  Eigen::MatrixXd part3_y =
      part2_y(num_points_part2 - 2) + Preview_LinearPnts_S_Y3.array();
  // x 部分 Concatenate X values of different parts
  Eigen::MatrixXd part1_x = Preview_LinearPnts_S_X1.array() + obs_x_;
  Eigen::MatrixXd part2_x =
      part1_x(num_points_part1 - 2) + Preview_LinearPnts_S_X2.array();
  Eigen::MatrixXd part3_x =
      part2_x(num_points_part2 - 2) + Preview_LinearPnts_S_X3.array();
  // 创建新的矩阵，将三个部分拼接在一起
  Eigen::MatrixXd fixed_points_x(1, 19);  // 1×19
  Eigen::MatrixXd fixed_points_y(1, 19);  // 1×19
  // 目标车前方轨迹点y
  fixed_points_y.row(0).segment(0, num_points_part1 - 1) =
      part1_y;  // 10 ,from 0 to 9
  fixed_points_y.row(0).segment(10, num_points_part2 - 1) =
      part2_y;  // 6,from 10 to 15
  fixed_points_y.row(0).segment(16, num_points_part3 - 1) =
      part3_y;  // 3,from 16 to 18
  // 目标车前方轨迹点x
  fixed_points_x.row(0).segment(0, num_points_part1 - 1) =
      part1_x;  // 10 ,from 0 to 9
  fixed_points_x.row(0).segment(10, num_points_part2 - 1) =
      part2_x;  // 6,from 10 to 15
  fixed_points_x.row(0).segment(16, num_points_part3 - 1) =
      part3_x;  // 3,from 16 to 18
                // 打印目标前方所有点
  // AERROR << "目标当前位置:[_x" << obs_x_ << "],_y: " << obs_y_;
  // for (int i = 0; i < fixed_points_x.cols(); ++i) {
  //   AERROR << "fixed_points_x:[" << i << "]_x: " << fixed_points_x(0, i)
  //          << " y: " << fixed_points_y(0, i);
  // }
  Eigen::MatrixXd oddItemsX(1, 9);   // 存储奇数项的矩阵
  Eigen::MatrixXd evenItemsX(1, 9);  // 存储偶数项的矩阵
  // Eigen::MatrixXd resultMatrixX(1, 9);  // 存储奇数项和偶数项的平均值矩阵
  Eigen::MatrixXd oddItemsY(1, 9);   // 存储奇数项的矩阵
  Eigen::MatrixXd evenItemsY(1, 9);  // 存储偶数项的矩阵
  // Eigen::MatrixXd resultMatrixY(1, 9);  // 存储奇数项和偶数项的平均值矩阵
  int oddIndex = 0;   // 用于追踪奇数项数组的索引
  int evenIndex = 0;  // 用于追踪偶数项数组的索引

  // 使用for循环遍历原始矩阵，提取奇数项和偶数项
  for (int i = 2; i < 19; i += 2) {
    oddItemsX(0, oddIndex) = fixed_points_x(0, i);
    oddItemsY(0, oddIndex) = fixed_points_y(0, i);
    oddIndex++;
  }

  for (int i = 1; i < 19; i += 2) {
    evenItemsX(0, evenIndex) = fixed_points_x(0, i);
    evenItemsY(0, evenIndex) = fixed_points_y(0, i);
    evenIndex++;
  }

  // 计算奇数项和偶数项的平均值
  *pre_points_x = (oddItemsX + evenItemsX) / 2.0;  // 9个点
  *pre_points_y = (oddItemsY + evenItemsY) / 2.0;
  return true;
}

// 最终历史轨迹点和预测轨迹点的merge，前车后方轨迹点x：10，前车前方预测轨迹点x：9，第20个点x 填充0
void ObsHisPointsGenerator::TrgtVehTrajPntsMergeFix(
    const Eigen::MatrixXd& pre_points_x, const Eigen::MatrixXd& pre_points_y) {
  // const auto hist_points_size = hist_points_buffer_.size();
  size_t total_size = 20;
  // AERROR << "hist_points_size" << hist_points_size << ",total_size"
  //        << total_size;
  obs_points_.resize(total_size - 2);
  Eigen::MatrixXd oddItemsX(1, 10);   // 存储奇数项的矩阵
  Eigen::MatrixXd evenItemsX(1, 10);  // 存储偶数项的矩阵
  Eigen::MatrixXd resultMatrixX(1, 10);  // 存储奇数项和偶数项的平均值矩阵
  Eigen::MatrixXd oddItemsY(1, 10);   // 存储奇数项的矩阵
  Eigen::MatrixXd evenItemsY(1, 10);  // 存储偶数项的矩阵
  Eigen::MatrixXd resultMatrixY(1, 10);  // 存储奇数项和偶数项的平均值矩阵
  int oddIndex = 0;                      // 用于追踪奇数项数组的索引
  int evenIndex = 0;                     // 用于追踪偶数项数组的索引

  // 使用for循环遍历原始矩阵，提取奇数项和偶数项
  for (int i = 0; i < 20; i += 2) {
    oddItemsX(0, oddIndex) = hist_points_buffer_[i].x();
    oddItemsY(0, oddIndex) = hist_points_buffer_[i].y();
    oddIndex++;
  }

  for (int i = 1; i < 20; i += 2) {
    evenItemsX(0, evenIndex) = hist_points_buffer_[i].x();
    evenItemsY(0, evenIndex) = hist_points_buffer_[i].y();
    evenIndex++;
  }
  // 计算奇数项和偶数项的平均值
  resultMatrixX = (oddItemsX + evenItemsX) / 2.0;  // 9个点
  resultMatrixY = (oddItemsY + evenItemsY) / 2.0;
  for (int i = 0; i < 10; ++i) {
    obs_points_[i].set_x(resultMatrixX(i));
    obs_points_[i].set_y(resultMatrixY(i));
  }
  for (int i = 10; i < 18; ++i) {
    obs_points_[i].set_x(pre_points_x(0, i - 10));
    obs_points_[i].set_y(pre_points_y(0, i - 10));
  }
  // Set the 20th element of hist_points_buffer_AftFix
}

// step 3（Target Vehicle Trajectory Predict）
void ObsHisPointsGenerator::LinSpace(Eigen::MatrixXd* LinearVector,
                                     double xDistObj, int NumHistBuff) {
  double interval = xDistObj / (NumHistBuff - 1);

  // LinearVector->reserve(NumHistBuff);  // 预分配足够的内存， 20个点
  for (int i = 1; i < NumHistBuff; i++) {
    (*LinearVector)(i - 1) = interval * i;
  }
}

// 插值表创建
void ObsHisPointsGenerator::TJAdistEndPointTableCreater() {
  for (int i = 0; i < 13; i++) {
    xy1_.emplace_back(std::make_pair(k_ODPR_V_VehSpdForVfEndPnt_Xaxis.at(i),
                                     k_ODPR_d_VfEndPntDist_Yaxis.at(i)));
  }
}

// JudgeTrgtVehIN 对目标的状态进行逻辑判断其合理性
bool ObsHisPointsGenerator::JudgeTrgtVehIN(
    const TL::perception::PerceptionObstacle& obs) {
  return (0.5 <= obs.position_flu().x() && obs.position_flu().x() <= 60) &&
         (abs(obs.position_flu().y()) <= 10) && (obs.id() != 0);
}

// 坐标转换 注意这里面的是theta
template <typename T>
void ObsHisPointsGenerator::CoordTranslate2d(T* output_points,
                                             const T& input_points,
                                             double theta, double dx, double dy,
                                             bool is_movefirst) {
  if (output_points == nullptr) {
    return;
  }
  output_points->clear();
  // AERROR "theta: " << theta;
  const auto size = input_points.size();
  const double cos_dtheta = cos(theta);
  const double sin_dtheta = sin(theta);

  if (is_movefirst) {
    for (size_t i = 0; i < size; i++) {
      const double x = (input_points[i].x() - dx) * cos_dtheta +
                       (input_points[i].y() - dy) * sin_dtheta;
      const double y = -1 * (input_points[i].x() - dx) * sin_dtheta +
                       (input_points[i].y() - dy) * cos_dtheta;
      output_points->emplace_back(x, y);
    }
  } else {
    for (size_t i = 0; i < size; i++) {
      const double x = input_points[i].x() * cos_dtheta +
                       input_points[i].y() * sin_dtheta - dx;
      const double y = -1 * input_points[i].x() * sin_dtheta +
                       input_points[i].y() * cos_dtheta - dy;
      output_points->emplace_back(x, y);
    }
  }
}

// EnableDynamicRateLimiter
void ObsHisPointsGenerator::EnableDynamicRateLimiter(int in, int ic, double up,
                                                     double low,
                                                     double mainLoopTime,
                                                     double* out) {
  static bool init_value = true;
  double out_last = 0;
  static double out_save = 0;

  if (init_value) {
    out_last = ic;
    init_value = false;
  } else {
    out_last = Ramp_Resettable_delay_.Deal(out_save);
  }
  double value = in - out_last;

  double y_after_limit =
      SaturationDynamicLimit(value, low * mainLoopTime, up * mainLoopTime);

  out_save = y_after_limit + out_last;
  *out = out_save;
  // AERROR << "VALUE:" << value << "y_after_limit:" << y_after_limit
  //        << "out_last:" << out_last;
}

// 限幅函数
double ObsHisPointsGenerator::SaturationDynamicLimit(double x, double low,
                                                     double high) {
  if (x > high) {
    return high;
  }
  if (x < low) {
    return low;
  }
  return x;
}

void ObsHisPointsGenerator::CreatInitLaneMarkerPoints(
    const TL::perception::LaneMarker& input_lanemarker,
    std::vector<Vec2d>* out_points) {
  out_points->clear();
  const double lanemarker_size = input_lanemarker.longitude_end();
  const double back_start = 20.0;
  const double step = 1.0;
  int max_index = floor((lanemarker_size + back_start) / step + 1);
  for (int i = 0; i < max_index; i++) {
    double x = i * step - back_start;
    double y = input_lanemarker.c0_position() +
               input_lanemarker.c1_heading_angle() * x +
               input_lanemarker.c2_curvature() * x * x +
               input_lanemarker.c3_curvature_derivative() * x * x * x;
    out_points->emplace_back(Vec2d(x, y));
  }
  ADEBUG << "Init lanemarker points size: " << out_points->size()
         << ",start_point_x: " << out_points->front().x()
         << ", start_point_y: " << out_points->front().y()
         << "; end_point_x: " << out_points->back().x()
         << ", end_point_y: " << out_points->back().y();
}

bool ObsHisPointsGenerator::ObsVaildDecider(const MissileObs& missile_obs) {
  bool has_no_target_obs = obstacles_state_->has_no_target_obs();
  bool is_missile_mode_active = vehicle_state_->is_missile_mode_active();
  if (has_no_target_obs &&
      (!is_missile_mode_active || missile_mode_state_ != 2)) {
    ADEBUG << "has_no_target_obs AND is_missile_mode_NO_active!!!";
    return false;
  }

  bool is_obs_valid = missile_obs.is_valid;
  ADEBUG << " is_obs_valid: " << is_obs_valid;
  if (!is_obs_valid && (!is_missile_mode_active || missile_mode_state_ != 2)) {
    ADEBUG << "has_target_obs time less 3s!!!";
    return false;
  }

  is_obj_id_changed_bl_ = missile_obs.deq_obs.front().id() != obs_id_last_ &&
                          (missile_obs.deq_obs.front().id() != 0);
  ADEBUG << " is_obj_id_changed_bl: " << is_obj_id_changed_bl_;
  if (is_obj_id_changed_bl_ && missile_obs.average_heading > 0.8 &&
      (!is_missile_mode_active || missile_mode_state_ != 2)) {
    ADEBUG << "s_obj_id_changed and average_heading > 0.8!!!";
    return false;
  }
  if (using_history_points_time_ > 10.0 ||
      using_history_points_length_ > max_using_history_points_length_) {
    using_history_points_time_ = 0.0;
    using_history_points_length_ = 0.0;
    return false;
  }
  return true;
}

bool ObsHisPointsGenerator::NeedUsingHisPoints(const MissileObs& missile_obs) {
  bool has_no_target_obs = obstacles_state_->has_no_target_obs();
  bool is_missile_mode_active = vehicle_state_->is_missile_mode_active();
  if (has_no_target_obs && is_missile_mode_active && missile_mode_state_ == 2) {
    ADEBUG << "has_no_target_obs AND is_missile_mode_active!!!";
    return true;
  }

  bool is_obs_valid = missile_obs.is_valid;
  if (!is_obs_valid && is_missile_mode_active && missile_mode_state_ == 2) {
    ADEBUG << "has_target_obs time less 3s!!!";
    return true;
  }

  if (is_obj_id_changed_bl_ && missile_obs.average_heading > 0.8 &&
      is_missile_mode_active && missile_mode_state_ == 2) {
    ADEBUG << "s_obj_id_changed and average_heading > 0.8!!!";
    return true;
  }
  return false;
}

double ObsHisPointsGenerator::CalculateLanemarkerY(
    double distance, const TL::perception::LaneMarker& lane_marker) {
  double x2 = distance * distance;
  double x3 = x2 * distance;
  return lane_marker.c0_position() + lane_marker.c1_heading_angle() * distance +
         lane_marker.c2_curvature() * x2 +
         lane_marker.c3_curvature_derivative() * x3;
}

double ObsHisPointsGenerator::RoadCurvatureCalculate(
    const TL::perception::LaneMarker& lane_marker, double length) {
  double y_d = lane_marker.c1_heading_angle() +
               2 * lane_marker.c2_curvature() * length +
               3 * lane_marker.c3_curvature_derivative() * length * length;
  double y_dd = 2 * lane_marker.c2_curvature() +
                2 * 3 * lane_marker.c3_curvature_derivative() * length;
  return abs(y_dd) / pow((1 + y_d * y_d), kCurvatureNum);
}

double ObsHisPointsGenerator::CalculateObsY(
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

std::pair<double, bool> ObsHisPointsGenerator::CalcObsInWhichLane(
    const LaneMarker& left_lanemarker,
    const LaneMarker& right_lanemarker) const {
  constexpr double delta_width = 0.3;
  // has_obs_flag_ = false;
  // int left_id = left_lanemarker.line_seq();
  // int right_id = right_lanemarker.line_seq();
  // AERROR << " left_id: " << left_id << " right_id: " << right_id;
  LaneMarker lanemarker;
  double half_lane_width{2.15};
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

  auto obs_width_dis = CalculateObsY(lanemarker, obs_x_, obs_y_);

  bool has_obs_flag = obs_width_dis < half_lane_width;
  ADEBUG << "half_lane_width: " << half_lane_width;

  return std::make_pair(obs_width_dis, has_obs_flag);
}

}  // namespace missilelane
}  // namespace planning
}  // namespace TL
