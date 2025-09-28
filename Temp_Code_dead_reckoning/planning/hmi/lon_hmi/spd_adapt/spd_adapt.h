/******************************************************************************
 * Copyright 2022 The TL Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <cstdint>
#include <list>
#include <memory>
#include <string>
#include <utility>

#include "google/protobuf/stubs/port.h"
#include "map/hdmap/path.h"
#include "planning/common/frame.h"
#include "planning/common/planning_gflags.h"
#include "planning/hmi/lon_hmi/speed_convertor/speed_convertor.h"
#include "planning/proto/hmi_config.pb.h"
#include "proto/fsm/function_manager.pb.h"
#include "proto/fsm/nnp_fct.pb.h"
#include "proto/soc/chassis.pb.h"

namespace TL {
namespace planning {
// using TL::hdmap::RoadSection;
using google::protobuf::uint32;
static constexpr double kKmh2Ms = 3.6;
static constexpr double kMinDis = 0.1;
static constexpr double kMaxDis = 600.0;
static constexpr int kCruiseSpdStableCnt = 3;
static constexpr int kCamerDistanceBlockedCnt = 30;

/**
 * @brief 智能车速匹配
 *
 */
class SpdAdaptScenario {
 public:
  SpdAdaptScenario() = default;
  ~SpdAdaptScenario() = default;

  struct CameraData {
    bool valid = false;
    bool last_valid = false;
    bool stucked = true;
    bool last_stucked = true;
    bool near = false;
    double distance = 0;
    double last_distance = 0;
    int speed_km = 0;
    int last_speed_km = 0;
    bool road_changed_during_camera = false;
    int road_speed_during_camera = 0;
    bool adapted = false;
    int mem_speed_km = 0;
    bool state_before_camera = false;
    bool ignore_out = false;
    int road_speed_before_camera = 0;
    double adapt_time = 0.0;
  };

  struct RampData {
    bool near = false;
    double distance = 0.0;
    bool adapted = false;
    double speed = 0.0;
  };

  struct TunnelData {
    bool find = false;
    bool near = false;
    double distance = 0;
    bool adapted = false;
    int speed_km = 0;
    int last_speed_km = 0;
    double dec = 0.0;
    bool state_before_tunnel = false;
    int mem_speed_km = 0;
    bool ignore_into = false;
    bool faraway = false;
    bool ignore_out = false;
    double adapt_time = 0.0;
    double last_distance = 0;
  };

  struct RoadData {
    bool is_main_road = false;
    bool last_is_main_road = false;
    int map_speed_km = 0;
    int road_speed_km = 0;
    int cal_road_speed_km = 0;
    int last_road_speed_km = 0;
    bool in_tunnel = false;
    bool last_in_tunnel = false;
    bool is_unkonwn_road = false;
    bool last_is_unkonwn_road = true;
    uint sd_speed_km = 0;
    bool sd_speed_valid = false;
    uint last_sd_speed_km = 0;
    bool last_sd_speed_valid = false;
    bool active_on_unkonwn_road = false;
  };

  /**
   * @brief init
   *
   * @param nnp_fct_in fct in 输入信息
   * @param speed_adapt_config 
   */
  void Init(const TL::functionmanager::SpeedAdaptConfig& speed_adapt_config);
  /**
   * @brief 更新输入信息
   *
   * @param nnp_fct_in fct in 输入信息
   * @param chassis 底盘
   */
  void UpdateInput(functionmanager::FunctionManagerIn* nnp_fct_in,
                   const functionmanager::FunctionManagerOut* fct_out,
                   const TL::common::VehicleState& vehicle_state,
                   int tgtspdctgset, int tgtspddrftset);
  /**
   * @brief 更新fct 的输出信息
   *
   * @param fct_out fct out输出信息
   * @param pnc_map pnc_map 地图
   */
  void UpdateOutput(functionmanager::FunctionManagerOut* fct_out,
                    const std::shared_ptr<hdmap::PncMap>& pnc_map,
                    planning::Frame* frame);

  /**
   * @brief 
   * 
   * @return true 
   * @return false 
   */
  bool UseMemory() const { return use_memery_; }

  /**
   * @brief 
   * 
   * @return true 
   * @return false 
   */
  bool MemState() const { return mem_state_; }

  /**
   * @brief Get the Cal Spd Offset object
   * 
   * @param origin_speed_km 
   * @return int 
   */
  int GetCalSpdOffset(int origin_speed_km) const;

 private:
  /**
   * @brief 
   * 
   * @return double 
   */
  [[nodiscard]] double ProcessDrivingMode() const;

  /**
   * @brief 更新fct 的输出信息
   *
   * @param fct_out fct out输出信息
   * @param pnc_map pnc_map 地图
   */
  static bool IsVehicleSpeedStableLessThan(uint32 chassis_speed,
                                           uint32 target_speed) {
    static uint32 cnt = 0;
    if (chassis_speed <= target_speed) {
      cnt++;
    } else {
      cnt = 0;
    }
    return cnt > kCruiseSpdStableCnt;
  }

  /**
   * @brief 
   * 
   * @param camera_distance 
   * @return true 
   * @return false 
   */
  static bool CameraDistanceStuck(const uint32 camera_distance) {
    static uint32 cnt = 0;
    static uint32 last_camera_distance = 0;
    if (camera_distance == last_camera_distance && camera_distance != 0) {
      cnt++;
    } else {
      cnt = 0;
    }
    last_camera_distance = camera_distance;
    return cnt > kCamerDistanceBlockedCnt;
  }

  /**
   * @brief 
   * 
   * @param chassis 
   */
  double ProcessBadWeather(const TL::soc::Chassis& chassis,
                           double orin_speed_ms);

  /**
   * @brief 
   * 
   * @param fct_out 
   * @param pnc_map 
   * @param frame 
   */
  void UpdateNNPOutput(functionmanager::FunctionManagerOut* fct_out,
                       const std::shared_ptr<hdmap::PncMap>& pnc_map,
                       planning::Frame* frame);
  /**
   * @brief 
   * 
   * @param fct_out 
   * @param pnc_map 
   * @param frame 
   */
  void UpdateNCPOutput(functionmanager::FunctionManagerOut* fct_out,
                       const std::shared_ptr<hdmap::PncMap>& pnc_map,
                       planning::Frame* frame);
  /**
 * @brief 
 * 
 * @param pnc_map 
 * @param frame 
 */
  void SelectCurrRoadSpeed(const std::shared_ptr<hdmap::PncMap>& pnc_map,
                           planning::Frame* frame);
  /**
   * @brief 
   * 
   * @param pnc_map 
   * @param frame 
   */
  void ProcessBaiduSim(functionmanager::FunctionManagerOut* fct_out,
                       const std::shared_ptr<hdmap::PncMap>& pnc_map,
                       planning::Frame* frame);

  int SelectAdapt(functionmanager::FunctionManagerOut* fct_out);
  /**
   * @brief 
   * 
   * @return int 
   */
  int AdaptNotActive(int origin_speed_km);
  /**
   * @brief 
   * 
   * @return int 
   */
  int AdaptActive(int origin_speed_km);
  /**
   * @brief 
   * 
   * @return int 
   */
  int AdaptRoadTypeChanged(int origin_speed_km);
  /**
   * @brief 
   * 
   * @return int 
   */
  int AdaptRoadSpeedChanged(int origin_speed_km);
  /**
   * @brief 
   * 
   * @return int 
   */
  int AdaptIntoRamp(int origin_speed_km);
  /**
   * @brief 
   * 
   * @return int 
   */
  int AdaptIntoTollhouse(int origin_speed_km);
  /**
   * @brief 
   * 
   * @return int 
   */
  int AdaptIntoTunnel(int origin_speed_km,
                      functionmanager::FunctionManagerOut* fct_out);
  /**
   * @brief 
   * 
   * @return int 
   */
  int AdaptOutTunnel(int origin_speed_km,
                     functionmanager::FunctionManagerOut* fct_out);
  /**
   * @brief 
   * 
   * @return int 
   */
  int AdaptTgtspdChanged(int origin_speed_km);
  /**
   * @brief 
   * 
   * @param origin_speed_km 
   * @return int 
   */
  int AdaptIntoCamera(int origin_speed_km);
  /**
   * @brief 
   * 
   * @param origin_speed_km 
   * @return int 
   */
  int AdaptOutCamera(int origin_speed_km);
  /**
   * @brief 
   * 
   * @param origin_speed_km 
   * @return int 
   */
  int AdaptCameraSpeedChanged(int origin_speed_km);
  /**
   * @brief 
   * 
   * @param pnc_map 
   * @param fct_out 
   */
  void CacheRoadElement(const std::shared_ptr<hdmap::PncMap>& pnc_map,
                        functionmanager::FunctionManagerOut* fct_out,
                        planning::Frame* frame);

 private:
  bool fct_ctrl_spd_valid_ = false;
  bool is_nnp_active_ = false;
  bool last_is_nnp_active_ = false;
  TL::functionmanager::SpeedAdaptConfig config_;
  TL::functionmanager::DrivingMode driving_mode_ =
      functionmanager::DrivingMode::UNKNOWN_MODE;
  bool usr_adjust_cruise_speed_ = false;
  uint32 fct_cruise_speed_km_ =
      static_cast<uint32>(FLAGS_default_cruise_speed * kKmh2Ms);

  int hmi_road_speed_km_ = 0;

  bool need_adapt_ = false;

  functionmanager::CDCSSpeedLmit cdcs_speed_limit_ = {};
  functionmanager::TsrInfo tsr_ = {};
  bool use_memery_ = false;
  bool mem_state_ = false;
  // 模式
  int tgtspdctgset_ = 0;
  // 值
  int tgtspddrftset_ = 0;
  int spd_offset_ = 0;
  bool tgtspd_valid_ = false;
  bool tgtspd_changed_ = false;
  CameraData camera_data_;
  RampData ramp_data_;
  RoadData road_data_;
  RampData tollhouse_data_;
  TunnelData tunnel_data_;
};  // namespace planning
}  // namespace planning
}  // namespace TL
