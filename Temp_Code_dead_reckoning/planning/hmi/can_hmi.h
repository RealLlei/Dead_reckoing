/******************************************************************************
 * Copyright 2022 The TL Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <list>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "common/file/file.h"
#include "gflags/gflags_declare.h"
#include "map/hdmap/path.h"
#include "planning/common/dependency_injector.h"
#include "planning/common/frame.h"
#include "planning/common/obstacle.h"
#include "planning/common/reference_line_info.h"
#include "planning/localview/local_view.h"

#include "planning/hmi/lon_hmi/lon_hmi.h"
#include "planning/hmi/lon_hmi/spd_adapt/spd_adapt.h"
#include "planning/proto/hmi_config.pb.h"
#include "proto/common/vehicle_state.pb.h"
#include "proto/fsm/function_manager.pb.h"
#include "proto/fsm/soc_to_mcu.pb.h"
#include "proto/planning/planning.pb.h"
#include "proto/planning/planning_status.pb.h"
#include "proto/routing/routing.pb.h"
#include "proto/soc/chassis.pb.h"

namespace TL {
namespace planning {

enum class ObstacleAvoidance {
  UNKOWN = 0,
  STATIC = 1,
  ONE_SIDES = 2,
  BOTH_SIDES = 3,
};

/**
 * @brief nnp can hmi
 *
 */
class CanNnpHmi {
 public:
  CanNnpHmi() = default;
  ~CanNnpHmi() = default;

  /**
   * @brief init
   *
   */
  void Init();
  /**
   * @brief fct 后处理
   *
   * @param frame
   * @param pnc_map
   * @param injector
   * @param trajectory_pb
   */
  void ProcessFctOutput(Frame* frame,
                        const std::shared_ptr<hdmap::PncMap>& pnc_map,
                        const std::shared_ptr<DependencyInjector>& injector,
                        const std::shared_ptr<ADCTrajectory>& trajectory_pb);
  /**
   * @brief 处理fct输入信息
   *
   * @param local_view
   */
  void ProcessFctInput(const std::shared_ptr<LocalView>& local_view);

 private:
  /**
   * @brief fct换道信息
   */
  void ChangeLaneOutputDecision();

  /**
   * @brief 检测nnp的激活条件
   */
  void CheckNnpActiveCondition();

  /**
   * @brief 检测是否退出功能
   */
  void DetectWhetherToExitFunction();

  /**
   * @brief 内部和外部激活条件合并
   */
  void MergeNnpActiveCondition();

  /**
   * @brief 检测车辆航向角是否满足激活条件和检测车辆是否处于逆行车道
   * @return std::pair<bool, bool> first:车辆航向角；second：是否处于逆行车道
   */
  std::pair<bool, bool> CheckVehicleHeadingAndReverseLane();

  /**
   * @brief 检测当前adc是否压线
   * @return true 压线
   * @return false
   */
  bool CheckTraWhetherCrimping();

  /**
   * @brief 检测对应轨迹点是否压线
   * @param path_point
   * @return true
   * @return false
   */
  bool CheckTraPointWhetherCrimping(const common::PathPoint& path_point);
  /**
   * @brief 检测前方5s位置道路曲率
   * @return true
   * @return false
   */
  bool CheckCurvature();

  /**
   * @brief 查表检测曲率和速度是否满足
   * @param curvature_velocity first：曲率；second：速度
   * @return true 满足返回
   * @return false
   */
  bool CheckCurvatureAndVelocity(
      const std::pair<double, double>& curvature_velocity);

  /**
   * @brief 判断当前车道信息
   */
  void CheckLaneMsg();

  /**
   * @brief 检测当前行车场景和判断audio play类型
   */
  void CheckScenariosOrAudioPlay();

  /**
   * @brief 检测前方是否有nudge障碍物并进行避让提醒和高亮
   */
  void CheckTargetObstacle();

  /**
   * @brief 检测变道过程中是否有危险障碍物
   */
  void CheckLaneChangeWaningObs();

  /**
   * @brief 检测前方是否lane reduce
   */
  void CheckLaneReduce();

  /**
   * @brief 智能车速匹配输出
   */
  void CheckSpeedAdapt();

  /**
   * @brief 检测接管提醒
   */
  void CheckTakeOver();

  /**
   * @brief 判断是否为导航图层
   * @return true
   * @return false
   */
  bool IsNviLayer();

  /**
   * @brief 车道过宽接管
   */
  void LaneWidthTakeOver();

  /**
   * @brief 汇入匝道口大于一个车道的接管提醒
   */
  void RampExitTakeOver();

  /**
   * @brief 下匝道前降级到pilot报接管
   */
  void RampDowngradeTakeOver();

  /**
   * @brief 在merge点前的接管提醒
   */
  void MergeLaneTakeOver();

  /**
   * @brief 纵向接管
   */
  void LongDirTakeOver();

  /**
   * @brief 汇入汇出场景定位故障播报“手动变道”
   */
  void LocationFaultForTakeOver();

  /**
   * @brief 汇入汇出播报场景
   */
  void InOutRamp();

  /**
   * @brief 进入和错过匝道口检测
   */
  void CheckEnterRampOrExitMissed();

  /**
   * @brief 判断当前车速在time_threshold时间内能否到达len
   * @param len
   * @param time_threshold
   * @return true
   * @return false
   */
  bool IsRightnessTimeToDis(double len, double time_threshold);

  /**
   * @brief 判断当前车速在time_threshold时间内能否到达len
   * @param len
   * @param time_threshold
   * @return true
   * @return false
   */
  bool IsPlayTimeToDis(double len, double time_threshold);

  /**
   * @brief lane split and merge ctl light
   */
  void CtlLightLaneSplitAndMerge();

  /**
   * @brief 判断merge方向打灯
   * @return std::pair<routing::ChangeLaneType, double>
   */
  std::pair<routing::ChangeLaneType, double> CtlLightForMerge();

  /**
   * @brief 处理没有变道场景的车道线高亮信号发送
   */
  void CheckLaneHighNoLaneChange();

  /**
   * @brief 检查在split场景变道参考线跳变报接管
   */
  void CheckSplitLaneChangeTakeOver();

  /**
   * @brief 处理fct输入的巡航车速和跟车时距信息
   * @param local_view
   */
  void ProcessFctInputLonCtrlInfo(const std::shared_ptr<LocalView>& local_view);

  /**
   * @brief update pilot lane change status
   * @param fct_in
   * @param nnp_fct_in
   */
  void UpdatePilotLaneChangeStatus(
      const functionmanager::FunctionManagerIn& fct_in,
      functionmanager::FctToNnpInput* nnp_fct_in);

  /**
   * @brief
   * 根据fct输入的nnp的状态，nnp激活才下发交互类报文，不激活情况下发激活条件fct信息到mcu
   */
  void ResetFctHmiOutByNNPSysState();

  /**
   * @brief 更新给座舱的轨迹信息
   */
  void UpdateNnpTrajectoryForHmi();
  /**
   * @brief fct_out set default value
   */
  void ResetFctHmiOut();

  /**
   * @brief 非变道状态set default value
   */
  void ResetOtherFctHmiOut();

  /**
   * @brief 变道过程中，将车的后轴中心过了车道线判定变道完成，
   * 拓展为后轴中心到目标车道中心线buffer范围内
   *
   * @return true 变道中
   * @return false 变道完成
   */
  bool ExpandChangeLaneOngoing();

  /**
   * @brief 优化变道语音交互
   * @param is_cancel_lane_change
   */
  void OptimizationChangeLaneAudioPlay(bool is_cancel_lane_change);

  /**
   * @brief 扩展变道输出状态
   */
  void ExpandChangeLaneOutput();

  /**
   * @brief 连续变道时，检测是否发送变道完成，否则会引起座舱车位渲染异常
   */
  void UpdateContinuousLaneChange();

  /**
   * @brief 更新变道状态输出
   * @param cl_type 变道类型
   */
  void UpdateChangeLaneOutput(const functionmanager::ChangeLaneInfor& cl_type);

  /**
   * @brief 语音变道，导航变道、效率变道以及导航下匝道和下路口情况下，控左转和右转灯
   * @param light_dir_ctl
   * @return std::pair<bool, functionmanager::LightReq>
   */
  std::pair<bool, functionmanager::LightReq> CtrlChangeLaneLight(
      const functionmanager::LightReq& light_dir_ctl);

  /**
   * @brief 拨杆变道时，变道完成或者变道取消，超时未关灯提醒
   */
  void ChangeLaneLightRemind();
  /**
   * @brief 纵向跟车目标高亮
   *
   */
  void UpdateLonFollowObsHighLight();

  /**
   * @brief 获取停止线前距离
   * @return std::optional<double> 
   */
  std::optional<double> GetToStopLineDis();

  /**
   * @brief 判断在junction里自车是否在执行车道
   * @return true 
   * @return false 
   */
  std::optional<bool> IsStraightLaneInJunction();

  /**
   * @brief 更新纵向大车避让
   * @param obs
   * @return true
   * @return false
   */
  bool UpdateLonBigCarAvoid(const Obstacle* obs);
  /**
   * @brief
   *
   * @param reference_line_info
   * @return true
   * @return false
   */
  static bool CheckFollowObsNeedHighLight(
      const ReferenceLineInfo* reference_line_info);
  /**
   * @brief
   *
   * @param reference_line_info
   * @param obs
   * @param left_buffer
   * @param right_buffer
   * @return true
   * @return false
   */
  static bool IsObsOnLaneWithBuffer(
      const ReferenceLineInfo* reference_line_info, const Obstacle* obs,
      double left_buffer, double right_buffer);
  /**
   * @brief
   *
   * @param reference_line_info
   * @param obs
   * @param left_buffer
   * @param right_buffer
   * @return true
   * @return false
   */
  static bool IsUseLast(const ReferenceLineInfo* reference_line_info,
                        const Obstacle* obs, double left_buffer,
                        double right_buffer);

 private:
  std::shared_ptr<ADCTrajectory> ptr_trajectory_pb_ = nullptr;
  Frame* frame_ = nullptr;
  std::shared_ptr<hdmap::PncMap> pnc_map_ = nullptr;
  std::shared_ptr<DependencyInjector> injector_ = nullptr;
  LonHmi lon_hmi_{};
  functionmanager::HmiConfig hmi_config_{};
  bool is_expand_change_lane_ = false;
  functionmanager::LaneChangeDir expand_change_lane_dir_ =
      functionmanager::LaneChangeDir::NONE_DIR;
  functionmanager::HmiChangeLaneReason change_lane_reason_ =
      functionmanager::HMI_NONE;
  common::VehicleParam vehicle_param_{};
  std::unordered_map<int32_t, std::pair<double, bool>> last_obs_avoid_ids_{};
  bool soon_in_carriageWay_ = false;
  bool soon_in_ramp_ = false;
  bool is_nnp_active_ = false;
  bool is_lat_override_ = false;
  bool is_override_ = false;
  bool is_soc_pilot_active_ = false;
  bool is_pilot_lat_override_ = false;
  bool is_pilot_override_ = false;
};

/**
 * @brief avp can hmi信息
 *
 */
class CanAvpHmi {
 public:
  CanAvpHmi() = default;
  ~CanAvpHmi() = default;
  /**
   * @brief init
   *
   */
  void Init();
  /**
   * @brief 处理fct out
   *
   * @param frame
   * @param pnc_map
   * @param injector
   * @param trajectory_pb
   */
  void ProcessFctOutput(Frame* frame,
                        const std::shared_ptr<hdmap::PncMap>& pnc_map,
                        const std::shared_ptr<DependencyInjector>& injector,
                        const std::shared_ptr<ADCTrajectory>& trajectory_pb);
  /**
   * @brief 处理fct 输入信息
   *
   * @param local_view
   */
  void ProcessFctInput(const std::shared_ptr<LocalView>& local_view);

 private:
  void UpdateAvpHmiData();
  /**
   * @brief Get the Traj Equation object 拟合轨迹方程
   *
   */
  void GetTrajEquation();
  /**
   * @brief Update park bar for TBA
   *
   * @param remain_distance remain_distance for TBA
   */
  void UpdateTBAParkBar(double remain_distance);
  /**
   * @brief 更新时间和距离
   *
   */
  void UpdateTimeAndDistance();
  /**
   * @brief 更新巡航过程中转弯提示信息
   *
   */
  void UpdateTurnLeftRightReminder();
  /**
   * @brief 更新巡航过程中避障和跟车提示信息
   *
   */
  void UpdateAvoidAndFollowReminder(
      const std::shared_ptr<ADCTrajectory>& trajectory_pb);
  /**
   * @brief 处理纵向输入信息
   *
   * @param local_view
   */
  void ProcessFctInputLonCtrlInfo(const std::shared_ptr<LocalView>& local_view);

  std::shared_ptr<ADCTrajectory> ptr_trajectory_pb_ = nullptr;
  Frame* frame_ = nullptr;
  std::shared_ptr<hdmap::PncMap> pnc_map_ = nullptr;
  std::shared_ptr<DependencyInjector> injector_ = nullptr;
  common::VehicleParam vehicle_param_;
  double total_distance_ = 1000.0;
  int last_park_bar_ = 1;
  double reminder_x_ = 0.0;
};

}  // namespace planning
}  // namespace TL
