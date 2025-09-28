/******************************************************************************
 * Copyright 2022 The TL Authors. All Rights Reserved.
 *****************************************************************************/
#pragma once
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "planning/common/frame.h"
#include "planning/localview/local_view.h"
#include "planning/warning/warning.h"

namespace TL {
namespace planning {

/**
 * @brief 以太网hmi信息
 *
 */
class EthHmi {
 public:
  EthHmi();
  /**
   * @brief 处理以太网hmi信息
   * @param localview
   * @param ptr_trajectory_pb
   */
  void ProcessEthHmi(const std::shared_ptr<LocalView>& local_view,
                     const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb);
  /**
   * @brief 获取地图元素
   * @param localview
   * @param ptr_trajectory_pb
   */
  void ExtractMapElement(
      const std::shared_ptr<LocalView>& local_view,
      const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb);

 private:
  /**
   * @brief 更新换道状态
   *
   * @param ptr_trajectory_pb
   */
  void UpdateHmiChangeLaneState(
      const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb);

  /**
   * @brief 检测变道时，目标车道是否有视觉车道线
   * @param localview 
   * @param ptr_trajectory_pb 
   */
  void CheckLaneChangeDirCameraLane(
      const std::shared_ptr<LocalView>& localview,
      const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb);

  /**
   * @brief 检测是否触发了避让静态障碍物
   * @param ptr_trajectory_pb
   */
  void CheckStaticObstacle(
      const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb);
  /**
   * @brief 更新hmi信息
   *
   * @param frame
   * @param ptr_trajectory_pb
   */
  void UpdateHighlightID(
      const std::shared_ptr<LocalView>& local_view,
      const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb);
  /**
   * @brief 更新hmi信息
   *
   * @param localview
   * @param ptr_trajectory_pb
   */
  void UpdateHmiData(const std::shared_ptr<LocalView>& local_view,
                     const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb);
  /**
   * @brief 读取车辆配置字
   *
   * @param localview
   * @param ptr_trajectory_pb
   */
  void UpdateVehicleCfgToMcu(
      const std::shared_ptr<LocalView>& local_view,
      const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb);
  /**
   * @brief 处理变道告警状态
   *
   * @param ptr_trajectory_pb
   */
  void UpdateLaneChangeWarn(
      const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb);

  /**
   * @brief update pilot lane change status to hmi
   * @param fct_in
   * @param ptr_trajectory_pb
   */
  void UpdatePilotLaneChangeStatusToHmi(
      const TL::functionmanager::FunctionManagerIn& fct_in,
      const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb);

  /**
   * @brief 更新驾驶模式
   *
   * @param nnp_fct_in
   * @param nnp_fct_out
   * @param nnp_fct_out
   */
  void UpdateDriverMode(const TL::functionmanager::FunctionManagerIn& fct_in,
                        const functionmanager::FunctionManagerOut& fct_out,
                        TL::hmi::NNPHmiOutput* nnp_hmi_out);

  void LcaAudioPlayMapping(
      const std::shared_ptr<LocalView>& localview,
      const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb);

  /**
   * @brief 更新给tsr的地图限速
   *
   * @param fct_in
   * @param nnp_fct_out
   * @param nnp_hmi_out
   */
  void UpdateTsrData(const TL::functionmanager::FunctionManagerIn& fct_in,
                     const TL::functionmanager::NnpToFctOutput& nnp_fct_out,
                     TL::hmi::NNPHmiOutput* nnp_hmi_out);
  /**
   * @brief 更新障碍物warning高亮信息
   *
   * @param mutable_hmi_out
   * @param id 障碍物id
   * @param high_light 是否高亮
   */
  void AddDynamicSRObject(TL::hmi::NNPHmiOutput* mutable_hmi_out,
                          u_int32_t id, u_int32_t high_light);

  /**
   * @brief UpdateNNPObsFollowHighLight
   * @param adc_trajectory
   * @param mutable_hmi_out
   */
  void UpdateNNPObsHighLight(
      const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb,
      hmi::NNPHmiOutput* mutable_hmi_out);

  /**
   * @brief UpdateNNPAlcObsFollowHighLight
   * @param adc_trajectory
   * @param mutable_hmi_out
   */
  void UpdateNNPAlcObsHighLight(
      const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb,
      hmi::NNPHmiOutput* mutable_hmi_out);

  void WarningTest(const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb);

  void CheckCameraLaneBoundaryType(
      const std::shared_ptr<LocalView>& localview,
      const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb);

  /**
   * @brief Get the Version object
   */
  void GetVersion();
  void GetVehicleType();
  void GetCarName();
  //   /**
  //    * @brief 检测变道是否因障碍物导致
  //    * @param ptr_trajectory_pb
  //    */
  //   void CheckLanechangeReason(
  //       const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb);

  /**
   * @brief 将规划的地图进行处理，发送至大屏进行显示
   * 
   * @param ptr_trajectory_pb 
   */
  static void UpdateMapPoint(
      const std::shared_ptr<LocalView>& local_view,
      const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb);
  /**
   * @brief 将规划的巡航轨迹进行处理，发送至大屏进行显示
   *
   */
  static void UpdateCruisingTraj(
      const std::shared_ptr<LocalView>& local_view,
      const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb);
  /**
   * @brief 将规划的泊车轨迹进行处理，发送至大屏进行显示
   *
   */
  static void UpdateParkingTraj(
      const std::shared_ptr<LocalView>& local_view,
      const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb);

  /**
   * @brief 
   *
   */
  static void FillParkingTrajGap(
      bool is_forward, const common::VehicleParam& vehicle_param,
      std::vector<TL::common::Point2D>* hmi_traj_flu_coordinates);

 private:
  std::unique_ptr<warning::Warning> warning_ = nullptr;
  std::set<uint32> lc_obs_warning_{};
  std::string version_ = "unkown";
  std::string car_name_;
  std::string vehicle_platform_type_ = "EP41";
};

}  // namespace planning
}  // namespace TL
