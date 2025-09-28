/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  planning
 */

#ifndef PLANNING_PLANNING_H_
#define PLANNING_PLANNING_H_

#include <unistd.h>
#include <cstdint>
#include <list>
#include <map>
#include <memory>
#include <shared_mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>
#include "adf/include/base.h"
#include "adf/include/log.h"
#include "adf/include/node_base.h"
#include "cm/include/proto_method.h"
#include "common/memory/arena_adapter.h"
#include "common/status/status.h"
#include "common/zmq_conventor/zmq_sender.h"
#include "map/hdmap/hdmap_util.h"
#include "planning/common/function_statistics.h"
#include "planning/core/on_lane_planning.h"
#include "planning/core/planning_base.h"
#include "planning/core/safety_guard_planning.h"
#include "planning/trigger/metric_collect.h"
#include "planning/trigger/orin_trigger_manager.h"
#include "planning/zmq2receiver/zmq2receiver.h"
#include "planning/proto/planning_config.pb.h"
#include "proto/common/pnc_point.pb.h"
#include "proto/control/adas_someip.pb.h"
#include "proto/control/mbd_control_debug.pb.h"
#include "proto/control/mcu_to_soc.pb.h"
#include "proto/fsm/function_manager.pb.h"
#include "proto/fsm/nnp_fct.pb.h"
#include "proto/fsm/soc_to_mcu.pb.h"
#include "proto/local_mapping/local_map.pb.h"
#include "proto/localization/localization.pb.h"
#include "proto/map/map.pb.h"
#include "proto/perception/perception_freespace.pb.h"
#include "proto/perception/perception_obstacle.pb.h"
#include "proto/perception/perception_parking_lot.pb.h"
#include "proto/perception/transport_element.pb.h"
#include "proto/planning/pad_msg.pb.h"
#include "proto/planning/planning.pb.h"
#include "proto/planning/trigger.pb.h"
#include "proto/prediction/prediction_obstacle.pb.h"
#include "proto/routing/routing.pb.h"
#include "proto/soc/apa2mcu_chassis.pb.h"
#include "proto/soc/chassis.pb.h"
#include "proto/statemachine/state_machine.pb.h"

namespace TL {

using TL::common::Header;
using TL::common::mcu_to_soc_DebugData;
using TL::common::Status;
using TL::control::AdasSomeipFromMCU;
using TL::control::MbdDebugFromMCU;
using TL::control::McuToSocPnc;
using TL::functionmanager::FunctionManagerIn;
using TL::functionmanager::FunctionManagerOut;
using TL::functionmanager::SocToFctBus;
using TL::functionmanager::TaPilotMode;
using TL::hdmap::HDMapUtil;
using TL::hdmap::Map;
using TL::localization::Localization;
using TL::mapping::LocalMap;
using TL::navigation_hdmap::MapMsg;
using TL::netaos::adf::BaseData;
using TL::netaos::adf::NodeBundle;
using TL::netaos::adf::ProfileToken;
using TL::perception::FreeSpaceOutArray;
using TL::perception::LaneMarkers;
using TL::perception::ParkingLotOutArray;
using TL::perception::PerceptionObstacles;
using TL::perception::TrafficLightDetection;
using TL::perception::TransportElement;
using TL::planning::ADCTrajectory;
using TL::planning::DependencyInjector;
using TL::planning::EventTrigger;
using TL::planning::LocalView;
using TL::planning::MetricCollect;
using TL::planning::OnLanePlanning;
using TL::planning::OrinTriggerManager;
using TL::planning::SafetyGuardPlanning;
using TL::planning::ZMQ2Dreamview;
using TL::prediction::PredictionObstacles;
using TL::routing::RoutingResponse;
using TL::soc::Apa2Chassis;
using TL::soc::Chassis;
using TL::soc::VehicleConfigure;
using TL::state::StateMachine;
using TL::trigger::TriggerConfig;

using BaseDataTypePtr = std::shared_ptr<BaseData>;

struct TriggerAndCfgPub {
  planning::WarningStatus warning_status{};
  int32_t adcs_longitud_ctrl_setdistance;
  planning::EventTrigger event_trigger{};
  planning::PlanningFault planning_fault{};
  bool is_state_ready_to_report = false;
};

/**
 * @brief Planning类,继承NodeBase，在mdc平台使用，收发数据，调用整体planning规划
 *
 */
class Planning : public TL::netaos::adf::NodeBase {
 public:
  Planning();

  ~Planning() { planning::FunctionStatistics done_guard("~Planning()"); }

  /**
   * @brief init planning，继承自node_base的AlgInit
   *
   * @return 1 means init success
   */
  virtual int32_t AlgInit();

  /**
   * @brief RegisterMessageType
   *
   */
  void RegisterMessageType();

  /**
   * @brief 用于在base注册nnp procrss线程，感知数据触发
   *
   * @param input base中提供的NodeBundle，数据通过NodeBundle中的关键字获取
   * @param token 统计端到端的耗时参数
   * @return 1 代表process函数运行正常
   */

  int32_t AlgProcessNNP(NodeBundle* input, const ProfileToken& token);

  /**
   * @brief 用于在base注册nnp procrss线程，100ms周期触发
   *
   * @param input base中提供的NodeBundle，数据通过NodeBundle中的关键字获取
   * @param token 统计端到端的耗时参数
   * @return 1 代表process函数运行正常
   */
  int32_t AlgProcessAVP(NodeBundle* input, const ProfileToken& token);

  /**
   * @brief 用于在base注册自仿真的线程，100ms周期触发
   *
   * @param input base中提供的NodeBundle，数据通过NodeBundle中的关键字获取
   * @param token 统计端到端的耗时参数
   * @return 1 代表函数运行正常
   */
  int32_t SelfSimulatorFunc(NodeBundle* input);

  /**
   * @brief AlgReleases用于程序停止时释放资源，继承自node_base的AlgRelease
   */
  virtual void AlgRelease();

 private:
  /**
   * @brief 从minieye或者space获取指定的定位信息并转换成proto数据
   *
   * @param input base中提供的NodeBundle，数据通过NodeBundle中的关键字获取
   * @param is_nnp_mode is nnp mode
   */
  void GetLocation(NodeBundle* input, bool is_nnp_mode);

  /**
   * @brief 从minieye或者space获取指定的障碍物信息并转换成proto数据
   *
   * @param input base中提供的NodeBundle，数据通过NodeBundle中的关键字获取
   * @param is_nnp_mode is nnp mode
   */
  void GetObstacle(NodeBundle* input, bool is_nnp_mode);

  /**
   * @brief 获取底盘数据并转换成对应的proto
   *
   * @param input base中提供的NodeBundle，数据通过NodeBundle中的关键字获取
   */
  void GetChassis(NodeBundle* input);

  /**
   * @brief 从minieye或者space获取指定的车道线信息并转换成proto数据
   *
   * @param input base中提供的NodeBundle，数据通过NodeBundle中的关键字获取
   * @param is_nnp_mode is nnp mode
   */
  void GetTransportElement(NodeBundle* input, bool is_nnp_mode);

  /**
   * @brief 获取红绿灯数据并转换成proto数据
   * @param input base中提供的NodeBundle，数据通过NodeBundle中的关键字获取
   */
  void GetTrafficLight(NodeBundle* input);

  /**
   * @brief 获取local map数据
   * @param input 
   */
  void GetLocalMap(NodeBundle* input);

  /**
   * @brief space获取指定的泊车位信息并转换成proto数据
   *
   * @param input base中提供的NodeBundle，数据通过NodeBundle中的关键字获取
   */
  void GetParkingLot(NodeBundle* input);

  /**
   * @brief 泊车模式下，清空行车errorcode数据
   */
  void ClearNNPData();

  /**
   * @brief 行车模式下，清空泊车errorcode数据
   */
  void ClearAVPData();

  /**
   * @brief 从minieye或者space获取指定的可行驶区域信息并转换成proto数据
   *
   * @param input base中提供的NodeBundle，数据通过NodeBundle中的关键字获取
   * @param is_nnp_mode is nnp mode
   */
  void GetFreeSpace(NodeBundle* input, bool is_nnp_mode);

  void GetTriggerConfig();

  void UpdateLonCtrlSetDisMem();

  /**
  * @brief 获取mcu上行的fct信息并转换成对应的proto
  *
  * @param input base中提供的NodeBundle，数据通过NodeBundle中的关键字获取
  * @return TaPilotMode
  */
  std::shared_ptr<FunctionManagerIn> GetFctIn(NodeBundle* input);

  /**
   * @brief process planning的函数，在输入数据格式转换完之后调用
   *
   * @param token 统计端到端的耗时参数
   */
  void ProcessPlanning(const ProfileToken& token);
  /**
   * @brief pub 线程
   * @return  代表函数运行正常
   */
  void PubTrajectoryThread();

  void PubTriggerCfgThread();

  void UpdateTopicHeaderInfo(
      const std::shared_ptr<ADCTrajectory>& ptr_adc_trajectory,
      const std::shared_ptr<LocalView>& local_view);

  /**
   * @brief pub轨迹函数，包含轨迹数据结构的转换，pb2struct
   *
   * @param ptr_adc_trajectory ptr_adc_trajectory共享指针
   * @param token 统计时延用
   */
  void PubTrajectory(const std::shared_ptr<ADCTrajectory>& ptr_adc_trajectory,
                     const std::shared_ptr<LocalView>& local_view,
                     const ProfileToken& token);
  /**
   * @brief pub trigger(event fault)
   * @param ptr_adc_trajectory
   * @param mcu_platform_fault
   */
  void PubTriggerAndCfg(
      const std::shared_ptr<ADCTrajectory>& ptr_adc_trajectory,
      const std::shared_ptr<TriggerAndCfgPub>& trigger_and_cfg_pub,
      uint8 mcu_platform_fault);

  /**
   * @brief pub routing 函数
   * @param routing_response
   * @param token 统计时延用
   */
  void PubRoutingResponse(
      const std::shared_ptr<RoutingResponse>& routing_response,
      const ProfileToken& token);
  /**
   * @brief pub预测数据
   * @param prediction_obstacles
   * @param token 统计时延用
   */
  void PubPredictionObstacles(
      const std::shared_ptr<PredictionObstacles>& ptr_prediction_obstacles,
      const ProfileToken& token);
  /**
   * @brief pub下行fct数据
   * @param soc_to_fct_bus
   * @param token 统计时延用
   */
  void PubFctBus(const std::shared_ptr<SocToFctBus>& ptr_soc_to_fct_bus,
                 const ProfileToken& token);

  /**
   * @brief 发送给大屏的hmi信息
   *
   * @param adc_trajectory
   * @param token 统计时延用
   */
  void PubEthHmi(const ADCTrajectory& adc_trajectory,
                 const ProfileToken& token);
  /**
   * @brief 给mcu下发fct状态机信息
   *
   * @param adc_trajectory
   * @param token 统计时延用
   */
  void PubFctOut2MCU(const ADCTrajectory& adc_trajectory,
                     const ProfileToken& token);
  /**
   * @brief 下发到mcu的warnning信息
   *
   * @param adc_trajectory adc_trajectory proto
   * @param token 统计时延用
   */
  void PubWarning(const ADCTrajectory& adc_trajectory,
                  const ProfileToken& token);

  /**
   * @brief 处理给座舱的hmi信息
   *
   * @param local_view local_view共享指针
   * @param ptr_adc_trajectory ptr_adc_trajectory共享指针
   */
  void ProcessWarningAndEthHmi(
      const std::shared_ptr<LocalView>& local_view,
      const std::shared_ptr<ADCTrajectory>& ptr_adc_trajectory,
      uint8 mcu_platform_fault);

  /**
   * @brief ProcessRegularTrajectory
   *
   * @param retrived_local_view
   * @param token
   */
  bool ProcessRegularTrajectory(
      const std::shared_ptr<LocalView>& retrived_local_view,
      const ProfileToken& token);
  /**
   * @brief ProcessGuardTrajectory
   *
   * @param retrived_local_view
   * @param token
   * @return true GuardTrajectory enable
   * @return false GuardTrajectory disable
   */
  bool ProcessGuardTrajectory(
      const std::shared_ptr<LocalView>& current_local_view,
      const ProfileToken& token);
  /**
   * @brief send output
   *
   * @param token
   * @param output
   */
  template <typename ProtoDataType>
  void Send(const ProfileToken& token, const std::string& tpoic,
            const std::shared_ptr<ProtoDataType>& proto_data);

  /**/
  void PubControlData(const std::shared_ptr<McuToSocPnc>& ptr_mcu_to_soc_pnc,
                      const ProfileToken& token);

  /**
   * @brief Publish fct in
   * @param ptr_fct_in
   * @param token
   */
  void PubFctIn(const std::shared_ptr<FunctionManagerIn>& ptr_fct_in,
                const ProfileToken& token);

  /**
   * @brief check 轨迹点，对应的给fct报nnp的故障
   *
   * @param ptr_adc_trajectory
   */
  void UpdateFctOutByTrajectory(
      const std::shared_ptr<ADCTrajectory>& ptr_adc_trajectory);

  /**
   * @brief RecordCpuAndMemInfo
   *
   * @param ptr_adc_trajectory
   */
  void RecordCpuAndMemInfo(ADCTrajectory* ptr_adc_trajectory);

  void GetMapFusionData(NodeBundle* input);

  /**
  * @brief 监听当前的ta polit mode
  *
  * @param ta_polit_mode
  * @param function_topic
  * @return true mode change
  * @return false mode not change
  */
  bool FunctionMonitor(const TaPilotMode& ta_polit_mode,
                       const std::string& function_topic);
  /**
   * @brief Get the Token object
   *
   * @param retrived_local_view
   * @return hz_Adsfi::ProfileToken
   */
  ProfileToken GetToken(const std::shared_ptr<LocalView>& retrived_local_view);

  /**
 * @brief Publish debug
 *
 * @param local_view
 * @param adctrajectory
 */
  void PublishDebug(const std::shared_ptr<LocalView>& local_view,
                    const ADCTrajectory& adctrajectory);

  /**
   * @brief Get the Data By Topic object
   *
   * @tparam ProtoDataType
   * @param input
   * @param topic
   * @param ptr_data
   * @return int
   */
  template <typename ProtoDataType>
  std::shared_ptr<ProtoDataType> GetDataByTopic(NodeBundle* input,
                                                const std::string& topic,
                                                int* ret);

  void GetSomeipAdasFromMcu(NodeBundle* input);
  /**
   * @brief 
   * 
   * @param local_view 
   * @param ptr_trajectory_pb 
   */
  void UpdateNTPDistanceAndTime(
      const LocalView& local_view,
      const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb);
  /**
   * @brief 
   * 
   * @param remain_dist 
   * @param ptr_trajectory_pb 
   */
  void UpdateLAPAParkingStageHmiData(
      double remain_dist,
      const std::shared_ptr<ADCTrajectory>& ptr_trajectory_pb);

 private:
  std::shared_ptr<Chassis> chassis_ = nullptr;

  // perception
  std::shared_ptr<PerceptionObstacles> obstacles_ = nullptr;
  // location
  std::shared_ptr<Localization> location_ = nullptr;
  std::shared_ptr<common::Pose> slam_map_pose_ = nullptr;
  // lanemarker
  std::shared_ptr<TransportElement> transport_element_ = nullptr;
  std::shared_ptr<AdasSomeipFromMCU> adas_someip_from_mcu_ = nullptr;
  std::shared_ptr<LaneMarkers> lane_markers_ = nullptr;
  /// ParkingLotOutArray
  std::shared_ptr<ParkingLotOutArray> parking_lot_array_ = nullptr;
  // freespace
  std::shared_ptr<FreeSpaceOutArray> free_space_array_ = nullptr;
  std::shared_ptr<FunctionManagerIn> fct_in_ = nullptr;
  std::shared_ptr<TrafficLightDetection> traffic_light_detection_ = nullptr;
  std::shared_ptr<LocalMap> local_map_ = nullptr;
  std::unique_ptr<TL::planning::PlanningBase> regular_planning_base_;
  std::unique_ptr<TL::planning::PlanningBase> guard_planning_base_;
  MbdDebugFromMCU mbd_debug_;
  mcu_to_soc_DebugData adas_data_;
  std::shared_ptr<DependencyInjector> injector_;
  TL::planning::PlanningConfig config_;
  std::atomic<bool> is_pub_thread_stop_{false};
  std::atomic<bool> is_assure_run_once_{false};
  std::atomic<bool> is_pub_trigger_thread_stop_{false};

  std::unique_ptr<TL::common::ZMQSender> zmq_sender_;
  std::thread writer_trigger_cgf_thread_;
  std::unique_ptr<ZMQ2Dreamview> zmq_receiver_;
  VehicleConfigure vehicle_cfg_;
  std::unique_ptr<OrinTriggerManager> orin_trigger_manager_;
  std::unique_ptr<MetricCollect> metric_collect_;
  TL::soc::WarningSwitchMemory warning_switch_mem_;
  uint32_t lon_ctrl_set_dis_mem_ = 0;
  TL::common::VehicleParam vehicle_param_;
  TL::planning::warning::WarningStateSwitch last_warning_state_;
  std::thread writer_thread_;
  bool trajectory_check_ok_ = true;
  std::shared_ptr<const ADCTrajectory> last_regular_adc_trajectory_;
  const std::string avp_topic_ = "avp_main";
  const std::string nnp_topic_ = "nnp_main";
  TL::planning::PublishQueuePtr publish_queue_;
  common::base::BoundedQueue<std::shared_ptr<TriggerAndCfgPub>>
      planning_trigger_cfg_queue_;
  uint64_t last_process_ms_ = 0;
  const unsigned gap_ = 75;
  std::atomic_bool has_new_trigger_config_ = false;
  TriggerConfig trigger_config_{};
  std::shared_ptr<MapMsg> map_fusion_ = nullptr;
  bool map_fusion_recieved_{false};

  std::shared_ptr<McuToSocPnc> control_data_ = nullptr;
  std::shared_ptr<common::memory::ArenaAdapter> arena_adapter_ = nullptr;
  common::memory::ArenaPtr arena_ = nullptr;
  uint32 sequence_num_ = 0;
  double all_his_time_ = 0.0;
  double pre_time_ = 0.0;
  double all_his_distance_ = 0.0;
  double pre_distance_ = 0.0;
  soc::Chassis::DrivingMode pre_driving_mode_ = soc::Chassis::COMPLETE_MANUAL;
};
}  // namespace TL

#endif  // PLANNING_PLANNING_H_
