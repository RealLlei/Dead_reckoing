/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  planning
 */

#ifndef PLANNING_PLANNING_H_
#define PLANNING_PLANNING_H_

#include <array>
#include <atomic>
#include <cstdint>
#include <list>
#include <map>
#include <memory>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "adsf/node/node_base.h"
#include "adsfi/include/data_types/planning/planning_debug.h"
#include "common/status/status.h"
#include "common/zmq_conventor/zmq_sender.h"
#include "data_collect_client/include/data_collect_client.h"
#include "google/protobuf/repeated_field.h"
#include "map/hdmap/hdmap_util.h"
#include "planning/core/on_lane_planning.h"
#include "planning/core/planning_base.h"
#include "planning/core/safety_guard_planning.h"
#include "planning/trigger/adc_trigger_manager.h"
#include "planning/trigger/metric_collect.h"
#include "planning/zmq2receiver/zmq2receiver.h"

#include "planning/proto/planning_config.pb.h"
#include "proto/control/mbd_control_debug.pb.h"
#include "proto/control/mcu_to_soc.pb.h"
#include "proto/dead_reckoning/dr.pb.h"
#include "proto/fsm/trigger_config.pb.h"
#include "proto/hmi/nns_location.pb.h"
#include "proto/hmi/nns_router.pb.h"
#include "proto/localization/localization.pb.h"
#include "proto/map/map.pb.h"
#include "proto/perception/perception_freespace.pb.h"
#include "proto/perception/perception_parking_lot.pb.h"
#include "proto/perception/transport_element.pb.h"
#include "proto/planning/pad_msg.pb.h"
#include "proto/planning/planning.pb.h"
#include "proto/planning/trigger.pb.h"
#include "proto/prediction/prediction_obstacle.pb.h"
#include "proto/routing/routing.pb.h"

namespace TL {
using TL::common::Header;
using TL::common::mcu_to_soc_DebugData;
using TL::common::Status;
using TL::control::MbdDebugFromMCU;
using TL::datacollect::client::DataCollectClient;
using TL::datacollect::client::DataCollectResType;
using TL::dead_reckoning::DeadReckoning;
using TL::functionmanager::FunctionManagerIn;
using TL::functionmanager::TaPilotMode;
using TL::hdmap::HDMapUtil;
using TL::hdmap::Map;
using TL::localization::Localization;
using TL::navigation_hdmap::MapMsg;
using TL::perception::LaneMarkers;
using TL::perception::TrafficLightDetection;
using TL::perception::TransportElement;
using TL::planning::ADCTrajectory;
using TL::planning::CollectTriggerManager;
using TL::planning::DependencyInjector;
using TL::planning::EventTrigger;
using TL::planning::LocalView;
using TL::planning::MetricCollect;
using TL::planning::OnLanePlanning;
using TL::planning::SafetyGuardPlanning;
using TL::planning::ZMQ2Dreamview;
using TL::prediction::PredictionObstacles;
using TL::routing::RoutingResponse;
using TL::soc::Chassis;
using TL::soc::VehicleConfigure;
using TL::trigger::TriggerConfig;
using hz_Adsfi::NodeBase;
using hz_Adsfi::NodeBundle;

/**
 * @brief Planning类,继承NodeBase，在mdc平台使用，收发数据，调用整体planning规划
 *
 */
class Planning : public NodeBase {
 public:
  Planning();

  ~Planning() override = default;

  /**
   * @brief init planning，继承自node_base的AlgInit
   *
   * @return 1 means init success
   */
  int32_t AlgInit() override;
  /**
   * @brief 用于在base注册nnp procrss线程，感知数据触发
   *
   * @param input base中提供的NodeBundle，数据通过NodeBundle中的关键字获取
   * @param token 统计端到端的耗时参数
   * @return 1 代表process函数运行正常
   */
  int32_t AlgProcessNNP(NodeBundle* input, const hz_Adsfi::ProfileToken& token);

  /**
   * @brief 用于在base注册nnp procrss线程，100ms周期触发
   *
   * @param input base中提供的NodeBundle，数据通过NodeBundle中的关键字获取
   * @param token 统计端到端的耗时参数
   * @return 1 代表process函数运行正常
   */
  int32_t AlgProcessAVP(NodeBundle* input, const hz_Adsfi::ProfileToken& token);

  /**
   * @brief 用于在base注册收ehp数据的线程，数据触发，收到map_msg触发一次
   *
   * @param input base中提供的NodeBundle，数据通过NodeBundle中的关键字获取
   * @param token 统计端到端的耗时参数
   * @return 1 代表函数运行正常
   */
  /**
   * @brief 用于在base注册收ehp数据的线程，数据触发，收到map_msg触发一次
   *
   * @param input base中提供的NodeBundle，数据通过NodeBundle中的关键字获取
   * @param token 统计端到端的耗时参数
   * @return 1 代表函数运行正常
   */
  int32_t GetEhpThread(NodeBundle* input, const hz_Adsfi::ProfileToken& token);
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
  void AlgRelease() override;

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
  void GetLaneArray(NodeBundle* input, bool is_nnp_mode);

  /**
   * @brief space获取指定的泊车位信息并转换成proto数据
   *
   * @param input base中提供的NodeBundle，数据通过NodeBundle中的关键字获取
   */
  void GetParkingLot(NodeBundle* input);

  /**
   * @brief 从minieye或者space获取指定的可行驶区域信息并转换成proto数据
   *
   * @param input base中提供的NodeBundle，数据通过NodeBundle中的关键字获取
   * @param is_nnp_mode is nnp mode
   */
  void GetFreeSpace(NodeBundle* input, bool is_nnp_mode);

  /**
  * @brief 获取mcu上行的fct信息并转换成对应的proto
  * 
  * @param input base中提供的NodeBundle，数据通过NodeBundle中的关键字获取
  * @return TaPilotMode 
  */
  TaPilotMode GetFctIn(NodeBundle* input,
                       TL::functionmanager::FunctionManagerIn* fct_in);
  /**
   * @brief 获取mcu上行的fcw和aeb信息，用于给处理座舱hmi显示信息
   *
   * @param input base中提供的NodeBundle，数据通过NodeBundle中的关键字获取
   */
  void GetAebInfo(NodeBundle* input);
  /**
   * @brief 获取mcu上行的mbd debug,只用于转成cyber数据发出去
   *
   * @param input base中提供的NodeBundle，数据通过NodeBundle中的关键字获取
   */
  void GetMbdDebugFromMcu(NodeBundle* input);
  /**
   * @brief 获取mcu上行的adas data,只用于转成cyber数据发出去
   * @param input base中提供的NodeBundle，数据通过NodeBundle中的关键字获取
   */
  void GetAdasDataFromMcu(NodeBundle* input);
  /**
   * @brief Get the Tsr Data object
   * 
   * @param input 获取tsr的限速
   */
  void GetTsrData(NodeBundle* input);

  /**
   * @brief Get the Nns Location object
   * 
   * @param input 
   */
  void GetNnsLocation(NodeBundle* input);
  /**
   * @brief Get the Nns Route object
   * 
   * @param input 
   */
  void GetNnsRoute(NodeBundle* input);

  /**
   * @brief Get the Trigger Config object
   */
  void GetTriggerConfig();

  /**
   * @brief process planning的函数，在输入数据格式转换完之后调用
   *
   * @param token 统计端到端的耗时参数
   */
  void ProcessPlanning(const hz_Adsfi::ProfileToken& token);
  /**
   * @brief pub 线程
   * @return  代表函数运行正常
   */
  void PubTrajectoryThread();

  /**
   * @brief pub轨迹函数，包含轨迹数据结构的转换，pb2struct
   *
   * @param ptr_adc_trajectory ptr_adc_trajectory共享指针
   * @param token 统计时延用
   */
  void PubTrajectory(const ADCTrajectory& adc_trajectory,
                     const hz_Adsfi::ProfileToken& token);
  /**
   * @brief 发送给大屏的hmi信息
   *
   * @param adc_trajectory
   * @param token 统计时延用
   */
  void PubEthHmi(const ADCTrajectory& adc_trajectory,
                 const hz_Adsfi::ProfileToken& token);
  /**
   * @brief 给mcu下发fct状态机信息
   *
   * @param adc_trajectory
   * @param token 统计时延用
   */
  void PubFctOut2MCU(const ADCTrajectory& adc_trajectory,
                     const hz_Adsfi::ProfileToken& token);
  /**
   * @brief 下发到mcu的warnning信息
   *
   * @param adc_trajectory adc_trajectory proto
   * @param token 统计时延用
   */
  void PubWarning(const ADCTrajectory& adc_trajectory,
                  const hz_Adsfi::ProfileToken& token);

  /**
   * @brief 处理给座舱的hmi信息
   *
   * @param local_view local_view共享指针
   * @param ptr_adc_trajectory ptr_adc_trajectory共享指针
   */
  void ProcessWarningAndEthHmi(
      const std::shared_ptr<LocalView>& local_view,
      const std::shared_ptr<ADCTrajectory>& ptr_adc_trajectory);

  /**
   * @brief ProcessRegularTrajectory
   *
   * @param retrived_local_view
   */
  bool ProcessRegularTrajectory(
      const std::shared_ptr<LocalView>& retrived_local_view);
  /**
   * @brief ProcessGuardTrajectory
   *
   * @param retrived_local_view
   * @return true GuardTrajectory enable
   * @return false GuardTrajectory disable
   */
  bool ProcessGuardTrajectory(
      const std::shared_ptr<LocalView>& retrived_local_view);
  /**
   * @brief send output
   *
   * @param token
   * @param output
   */
  void Send(const hz_Adsfi::ProfileToken& token, const std::string& tpoic,
            const std::shared_ptr<hz_Adsfi::AlgDataBase>& data);

  /**
   * @brief check 轨迹点，对应的给fct报nnp的故障
   *
   * @param ptr_adc_trajectory
   */
  void UpdateFctOutByTrajectory(ADCTrajectory* ptr_adc_trajectory);

  /**
   * @brief RecordCpuAndMemInfo
   * 
   * @param ptr_adc_trajectory 
   */
  void RecordCpuAndMemInfo(ADCTrajectory* ptr_adc_trajectory);

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
  hz_Adsfi::ProfileToken GetToken(
      const std::shared_ptr<LocalView>& retrived_local_view);

  /**
 * @brief Publish debug
 * 
 * @param retrived_local_view 
 */
  void PublishDebug(const std::shared_ptr<LocalView>& retrived_local_view);
  /**
   * @brief Get the Mcu 7k Data object
   * 
   * @param input 
   */
  void GetMcu7kData(NodeBundle* input);
  /**
   * @brief Get the Traffic Light And Routing object
   * 
   * @param input 
   */
  void GetTrafficLightAndRouting(NodeBundle* input);
  /**
   * @brief 
   * 
   * @param is_nnp_mode 
   */
  void ClearDataByFctMode(bool is_nnp_mode);

 private:
  Chassis chassis_;

  // perception
  TL::perception::PerceptionObstacles obstacles_minieye_;
  TL::perception::PerceptionObstacles obstacles_;
  // location
  Localization location_;
  // lanemarker
  LaneMarkers lane_markers_minieye_;
  LaneMarkers lane_markers_;
  /// ParkingLotOutArray
  TL::perception::ParkingLotOutArray parking_lot_array_;
  // freespace
  TL::perception::FreeSpaceOutArray free_space_array_;
  TL::planning::PadMessage pad_msg_;
  TL::hmi::NNSLocFrame nns_location_;
  TL::hmi::NNSRouteInfo nns_route_;
  TL::perception::TransportElement traffic_light_;
  TL::perception::TransportElement TL_traffic_light_;
  TL::perception::TransportElement origin_traffic_lights_;
  TL::routing::RoutingRequest routing_request_;
  std::unique_ptr<TL::planning::PlanningBase> regular_planning_base_;
  std::unique_ptr<TL::planning::PlanningBase> guard_planning_base_;
  MbdDebugFromMCU mbd_debug_;
  mcu_to_soc_DebugData adas_data_;
  std::shared_ptr<DependencyInjector> injector_;
  TL::planning::PlanningConfig config_;
  std::atomic<bool> is_pub_thread_stop_{false};
  std::atomic<bool> is_assure_run_once_{false};

  std::unique_ptr<TL::common::ZMQSender> zmq_sender_;
  TL::functionmanager::FunctionManagerIn fct_in_;
  std::unique_ptr<ZMQ2Dreamview> zmq_receiver_;
  VehicleConfigure vehicle_cfg_;
  std::unique_ptr<CollectTriggerManager> collect_trigger_manager_;
  std::unique_ptr<MetricCollect> metric_collect_;
  TL::soc::WarningSwitchMemory warning_switch_mem_;
  TL::common::VehicleParam vehicle_param_;
  TL::planning::warning::WarningStateSwitch last_warning_state_;
  std::future<void> writer_future_;
  bool trajectory_check_ok_ = true;
  std::shared_ptr<const ADCTrajectory> last_regular_adc_trajectory_;
  const std::string avp_topic_ = "avp_main";
  const std::string nnp_topic_ = "nnp_main";
  TL::planning::PublishQueuePtr publish_queue_;
  uint64_t last_process_ms_ = 0;
  const unsigned gap_ = 75;
  double recv_queue_timestamp_ = 0.0;
  TL::control::AdasSomeipFromMCU adas_7k_data_;
  std::atomic_bool has_new_trigger_config_ = false;
  TriggerConfig trigger_config_{};
};
}  // namespace TL
#endif  // PLANNING_PLANNING_H_
