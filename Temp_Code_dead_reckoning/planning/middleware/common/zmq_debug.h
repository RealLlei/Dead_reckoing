/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  planning debug
 */

#pragma once

#include <list>
#include <map>
#include <memory>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#ifdef ISMDC
#include "adsf/node/node_base.h"
#include "adsfi/include/data_types/planning/planning_debug.h"
#include "core/core.h"
#endif
#include "common/status/status.h"
#include "common/zmq_conventor/zmq_sender.h"

#include "planning/localview/local_view.h"
#include "planning/proto/planning_config.pb.h"
#include "proto/control/mbd_control_debug.pb.h"
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
#include "proto/soc/chassis.pb.h"

/**
 * @namespace TL::planning::debug
 * @brief ozon::planning::debug
 */
namespace TL {
namespace planning {
namespace ZmqDebug {
using TL::common::Header;
using TL::common::Status;
using TL::control::MbdDebugFromMCU;
using TL::functionmanager::FunctionManagerIn;
using TL::hdmap::Map;
using TL::localization::Localization;
using TL::navigation_hdmap::MapMsg;
using TL::perception::LaneMarkers;
using TL::perception::TransportElement;
using TL::planning::ADCTrajectory;
using TL::planning::EventTrigger;
using TL::planning::LocalView;
using TL::prediction::PredictionObstacles;
using TL::soc::Chassis;
using TL::soc::WarningFault;
#ifdef ISMDC
using hz_Adsfi::NodeBase;
using hz_Adsfi::NodeBundle;
#endif

/**
 * @brief
 * 把proto序列化成string，用zmq发出，cyber收到后反序列化转成cyber，用于显示
 *
 * @param local_view local_view共享指针
 * @param adc_trajectory adc_trajectory轨迹信息
 * @param output base中提供的NodeBundle，数据通过的关键字存储发送出去
 */
#ifdef ISMDC
void SerializeProtoToString(
    const std::shared_ptr<LocalView>& local_view,
    const ADCTrajectory& adc_trajectory,
    const std::shared_ptr<hz_Adsfi::DebugPlanningFrame>& planning_debug,
    const std::unique_ptr<TL::common::ZMQSender>& zmq_sender);
#endif

#ifdef ISORIN
void SerializeProtoToString(
    const std::shared_ptr<LocalView>& local_view,
    const ADCTrajectory& adc_trajectory,
    std::vector<std::string>* const msg_vec,
    const std::unique_ptr<TL::common::ZMQSender>& zmq_sender);

#endif

}  // namespace ZmqDebug
}  // namespace planning
}  // namespace TL
