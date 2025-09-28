/*
 * @Author: 80040285 zhangyu@TLauto.com
 * @Date: 2023-08-31 13:56:57
 * @LastEditors: 80040285 zhangyu@TLauto.com
 * @LastEditTime: 2023-09-06 16:28:30
 * @FilePath: /europa/planning/localview/lane_line_builder/lane_line_base.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * Copyright (c) TL auto Co., Ltd. 2021-2022. All rights reserved.
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "common/configs/vehicle_config_helper.h"
#include "common/math/vec2d.h"
#include "common/status/status.h"
#include "common/util/message_util.h"
#include "common/util/util.h"
#include "map/hdmap/hdmap_util.h"
#include "planning/common/planning_gflags.h"
#include "planning/localview/local_view.h"
#include "planning/localview/localview_comdata_manager.h"
#include "planning/pnc_map/pnc_map.h"

namespace TL {
namespace planning {

static constexpr uint32_t kLaneSuccess = 0x10;
// 建图PILOT模式下，视觉地图和巡航模式默认有效(除了前方目标存在标识不置有效)
static constexpr uint32_t kMapfusionAdasSuccess = 0x12;
// 建图NNP模式下，视觉地图和巡航模式默认有效
static constexpr uint32_t kMapfusionPilotSuccess = 0x13;
// soc_2_fct_tbd_u32_03的bit16下发到mcu用于判断NCP模式
static constexpr uint32_t kIsNcpMode = 0x10000;

class LaneLineBase {
 public:
  LaneLineBase() = default;

  /**
   * @brief destructor
   */
  virtual ~LaneLineBase() = default;

  /**
   * @brief module initialization function
   * @return initialization status
   */
  virtual TL::common::Status Init() = 0;

  /**
   * @brief module start function
   * @return start status
   */
  virtual TL::common::Status Start() = 0;

  /**
   * @brief module stop function
   */
  virtual void Stop() = 0;

  /**
   * @brief main logic of the navigation_hdmap module, runs periodically
   * triggered by timer.
   */
  virtual bool Process(const std::shared_ptr<LocalView>& local_view,
                       functionmanager::FunctionManagerOut* to_fct) = 0;
  virtual const std::shared_ptr<navigation_hdmap::MapMsg>& GetMapMsg(
      bool refresh) = 0;
  virtual const std::shared_ptr<routing::RoutingResponse>&
  GetRoutingResponse() = 0;

  static bool JudgeIsInMapFirstly(const std::shared_ptr<LocalView>& local_view,
                                  const std::shared_ptr<hdmap::HDMap>& hd_map);
  static bool JudgeIsInMapContinuously(
      const std::shared_ptr<LocalView>& local_view,
      const std::shared_ptr<hdmap::HDMap>& hd_map);
  static bool IsNewRoutingRequest(
      const routing::RoutingRequest& prev_routing_request,
      const routing::RoutingRequest& routing_request);
  std::shared_ptr<LocalViewData> local_view_data_ = nullptr;
};
}  // namespace planning
}  // namespace TL
