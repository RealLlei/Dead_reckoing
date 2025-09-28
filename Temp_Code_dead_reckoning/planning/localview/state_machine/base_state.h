
//  Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.

#pragma once

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/front/functor_row.hpp>
#include <boost/msm/front/internal_row.hpp>
#include <boost/msm/front/state_machine_def.hpp>

#include "common/configs/config_gflags.h"
#include "common/file/file.h"
#include "common/math/math_utils.h"
#include "common/math/vec2d.h"
#include "map/hdmap/hdmap_util.h"
#include "planning/pnc_map/pnc_map.h"
#include "planning/localview/local_view.h"
#include "planning/localview/state_machine/event.h"
#include "proto/perception/perception_parking_lot.pb.h"
#include "proto/routing/poi.pb.h"
#include "planning/localview/localview_comdata_manager.h"

namespace TL {
namespace planning {
// NOLINTBEGIN
#define SETACTIONSTRUCT(STRUCT_NAME, FUNCTION_NAME)                        \
  struct STRUCT_NAME {                                                     \
   public:                                                                 \
    template <class StateMachine, class Event, class SourceState,          \
              class TargetState>                                           \
    void operator()(const Event& event, const StateMachine& state_machine, \
                    const SourceState& source_state,                       \
                    const TargetState& target_state) {                     \
      UNUSED(source_state);                                                \
      const_cast<TargetState&>(target_state)                               \
          .FUNCTION_NAME(const_cast<Event*>(&event),                       \
                         const_cast<StateMachine*>(&state_machine));       \
    }                                                                      \
  };

// NOLINTEND
/**
 * @class BaseState
 * @brief
 */
class BaseState : public boost::msm::front::state<> {
 public:
  /**
   * @brief 处理BuildLocalView的action
   * @param event 引起状态转移的事件
   * @param state_machine 状态机对象
   * @param source_state 源状态
   * @param target_state 目标状态
   */
  BaseState() = default;
  virtual ~BaseState() = default;

  bool virtual Init() = 0;

  virtual common::Status BuildLocalView(
      const std::shared_ptr<LocalView>& local_view) = 0;
  std::shared_ptr<LocalViewData> local_view_data_ = nullptr;
};

}  // namespace planning
}  // namespace TL
