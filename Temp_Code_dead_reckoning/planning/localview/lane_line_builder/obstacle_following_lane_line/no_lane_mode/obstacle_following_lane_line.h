/******************************************************************************
 * Copyright (c) TL Technologies Co., Ltd. 2019-2022. All rights reserved.
 * Author: LingPeng
 * Created Time: 2022/7/17
 *****************************************************************************/

#pragma once

#include <memory>

#include "planning/localview/lane_line_builder/lane_line_base.h"
#include "planning/localview/lane_line_builder/obstacle_following_lane_line/no_lane_line_kernel.h"

namespace TL {
namespace planning {

class ObstacleFollowingLaneLine final : public LaneLineBase {
 public:
  ObstacleFollowingLaneLine();

  TL::common::Status Init() override;

  TL::common::Status Start() override;

  void Stop() override;

  bool Process(const std::shared_ptr<LocalView>& local_view,
               functionmanager::FunctionManagerOut* const to_fct) override;

  const std::shared_ptr<navigation_hdmap::MapMsg>& GetMapMsg(
      bool refresh) override;

  const std::shared_ptr<routing::RoutingResponse>& GetRoutingResponse()
      override;

 private:
  std::unique_ptr<TL::planning::nolane::NoLaneLineKernel> nolane_kernel_;
};

}  // namespace planning
}  // namespace TL
