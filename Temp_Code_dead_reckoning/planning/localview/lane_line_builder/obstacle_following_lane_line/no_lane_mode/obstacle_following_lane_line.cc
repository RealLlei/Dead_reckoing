/******************************************************************************
 * Copyright (c) TL Technologies Co., Ltd. 2019-2022. All rights reserved.
 * Author: LingPeng
 * Created Time: 2022/7/17
 *****************************************************************************/

#include "planning/localview/lane_line_builder/obstacle_following_lane_line/obstacle_following_lane_line.h"

#include <memory>

namespace TL {
namespace planning {
ObstacleFollowingLaneLine::ObstacleFollowingLaneLine()
    : nolane_kernel_(new TL::planning::nolane::NoLaneLineKernel()) {}

TL::common::Status ObstacleFollowingLaneLine::Init() {
  return TL::common::Status();
}

TL::common::Status ObstacleFollowingLaneLine::Start() {
  return TL::common::Status();
}

void ObstacleFollowingLaneLine::Stop() {}

bool ObstacleFollowingLaneLine::Process(
    const std::shared_ptr<LocalView>& local_view,
    functionmanager::FunctionManagerOut* const to_fct) {
  // lp: reuse previous lane line only once.
  auto prev_lane =
      nolane_kernel_->MutableGetLaneLineList()->GetLaneLinePtrPrevious();
  bool created_map{false};
  bool lane_line_construct_success = nolane_kernel_->Process(local_view);
  if (!lane_line_construct_success) {
    if (prev_lane) {
      nolane::LogProcess::LogProc("Interface_Process");
      created_map = nolane_kernel_->CreatMapOut(prev_lane);
    }
    nolane_kernel_->MutableGetLaneLineList()->SetLaneLinePtrPrevious(nullptr);
    if (nolane::debug_flag[48]) {
      AERROR << "prev_lane:" << GetPtr(prev_lane) << "  nolane process failed!";
    }
  }
  if (nolane::debug_flag[50]) {
    AERROR << "\n" << nolane::LogProcess::LogProc();
    nolane::LogProcess::Clear();
  }
  return lane_line_construct_success || created_map;
}

const std::shared_ptr<navigation_hdmap::MapMsg>&
ObstacleFollowingLaneLine::GetMapMsg(bool refresh) {
  return nolane_kernel_->GetMapMsg();
}

const std::shared_ptr<routing::RoutingResponse>&
ObstacleFollowingLaneLine::GetRoutingResponse() {
  return nolane_kernel_->GetRoutingResponse();
}
}  // namespace planning
}  // namespace TL
