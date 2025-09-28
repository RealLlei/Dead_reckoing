//  Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.

#include "planning/localview/state_machine/initial_state.h"

#include <memory>

namespace TL {
namespace planning {
bool InitialState::Init() {
  return true;
}

/**
 * @brief Buildlocalview
 * 
 * @param local_view Localview
 * @return Status ERROR code and error message
 */
common::Status InitialState::BuildLocalView(
    const std::shared_ptr<LocalView>& local_view) {
  UNUSED(local_view);
  return common::Status::OK();
}
}  // namespace planning
}  // namespace TL
