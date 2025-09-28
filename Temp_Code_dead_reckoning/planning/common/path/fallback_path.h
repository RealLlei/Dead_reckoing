#ifndef PLANNING_COMMON_PATH_FALLBACK_PATH_H
#define PLANNING_COMMON_PATH_FALLBACK_PATH_H

/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  planning
 */

#pragma once

#include <memory>
#include "planning/common/dependency_injector.h"
#include "planning/common/frame.h"
#include "planning/common/path/path_data.h"

namespace TL {
namespace planning {
class FallBackPath {
 public:
  FallBackPath() = delete;
  FallBackPath(const ScenarioConfig::StageConfig& config,
               const std::shared_ptr<DependencyInjector>& injector);
  ~FallBackPath() = default;

  bool GenerateFallbackPathProfile(const Frame* frame,
                                   ReferenceLineInfo* reference_line_info,
                                   PathData* path_data) const;

 private:
  void GenerateFallbackPathProfile(
      const ReferenceLineInfo* reference_line_info, PathData* path_data,
      const functionmanager::MachineStateType& type) const;
  bool RetrieveLastFramePathProfile(
      const ReferenceLineInfo* reference_line_info, const Frame* frame,
      PathData* path_data) const;

 private:
  const std::shared_ptr<DependencyInjector>& injector_;
};

}  // namespace planning
}  // namespace TL

#endif  // PLANNING_COMMON_PATH_FALLBACK_PATH_H
