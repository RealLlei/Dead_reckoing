//  Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.

#pragma once

#include "planning/proto/localview_common_data.pb.h"
#include <vector>
namespace TL {
namespace planning {

class LocalViewData {
 public:
  LocalViewData() = default;
  ~LocalViewData() = default;

  void Clear();

  DataUpdate* update_data();
  DataStatic* static_data();
  std::vector<int32_t>* get_cruise_target_ids();
  void set_cruise_target_ids(std::vector<int32_t> cruise_target_ids);

 private:
  DataUpdate update_data_{};
  DataStatic static_data_{};
  std::vector<int32_t> cruise_target_ids_{};
};

}  // namespace planning
}  // namespace TL
