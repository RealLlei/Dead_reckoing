//  Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
#include "planning/localview/localview_comdata_manager.h"

#include <utility>

namespace TL {
namespace planning {

void LocalViewData::Clear() {
  update_data_.Clear();
}

DataUpdate* LocalViewData::update_data() {
  return &update_data_;
}

DataStatic* LocalViewData::static_data() {
  return &static_data_;
}

std::vector<int32_t>* LocalViewData::get_cruise_target_ids() {
  return &cruise_target_ids_;
}

void LocalViewData::set_cruise_target_ids(
    std::vector<int32_t> cruise_target_ids) {
  cruise_target_ids_ = std::move(cruise_target_ids);
}

}  // namespace planning
}  // namespace TL
