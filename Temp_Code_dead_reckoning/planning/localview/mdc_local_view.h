/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  mdc_localview
 */

#ifndef PLANNING_LOCALVIEW_MDC_LOCAL_VIEW_H_
#define PLANNING_LOCALVIEW_MDC_LOCAL_VIEW_H_

#include <memory>
#include <shared_mutex>

// #include "adsf/node/node_profiler_token.h"
#include "planning/localview/local_view.h"

// NOLINTBEGIN
using TL::planning::LocalView;
// using hz_Adsfi::ProfileToken;

class MdcLocalView : public LocalView {
 public:
  MdcLocalView() = default;
  ~MdcLocalView() override = default;

  // void SetToken(const ProfileToken& token) { token_ = token; }

  // const ProfileToken& GetToken() const { return token_; }

  //  private:
  //   ProfileToken token_{};
};

// NOLINTEND
#endif  // PLANNING_LOCALVIEW_MDC_LOCAL_VIEW_H_
