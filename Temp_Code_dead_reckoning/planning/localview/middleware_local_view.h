/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  mdc_localview
 */

#ifndef PLANNING_LOCALVIEW_MDC_LOCAL_VIEW_H_
#define PLANNING_LOCALVIEW_MDC_LOCAL_VIEW_H_

#include <memory>
#include <shared_mutex>
#ifdef ISMDC
#include "adsf/node/node_profiler_token.h"
#endif
#ifdef ISORIN
#include "adf/include/node_profiler_token.h"
#endif

#include "planning/localview/local_view.h"
// NOLINTBEGIN
using TL::planning::LocalView;
#ifdef ISMDC
using hz_Adsfi::ProfileToken;
#endif
#ifdef ISORIN
using TL::netaos::adf::ProfileToken;
#endif
#ifdef ISX86
using TL::netaos::adf::ProfileToken;
#endif
class MiddleWareLocalView : public LocalView {
 public:
  MiddleWareLocalView() = default;
  ~MiddleWareLocalView() = default;

  void SetToken(const ProfileToken& token) { token_ = token; }

  const ProfileToken& GetToken() const { return token_; }

 private:
  ProfileToken token_;
};

// NOLINTEND
#endif  // PLANNING_LOCALVIEW_MDC_LOCAL_VIEW_H_
