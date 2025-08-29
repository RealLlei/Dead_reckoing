/*
 * Copyright (c) TL Auto Co., Ltd. 2019-2022. All rights reserved.
 * Description: control debug receive base
 */
#include "common/file/global_data.h"

namespace TL {
namespace common {
thread_local std::string sub_thread_name = "_main";  // NOLINT
std::string module_name = "undefined";               // NOLINT
}  // namespace common
}  // namespace TL
