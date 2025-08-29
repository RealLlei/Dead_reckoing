/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "common/util/perf_util.h"
#include "common/file/log.h"

std::string CalculateHex(int64_t quoti) {
  std::string hex;
  static constexpr int max_size = 30;
  hex.resize(max_size);

  int num = max_size - 1;
  uint temp = 0;
  if (quoti == 0) {
    return "nullptr";
  }
  while (quoti != 0 && num > 0) {
    temp = quoti % 16;
    temp += temp < 10 ? 48 : 55;
    hex[num--] = static_cast<char>(tolower(static_cast<char>(temp)));
    quoti /= 16;
  }
  hex[num--] = 'x';
  hex[num] = '0';
  if (num > 0) {
    return hex.substr(num);
  }
  return hex;
}

namespace {
std::string func_name_simplified(const std::string& str) {
  constexpr char kLeftBracket = '(';
  constexpr char kSpace = ' ';

  auto end = str.find(kLeftBracket);
  auto begin = str.rfind(kSpace, end);

  if (begin == std::string::npos) {
    return str.substr(0, end);
  }
  if (end == std::string::npos) {
    return str.substr(begin + 1);
  }
  return str.substr(begin + 1, end - begin - 1);
}
}  // namespace

using TL::common::Time;  // NOLINT

namespace TL {
namespace common {
namespace util {

std::string function_signature(const std::string& func_name,
                               const std::string& indicator) {
  auto simplified_name = func_name_simplified(func_name);
  if (indicator.empty()) {
    return simplified_name;
  }
  return absl::StrCat(indicator, "_", simplified_name);
}

void Timer::Start() {
  start_time_ = common::Clock::NowInMicroseconds();
}

double Timer::End(const std::string& msg) {
  end_time_ = common::Clock::NowInMicroseconds();
  double elapsed_time = end_time_ - start_time_;
  AINFO << "TIMER " << msg << " elapsed_time: " << elapsed_time << " ms";

  // start new timer.
  start_time_ = end_time_;
  return elapsed_time;
}

}  // namespace util
}  // namespace common
}  // namespace TL
