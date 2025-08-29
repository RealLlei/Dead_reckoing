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

#include "common/time/clock.h"

// #include "cyber/common/global_data.h"
#include "common/configs/config_gflags.h"
#include "common/file/log.h"

// #include "cyber/common/util.h"

namespace TL {
namespace common {

// using GlobalData = ::TL::cyber::common::GlobalData;

using AtomicRWLock = ::TL::common::base::AtomicRWLock;
using AtomicWriteLockGuard =
    ::TL::common::base::WriteLockGuard<AtomicRWLock>;
using AtomicReadLockGuard = ::TL::common::base::ReadLockGuard<AtomicRWLock>;

Clock::Clock() {}  // NOLINT

TL::common::Time Clock::Now() {
  auto* clock = Instance();  // NOLINT

  AtomicReadLockGuard lg(clock->rwlock_);
  return TL::common::Time::Now();
}

double Clock::NowInSeconds() {
#ifdef FOR_BAIDU_SIMULATION
  if (FLAGS_is_record_replay || FLAGS_enable_planning_self_simulator) {
    return Now().ToSecond();
  }
  return apollo::sim::Time::Now().ToSecond();
#endif
  return Now().ToSecond();
}

double Clock::NowInMicroseconds() {
#ifdef FOR_BAIDU_SIMULATION
  if (FLAGS_is_record_replay || FLAGS_enable_planning_self_simulator) {
    return Now().ToSecond() * 1e3;
  }
  return apollo::sim::Time::Now().ToSecond() * 1e3;
#endif
  return Now().ToSecond() * 1e3;
}

double Clock::NowInSecondsForBeiJing() {
  struct timespec time = {0};
  clock_gettime(12, &time);
  uint64_t micro_secs = time.tv_sec * 1000000 + time.tv_nsec / 1000;

  return static_cast<double>(micro_secs) / 1000000.0;
}

double Clock::NowInMicrosecondsForBeiJing() {
  struct timespec time = {0};
  clock_gettime(12, &time);
  uint64_t micro_secs = time.tv_sec * 1000000 + time.tv_nsec / 1000;

  return static_cast<double>(micro_secs) / 1000.0;
}

uint64_t Clock::NowInNanoseconds() {
#ifdef FOR_BAIDU_SIMULATION
  if (FLAGS_is_record_replay || FLAGS_enable_planning_self_simulator) {
    return Now().ToNanosecond();
  }
  return apollo::sim::Time::Now().ToNanosecond();
#endif
  return Now().ToNanosecond();
}

std::string Clock::NowToString() {
#ifdef FOR_BAIDU_SIMULATION
  if (FLAGS_is_record_replay || FLAGS_enable_planning_self_simulator) {
    return Now().ToString();
  }
  return apollo::sim::Time::Now().ToString();
#endif
  return Now().ToString();
}

// void Clock::SetMode(ClockMode mode) {
//   auto clock = Instance();
//   AtomicWriteLockGuard lg(clock->rwlock_);
//   switch (mode) {
//     case ClockMode::MODE_MOCK: {
//       clock->mode_ = mode;
//       break;
//     }
//     case ClockMode::MODE_CYBER: {
//       clock->mode_ = mode;
//       break;
//     }
//     default:
//       AERROR << "Unknown ClockMode: " << mode;
//   }
//   clock->mock_now_ = Time(0);
// }

// ClockMode Clock::mode() {
//   auto clock = Instance();
//   AtomicReadLockGuard lg(clock->rwlock_);
//   return clock->mode_;
// }

// void Clock::SetNow(const Time& now) {
//   auto clock = Instance();
//   AtomicWriteLockGuard lg(clock->rwlock_);
//   if (clock->mode_ != ClockMode::MODE_MOCK) {
//     AERROR << "SetSimNow only works for ClockMode::MOCK";
//     return;
//   }
//   clock->mock_now_ = now;
// }

}  // namespace common
}  // namespace TL
