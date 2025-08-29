/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  common thread pool
 * Author: ROC
 */

#include "common/thread/thread_pool.h"
#include <algorithm>

namespace TL {
namespace common {
namespace thread {

thread_local int ThreadPool::s_thread_pool_level = 0;
std::vector<int> BaseThreadPool::THREAD_POOL_CAPACITY = {  // NOLINT
    FLAGS_threadpool_level0_thread_num, FLAGS_threadpool_level1_thread_num,
    FLAGS_threadpool_level2_thread_num};

void BaseThreadPool::Stop() {
  io_.stop();
  thread_group_.join_all();
  stopped_ = true;
}

BaseThreadPool::~BaseThreadPool() {  // NOLINT
  if (!stopped_) {
    try {                                                             // NOLINT
      Stop();                                                         // NOLINT
    } catch (std::exception& e) {                                     // NOLINT
      AERROR << "ThreadPool: stop thread pool failed. " << e.what();  // NOLINT
    }                                                                 // NOLINT
  }
}

BaseThreadPool::BaseThreadPool(int thread_num, int next_thread_pool_level)
    : BaseThreadPool(thread_num, next_thread_pool_level, 0) {}

BaseThreadPool::BaseThreadPool(int thread_num, int next_thread_pool_level,
                               int pool_index)
    : work_(io_) {
  AINFO << "ThreadPool: Starting thread pool, thread_num = " << thread_num
        << ", index = " << pool_index;
  for (int i = 0; i != thread_num; ++i) {
    thread_group_.create_thread([this, next_thread_pool_level, pool_index, i] {
      ThreadPool::s_thread_pool_level = next_thread_pool_level;
      this->io_.run();
    });
  }
}

BaseThreadPool* ThreadPool::Instance() {
  int pool_level = s_thread_pool_level;
  int max_thread_pool_level =
      static_cast<int>(BaseThreadPool::THREAD_POOL_CAPACITY.size());
  CHECK_LT(pool_level, max_thread_pool_level);
  int index = s_thread_pool_level;
  switch (index) {  // NOLINT
    case 0: {
      return LevelThreadPool<0>::Instance();
    }
    case 1: {
      return LevelThreadPool<1>::Instance();
    }
    case 2: {
      return LevelThreadPool<2>::Instance();
    }
    default: {
      return LevelThreadPool<0>::Instance();
    }
  }
  AERROR << "ThreadPool: Should not hit here";
  return LevelThreadPool<0>::Instance();
}

}  // namespace thread
}  // namespace common
}  // namespace TL
