/******************************************************************************
 * Copyright 2019 The TL Authors. All Rights Reserved.
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
#ifndef _OPEN_SPACE_THREAD_MANAGER_H_
#define _OPEN_SPACE_THREAD_MANAGER_H_

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <set>
#include <thread>
#include <utility>
#include <vector>
#include "common/file/log.h"
#include "common/time/clock.h"
#include "planning/common/planning_gflags.h"

namespace TL {
namespace planning {

template <typename Config, typename SearchClass, typename SmoothClass,
          typename Input, typename Output, typename Debug>
class OpenSpaceThreadManager {
  struct ThreadCoreInfo {
    std::condition_variable cv;
    std::future<void> future;
    std::mutex cv_mutex;
    std::mutex data_mutex;
    std::atomic_bool stop_flag{false};
    std::atomic_bool suspend_flag{true};
    std::atomic_bool finish_flag{true};

    /**
     * Initializes the function.
     */
    void Init() {
      stop_flag.store(false);
      suspend_flag.store(true);
      finish_flag.store(true);
    }

    /**
     * Stops the function execution by setting flags and notifying all threads.
     */
    void Stop() {
      stop_flag.store(true);
      suspend_flag.store(true);
      finish_flag.store(true);
      cv.notify_all();
      if (future.valid()) {
        future.get();
      }
    }
  };

 public:
  OpenSpaceThreadManager(size_t thread_num, const Config& config)
      : thread_num_(thread_num),
        config_(config),
        park_ids_(thread_num, -1),
        search_thread_infos_(thread_num),
        search_inputs_(thread_num),
        search_outputs_(thread_num),
        search_debugs_(thread_num),
        smooth_path_generator_(config.warm_start_config()),
        smoother_(config.nlp_path_smoother_config()),
        search_plan_disabled_{false} {
    ADEBUG << "open space thread manager ctor";
    Init();
  }

  OpenSpaceThreadManager(const OpenSpaceThreadManager&) = delete;
  OpenSpaceThreadManager& operator=(const OpenSpaceThreadManager&) = delete;
  OpenSpaceThreadManager(OpenSpaceThreadManager&&) = delete;

  ~OpenSpaceThreadManager() {
    StopAllSearchThreads();
    StopSmoothThread();
  }

  /**
   * PrePlan function preplans the search inputs for the given search_inputs.
   *
   * @param search_inputs the vector of Input objects containing the search inputs
   *
   * @throws None
   */
  void PrePlan(const std::vector<Input>& search_inputs) {
    if (search_plan_disabled_) {
      return;
    }

    // find new park id and already parkid ->already planning thread id
    std::vector<int> new_park_indexs;
    std::vector<int> already_park_thread_indexs;
    for (int i = 0;
         i < static_cast<int>(std::min(search_inputs.size(), thread_num_));
         i++) {
      int index = GetParkThreadIndex(search_inputs[i].path_id);
      if (index >= thread_num_) {
        new_park_indexs.push_back(i);
      } else {
        already_park_thread_indexs.push_back(index);
      }
    }

    // find can resume thread index
    std::vector<int> resume_thread_indexs;
    for (int i = 0; i < thread_num_; i++) {
      if (std::find(already_park_thread_indexs.begin(),
                    already_park_thread_indexs.end(),
                    i) == already_park_thread_indexs.end()) {
        resume_thread_indexs.push_back(i);
      }
    }

    // resume thread use new park id
    auto resume_size = static_cast<int>(
        std::min(new_park_indexs.size(), resume_thread_indexs.size()));
    for (int i = 0; i < resume_size; i++) {
      ADEBUG << "Resume thread id: " << resume_thread_indexs[i] << " due to"
             << " new park id: " << search_inputs[new_park_indexs[i]].path_id
             << " old park id: " << park_ids_[resume_thread_indexs[i]]
             << " finish falg: "
             << search_thread_infos_[resume_thread_indexs[i]].finish_flag;
      ResumeSearchThread(resume_thread_indexs[i],
                         search_inputs[new_park_indexs[i]]);
    }

    // free thread should be suspend
    for (int i = resume_size; i < resume_thread_indexs.size(); i++) {
      ADEBUG << "Suspend thread id: " << resume_thread_indexs[i]
             << " park id: " << park_ids_[resume_thread_indexs[i]]
             << " finish falg: "
             << search_thread_infos_[resume_thread_indexs[i]].finish_flag;
      SuspendSearchThread(resume_thread_indexs[i]);
    }
  }

  /**
   * TargetPlan function.
   *
   * @param search_input Input object representing the search input.
   *
   * @return void
   *
   * @throws None
   */
  void TargetPlan(const Input& search_input) {
    search_plan_disabled_ = true;
    ResumeSearchSmoothThread(search_input);
  }

  /**
   * Gets the target output and debug information.
   *
   * @param output a pointer to the output object to be filled
   * @param debug a pointer to the debug object to be filled
   *
   * @return true if successful in getting the target output and debug information, false otherwise
   *
   * @throws None
   */
  bool GetTargetOutput(Output* const output, Debug* const debug) {
    if (smooth_thread_info_.suspend_flag.load()) {
      *output = smooth_search_output_;
      *debug = smooth_search_debug_;
      return true;
    }
    return false;
  }

  /**
   * Retrieves the output and debug information for a given park ID.
   *
   * @param park_id the ID of the park
   * @param output a pointer to an Output object to store the retrieved output
   * @param debug a pointer to a Debug object to store the retrieved debug information
   *
   * @return true if the retrieval was successful, false otherwise
   *
   * @throws None
   */
  bool GetParkOutput(const int park_id, Output* const output,
                     Debug* const debug) {
    int index = GetParkThreadIndex(park_id);
    if (index >= thread_num_) {
      return false;
    }
    if (search_thread_infos_[index].finish_flag.load() &&
        search_thread_infos_[index].suspend_flag.load()) {
      *output = search_outputs_[index];
      *debug = search_debugs_[index];
      return true;
    }
    return false;
  }

  std::vector<int> GetThreadParkIds() { return park_ids_; }

 private:
  /**
   * Suspends the search thread with the given thread ID.
   *
   * @param thread_id the ID of the thread to suspend
   *
   * @return void
   *
   * @throws None
   */
  void SuspendSearchThread(const int thread_id) {
    if (IndexInValid(thread_id)) {
      return;
    }

    search_thread_infos_[thread_id].suspend_flag.store(true);
    park_ids_[thread_id] = -1;
  }

  /**
 * Resumes a search thread.
 *
 * @param thread_id the ID of the thread to resume
 * @param search_input the input for the search thread
 *
 * @throws None
 */
  void ResumeSearchThread(const int thread_id, const Input& search_input) {
    if (IndexInValid(thread_id)) {
      return;
    }
    if (!search_thread_infos_[thread_id].stop_flag.load()) {
      search_thread_infos_[thread_id].suspend_flag.store(true);
      park_ids_[thread_id] = search_input.path_id;
      double start_time = common::Clock::NowInSeconds();
      while (!search_thread_infos_[thread_id].finish_flag.load()) {
        if ((common::Clock::NowInSeconds() - start_time) >
            FLAGS_open_space_thread_manager_max_wait_time) {
          park_ids_[thread_id] = -1;
          return;
        }
      }

      ADEBUG << "start resume thread id: " << thread_id;
      // don't chang finish flag, change it in thread is actual execute
      search_inputs_[thread_id] = search_input;
      search_thread_infos_[thread_id].suspend_flag.store(false);
      search_thread_infos_[thread_id].cv.notify_all();
    }
  }

  /**
   * Resumes the smooth search thread if it is not finished and suspended.
   *
   * @param search_input the input for the search
   *
   * @throws None
   */
  void ResumeSearchSmoothThread(const Input& search_input) {
    // if smooth thread is not finished, do nothing
    if (!smooth_thread_info_.stop_flag.load() &&
        smooth_thread_info_.suspend_flag.load()) {
      ADEBUG << "Resume smooth thread";
      smooth_search_input_ = search_input;
      smooth_thread_info_.suspend_flag.store(false);
      smooth_thread_info_.cv.notify_all();
    }
  }

  /**
   * Stop the search thread with the given thread ID.
   *
   * @param thread_id the ID of the thread to stop
   *
   * @return void
   *
   * @throws None
   */
  void StopSearchThread(const int thread_id) {
    if (IndexInValid(thread_id) ||
        search_thread_infos_[thread_id].stop_flag.load()) {
      return;
    }

    park_ids_[thread_id] = -1;
    search_thread_infos_[thread_id].Stop();
    ADEBUG << "thread: " << thread_id << " stoped!!";
  }

  /**
   * Stops the smooth thread.
   *
   * @param None
   *
   * @return None
   *
   * @throws None
   */
  void StopSmoothThread() {
    smooth_thread_info_.Stop();
    ADEBUG << "thread smooth stop!!";
  }

  /**
   * Stops all search threads.
   *
   * @param None
   *
   * @return None
   *
   * @throws None
   */
  void StopAllSearchThreads() {
    for (int i = 0; i < thread_num_; i++) {
      StopSearchThread(i);
    }
  }

  bool IndexInValid(const int i) { return i < 0 || i >= thread_num_; }

  /**
   * Get the index of the park thread.
   *
   * @param park_id the ID of the park
   *
   * @return the index of the park thread
   *
   * @throws None
   */
  int GetParkThreadIndex(const int park_id) {
    if (park_id < 0) {
      return thread_num_;
    }
    return std::distance(
        park_ids_.begin(),
        std::find(park_ids_.begin(), park_ids_.end(), park_id));
  }

  /**
   * OpenSpacePathSearch is a function that performs a path search in an open space.
   *
   * @param i the index of the search thread
   *
   * @throws None
   */
  void OpenSpacePathSearch(const int i) {
    if (IndexInValid(i)) {
      return;
    }

    while (!search_thread_infos_[i].stop_flag.load()) {
      std::unique_lock<std::mutex> lock(search_thread_infos_[i].cv_mutex);
      search_thread_infos_[i].cv.wait(lock, [&]() {
        return search_thread_infos_[i].stop_flag.load() ||
               !search_thread_infos_[i].suspend_flag.load();
      });

      if (search_thread_infos_[i].stop_flag.load()) {
        break;
      }
      // this line present thread start, change finish flag
      search_thread_infos_[i].finish_flag.store(false);

      SearchClass search(config_.warm_start_config());
      search.Plan(search_thread_infos_[i].suspend_flag, search_inputs_[i],
                  &search_outputs_[i]);
      search.UpdateDebugInfo(&search_debugs_[i]);
      search_thread_infos_[i].finish_flag.store(true);
      search_thread_infos_[i].suspend_flag.store(true);
      ADEBUG << "thread " << i
             << " search finished, park id: " << search_inputs_[i].path_id;
    }
  }

  /**
   * OpenSpacePathSearchSmooth function performs smooth path search in an open space.
   *
   * @throws None
   */
  void OpenSpacePathSearchSmooth() {
    while (!smooth_thread_info_.stop_flag.load()) {
      std::unique_lock<std::mutex> lock(smooth_thread_info_.cv_mutex);
      smooth_thread_info_.cv.wait(lock, [&]() {
        return smooth_thread_info_.stop_flag.load() ||
               !smooth_thread_info_.suspend_flag.load();
      });

      if (smooth_thread_info_.stop_flag.load()) {
        break;
      }

      int index = GetParkThreadIndex(smooth_search_input_.path_id);
      if (index >= thread_num_) {
        ADEBUG << "target id not found !!!";
        StopAllSearchThreads();
        smooth_path_generator_.Plan(smooth_thread_info_.suspend_flag,
                                    smooth_search_input_,
                                    &smooth_search_output_);
        smooth_path_generator_.UpdateDebugInfo(&smooth_search_debug_);
      } else {
        ADEBUG << "target id found: " << index << " wait result........";
        // wait search result
        double start_time = common::Clock::NowInSeconds();
        while (!(search_thread_infos_[index].suspend_flag.load() &&
                 search_thread_infos_[index].finish_flag.load())) {
          if ((common::Clock::NowInSeconds() - start_time) >
              FLAGS_open_space_thread_manager_max_wait_time) {
            break;
          }
        }
        ADEBUG << " ....... thread: " << index
               << " get result success, start smooth";
        smooth_search_output_ = search_outputs_[index];
        smooth_search_debug_ = search_debugs_[index];
        StopAllSearchThreads();
      }

      // SmoothOutput smooth_output;
      smoother_.Smooth(smooth_search_input_, &smooth_search_output_);
      smoother_.UpdateDebugInfo(&smooth_search_debug_);
      smooth_thread_info_.suspend_flag.store(true);
      ADEBUG << "smooth finished, target park id: "
             << smooth_search_input_.path_id;
    }
  }

  /**
 * Initializes the OpenSpaceThreadManager by initializing the search_thread_infos_ 
 * array and launching the OpenSpacePathSearch function in multiple threads.
 *
 * @throws std::exception if an error occurs during initialization
 */
  void Init() {
    for (int i = 0; i < thread_num_; i++) {
      search_thread_infos_[i].Init();
      search_thread_infos_[i].future =
          std::async(std::launch::async,
                     &OpenSpaceThreadManager::OpenSpacePathSearch, this, i);
    }
    smooth_thread_info_.future =
        std::async(std::launch::async,
                   &OpenSpaceThreadManager::OpenSpacePathSearchSmooth, this);
  }

  size_t thread_num_;
  const Config& config_;

  // search thread only search
  std::vector<int> park_ids_;
  std::vector<ThreadCoreInfo> search_thread_infos_;
  std::vector<Input> search_inputs_;
  std::vector<Output> search_outputs_;
  std::vector<Debug> search_debugs_;

  // smooth thread serach and smooth
  ThreadCoreInfo smooth_thread_info_;
  Input smooth_search_input_;
  Output smooth_search_output_;
  Debug smooth_search_debug_;
  SearchClass smooth_path_generator_;
  SmoothClass smoother_;

  // a flag to balance search and smooth
  bool search_plan_disabled_;
};

}  // namespace planning
}  // namespace TL

#endif
