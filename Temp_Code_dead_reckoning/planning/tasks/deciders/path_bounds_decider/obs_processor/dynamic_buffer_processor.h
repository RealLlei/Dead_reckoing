/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  planning dynamic obstacle buffer processor
 * Author: ROC
 */

#pragma once

#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "planning/common/obstacle.h"
#include "planning/proto/dynamic_buffer_config.pb.h"

namespace TL {
namespace planning {

class DynamicBufferProcessor {
 public:
  DynamicBufferProcessor();
  DynamicBufferProcessor(const DynamicBufferProcessor& dynamicBufferCalculate) =
      delete;
  DynamicBufferProcessor& operator=(
      const DynamicBufferProcessor& dynamicbufferCalculate) = delete;
  ~DynamicBufferProcessor() = default;

  /**
   * @brief Calculate a obstacle dynamic buffer
   * 
   * @param obs_s 
   * @param obs_l 
   * @param cur_v 
   * @return double 
   */
  double BufferCalculate(double obs_s, double obs_l, double cur_v);

  /**
   * @brief Calculate a obstacle dynamic buffer
   * 
   * @param cur_v 
   * @param relative_v 
   * @param obstacle 
   * @return double 
   */
  double BufferCalculate(double cur_v, double relative_v,
                         const Obstacle& obstacle);

  /**
   * @brief Dynamic Obs Dynamic Buffer Calculate
   * 
   * @param cur_v 
   * @return double 
   */
  double DynamicObsDynamicBufferCalculate(double cur_v) const;

  /**
   * @brief Static Obs Dynamic Buffer Calculate
   * 
   * @param cur_v 
   * @return double 
   */
  double StaticObsDynamicBufferCalculate(double cur_v) const;

 private:
  /**
   * @brief Check whether get data from proto sucessfully, if not return false,
   *    otherwise get the threshold value.
   * 
   * @return true: regular 
   * @return false: can not get data from proto sucessfully 
   */
  bool Init();

  DynamicBufferConfig dynamic_buffer_config_;
  double lower_threshold_ = 0.0;
  double upper_threshold_ = 0.5;
  bool init_status_ = false;
};
}  // namespace planning
}  // namespace TL
