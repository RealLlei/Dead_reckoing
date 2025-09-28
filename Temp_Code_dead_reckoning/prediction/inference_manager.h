/******************************************************************************
 * Copyright 2022 The TL Authors. All Rights Reserved.
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

/**
 * @file
 */
#pragma once

#include <cstddef>
#include <memory>
#include <vector>
#include "common/file/log.h"
#if defined(ISMDC)
#include "core/core.h"
#include "core/group.h"
#include "dnn/dnn.h"
#elif defined(ISORIN)
#include "NvInfer.h"
#else
#include <onnxruntime_cxx_api.h>

#include <string>
#endif

/**
 * @namespace TL::prediction
 * @brief TL::prediction
 */
namespace TL {
namespace prediction {

#ifdef ISORIN
class Logger : public nvinfer1::ILogger {
 public:
  explicit Logger(Severity severity = Severity::kWARNING)
      : reportable_severity(severity) {}

  void log(Severity severity, const char* msg) noexcept override {
    if (severity > reportable_severity)
      return;

    switch (severity) {
      case Severity::kINTERNAL_ERROR:
        AERROR << "INTERNAL_ERROR: ";
        break;
      case Severity::kERROR:
        AERROR << "ERROR: ";
        break;
      case Severity::kWARNING:
        AERROR << "WARNING: ";
        break;
      case Severity::kINFO:
        AERROR << "INFO: ";
        break;
      default:
        AERROR << "UNKNOWN: ";
        break;
    }
    AERROR << msg;
  }

  Severity reportable_severity;
};
#endif

class InferenceManager {
 public:
  InferenceManager() {
#ifdef ISMDC
    adsfi_context_ptr_ = std::make_unique<Adsfi::HafContext>();
    Adsfi::HafContextParameters contextParam{};

    if (Adsfi::HafInitialize(*adsfi_context_ptr_, contextParam) !=
        Adsfi::HAF_SUCCESS) {
      AERROR << "Initialize NN device failed.";
    }
    if (Adsfi::HafSetGroup(aicore_group_id_) != Adsfi::HAF_SUCCESS) {
      AERROR << "HafInitialize failed!";
    }
    ADEBUG << "Adsfi::HafInitialize";
#elif defined(ISORIN)
    nv_runtime_.reset(nvinfer1::createInferRuntime(g_logger_));
#else
    ort_env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING,
                                          instanceName_.c_str());
    ort_session_options_ = std::make_unique<Ort::SessionOptions>();
    ort_session_options_->SetIntraOpNumThreads(1);
    ort_session_options_->SetGraphOptimizationLevel(
        GraphOptimizationLevel::ORT_ENABLE_ALL);
    ort_session_options_->SetLogSeverityLevel(4);
#endif
  }

  ~InferenceManager() = default;

 public:
#ifdef ISMDC
  const std::unique_ptr<Adsfi::HafContext>& adsfi_context() {
    return adsfi_context_ptr_;
  }
#elif defined(ISORIN)
  const std::shared_ptr<nvinfer1::IRuntime>& nv_runtime() {
    return nv_runtime_;
  }
#else
  const std::unique_ptr<Ort::Env>& ort_env() { return ort_env_; }

  const std::unique_ptr<Ort::SessionOptions>& ort_session_options() {
    return ort_session_options_;
  }
#endif

 private:
#ifdef ISMDC
  size_t aicore_group_id_ = 3;
  std::unique_ptr<Adsfi::HafContext> adsfi_context_ptr_;
#elif defined(ISORIN)
  Logger g_logger_;
  std::shared_ptr<nvinfer1::IRuntime> nv_runtime_;
#else
  std::unique_ptr<Ort::Env> ort_env_;
  std::unique_ptr<Ort::SessionOptions> ort_session_options_;
  std::string instanceName_{"prediction-inference"};
#endif
};

}  // namespace prediction
}  // namespace TL
