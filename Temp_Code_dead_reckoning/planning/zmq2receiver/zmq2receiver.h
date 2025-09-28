/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include <algorithm>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>

#include "common/util/message_util.h"
#include "common/zmq_conventor/zmq_receiver.h"

#include "proto/fsm/function_manager.pb.h"
#include "proto/perception/transport_element.pb.h"
#include "proto/routing/routing.pb.h"

namespace TL {
namespace planning {

class DreamviewReceiver : public TL::common::ZMQReceiver {
 public:
  void Process(const std::vector<std::string>& msg) override;
  TL::functionmanager::FunctionManagerIn& GetFunctionManagerIn();
  TL::perception::TransportElement& GetTransportElement();
  TL::routing::RoutingRequest& GetRoutingRequest();

 private:
  void PublishPlaning(const std::vector<std::string>& msg);
  TL::functionmanager::FunctionManagerIn fct_in_;
  TL::perception::TransportElement transport_element_;
  TL::routing::RoutingRequest routing_requese_;

  std::mutex zmq_mutex_;
};

class ZMQ2Dreamview {
 public:
  ZMQ2Dreamview();
  void Init();
  void Start();
  void Stop();
  TL::functionmanager::FunctionManagerIn& GetFunctionManagerIn();
  TL::perception::TransportElement& GetTransportElement();
  TL::routing::RoutingRequest& GetRoutingRequest();

 private:
  std::unique_ptr<DreamviewReceiver> dreamview_receiver_;
};

}  // namespace planning
}  // namespace TL
