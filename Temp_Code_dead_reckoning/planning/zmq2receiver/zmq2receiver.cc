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

#include "planning/zmq2receiver/zmq2receiver.h"
#include "common/adapters/adapter_gflags.h"
#include "common/configs/config_gflags.h"

namespace TL {
namespace planning {

void DreamviewReceiver::Process(const std::vector<std::string>& msg) {
  PublishPlaning(msg);
}

void DreamviewReceiver::PublishPlaning(
    const std::vector<std::string>& recv_msgs) {
  std::lock_guard<std::mutex> lock(zmq_mutex_);
  if (recv_msgs.size() >= 1) {
    fct_in_.ParseFromString(recv_msgs.at(0));
  }
  if (recv_msgs.size() >= 2) {
    transport_element_.ParseFromString(recv_msgs.at(1));
  }
  if (recv_msgs.size() >= 3) {
    routing_requese_.ParseFromString(recv_msgs.at(2));

  } else {
    AERROR << "msg size:" << recv_msgs.size();
  }
}

TL::functionmanager::FunctionManagerIn&
DreamviewReceiver::GetFunctionManagerIn() {
  std::lock_guard<std::mutex> lock(zmq_mutex_);
  return fct_in_;
}

TL::routing::RoutingRequest& GetRoutingRequest();

TL::perception::TransportElement& DreamviewReceiver::GetTransportElement() {
  std::lock_guard<std::mutex> lock(zmq_mutex_);
  return transport_element_;
}

TL::routing::RoutingRequest& DreamviewReceiver::GetRoutingRequest() {
  std::lock_guard<std::mutex> lock(zmq_mutex_);
  return routing_requese_;
}

ZMQ2Dreamview::ZMQ2Dreamview() {
  dreamview_receiver_ = std::make_unique<DreamviewReceiver>();
}

void ZMQ2Dreamview::Init() {
  dreamview_receiver_->Init(FLAGS_zmq_publisher_dreamview_ip,
                            std::to_string(FLAGS_zmq_data_dreamview_port));
}

TL::functionmanager::FunctionManagerIn&
ZMQ2Dreamview::GetFunctionManagerIn() {
  return dreamview_receiver_->GetFunctionManagerIn();
}

TL::perception::TransportElement& ZMQ2Dreamview::GetTransportElement() {
  return dreamview_receiver_->GetTransportElement();
}

TL::routing::RoutingRequest& ZMQ2Dreamview::GetRoutingRequest() {
  return dreamview_receiver_->GetRoutingRequest();
}

void ZMQ2Dreamview::Start() {
  dreamview_receiver_->Start();
}

void ZMQ2Dreamview::Stop() {
  dreamview_receiver_->Stop();
}

}  // namespace planning
}  // namespace TL
