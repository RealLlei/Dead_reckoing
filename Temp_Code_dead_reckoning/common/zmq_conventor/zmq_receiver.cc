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

#include "common/zmq_conventor/zmq_receiver.h"

#include "common/file/log.h"

namespace TL {
namespace common {

ZMQReceiver::ZMQReceiver() = default;

ZMQReceiver::~ZMQReceiver() {
  Stop();
}

void ZMQReceiver::Init(const std::string& ip, const std::string& port) {
  auto ip_port_ = ip + ":" + port;
  zmq_ctx_ = zmq::context_t(1);
  zmq_socket_subscriber_ = zmq::socket_t(zmq_ctx_, zmq::socket_type::sub);
  zmq_socket_subscriber_.setsockopt(ZMQ_RCVBUF, 20 * 1024 * 1024);

  zmq_socket_subscriber_.setsockopt(ZMQ_RCVTIMEO, 20);
  zmq_socket_subscriber_.connect(ip_port_);
  zmq_socket_subscriber_.set(zmq::sockopt::subscribe, "");
  is_init_ = true;
}

void ZMQReceiver::Start() {
  zmq_thread_ = std::make_unique<std::thread>([this] { UpdateThreadFunc(); });
  if (zmq_thread_ == nullptr) {
    AERROR << "Unable to create zmq thread.";
    return;
  }
  is_running_ = true;
}

void ZMQReceiver::Stop() {
  if (is_init_) {
    if (is_running_ && zmq_thread_ != nullptr && zmq_thread_->joinable()) {
      is_running_ = false;
      zmq_thread_->join();
      zmq_thread_.reset();
    }
    zmq_socket_subscriber_.close();
    zmq_ctx_term(&zmq_ctx_);
  }
}

void ZMQReceiver::UpdateThreadFunc() {
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  while (is_running_) {
    std::vector<zmq::message_t> recv_msgs;
    zmq::recv_multipart(zmq_socket_subscriber_, std::back_inserter(recv_msgs));
    std::vector<std::string> msg;
    msg.reserve(recv_msgs.size());
    for (const auto& one : recv_msgs) {
      msg.push_back(one.to_string());
    }
    ADEBUG << "receive msg size: " << recv_msgs.size();
    if (!recv_msgs.empty()) {
      Process(msg);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

void ZMQReceiver::Process(const std::vector<std::string>& msg) {  // NOLINT
  ADEBUG << "Receiver process ";
}

}  // namespace common
}  // namespace TL
