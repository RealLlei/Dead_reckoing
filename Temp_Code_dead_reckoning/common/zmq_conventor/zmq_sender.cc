
//   Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.

#include "common/zmq_conventor/zmq_sender.h"

#include "common/file/log.h"

namespace TL {
namespace common {

ZMQSender::~ZMQSender() {
  Stop();
}

void ZMQSender::Init(const std::string& port) {
  zmq_ctx_ = zmq::context_t(1);
  zmq_socket_publisher_ = zmq::socket_t(zmq_ctx_, zmq::socket_type::pub);
  // zmq_socket_publisher_.setsockopt(ZMQ_SNDHWM, 10000);
  zmq_socket_publisher_.setsockopt(ZMQ_SNDBUF, 20 * 1024 * 1024);
  zmq_socket_publisher_.setsockopt(ZMQ_SNDTIMEO, 20);
  zmq_socket_publisher_.bind("tcp://*:" + port);
  is_init_ = true;
}

void ZMQSender::Process(const std::vector<std::string>& msg) {
  std::vector<zmq::const_buffer> c_bufs;
  c_bufs.reserve(msg.size());
  for (const auto& one : msg) {
    c_bufs.emplace_back(zmq::buffer(one));
  }
  zmq::send_multipart(zmq_socket_publisher_, c_bufs);
  ADEBUG << "send msg size " << msg.size();
}

void ZMQSender::Stop() {
  if (is_init_) {
    zmq_socket_publisher_.close();
    zmq_ctx_term(&zmq_ctx_);
  }
}

}  // namespace common
}  // namespace TL
