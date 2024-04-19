// Copyright (c) 2024 ICHIRO ITS
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef SHISEN_CPP__CONFIG__GRPC__CONFIG_HPP_
#define SHISEN_CPP__CONFIG__GRPC__CONFIG_HPP_

#include <absl/flags/flag.h>
#include <absl/flags/parse.h>
#include <absl/strings/str_format.h>
#include <grpc/support/log.h>
#include <grpcpp/grpcpp.h>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shisen_cpp/config/grpc/call_data.hpp>
#include <shisen_cpp/config/grpc/call_data_base.hpp>
#include <shisen_interfaces/shisen.grpc.pb.h>
#include <shisen_interfaces/shisen.pb.h>
#include <shisen_interfaces/msg/capture_setting.hpp>
#include <shisen_interfaces/msg/camera_config.hpp>

#include <shisen_cpp/camera/node/camera_node.hpp>

#include <chrono>
#include <fstream>
#include <future>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <thread>


using shisen_interfaces::proto::Config;

namespace shisen_cpp
{
class ConfigGrpc
{
public:
  explicit ConfigGrpc();
  explicit ConfigGrpc(const std::string & path);

  ~ConfigGrpc();

  void Run(uint16_t port, const std::string & path, rclcpp::Node::SharedPtr node, std::shared_ptr<camera::CameraNode> camera_node);

private:
  std::string path;
  static void SignIntHandler(int signum);

  static inline std::unique_ptr<grpc::ServerCompletionQueue> cq_;
  static inline std::unique_ptr<grpc::Server> server_;
  std::thread thread_;
  shisen_interfaces::proto::Config::AsyncService service_;
  std::shared_ptr<camera::CameraNode> camera_node_;
};

}  // namespace shisen_cpp

#endif  // SHISEN_CPP__CONFIG__GRPC__CONFIG_HPP_
