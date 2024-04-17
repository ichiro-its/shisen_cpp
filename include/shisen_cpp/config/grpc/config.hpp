#ifndef SHISEN_CPP__CONFIG__GRPC__CONFIG_HPP_
#define SHISEN_CPP__CONFIG__GRPC__CONFIG_HPP_

#include <chrono>
#include <fstream>
#include <future>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/strings/str_format.h"
#include "shisen_cpp/config/grpc/call_data.hpp"
#include "shisen_cpp/config/grpc/call_data_base.hpp"
#include "shisen_interfaces/shisen.grpc.pb.h"
#include "shisen_interfaces/shisen.pb.h"
#include "shisen_interfaces/msg/capture_setting.hpp"
#include "shisen_interfaces/msg/camera_config.hpp"
#include "grpc/support/log.h"
#include "grpcpp/grpcpp.h"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"

using shisen_interfaces::proto::Config;

namespace shisen_cpp
{
class ConfigGrpc
{
public:
  explicit ConfigGrpc();
  explicit ConfigGrpc(const std::string & path);

  ~ConfigGrpc();

  void Run(uint16_t port, const std::string & path, rclcpp::Node::SharedPtr node);

private:
  std::string path;
  static void SignIntHandler(int signum);

  static inline std::unique_ptr<grpc::ServerCompletionQueue> cq_;
  static inline std::unique_ptr<grpc::Server> server_;
  std::thread thread_;
  shisen_interfaces::proto::Config::AsyncService service_;
};

}  // namespace shisen_cpp

#endif  // SHISEN_CPP__CONFIG__GRPC__CONFIG_HPP_
