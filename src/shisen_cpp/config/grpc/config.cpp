#include "shisen_cpp/config/utils/config.hpp"

#include <chrono>
#include <csignal>
#include <future>
#include <string>

#include "shisen_cpp/config/grpc/call_data_base.hpp"
#include "shisen_cpp/config/grpc/call_data_get_capture_setting.hpp"
#include "shisen_cpp/config/grpc/call_data_save_capture_setting.hpp"
#include "shisen_cpp/config/grpc/call_data_set_capture_setting.hpp"
#include "shisen_cpp/config/grpc/config.hpp"
#include "rclcpp/rclcpp.hpp"

using grpc::ServerBuilder;
using namespace std::chrono_literals;

namespace shisen_cpp
{
ConfigGrpc::ConfigGrpc() {}
ConfigGrpc::ConfigGrpc(const std::string & path) : path(path) {}

ConfigGrpc::~ConfigGrpc()
{
  server_->Shutdown();
  cq_->Shutdown();
}

void ConfigGrpc::SignIntHandler(int signum)
{
  server_->Shutdown();
  cq_->Shutdown();
  exit(signum);
}

void ConfigGrpc::Run(uint16_t port, const std::string & path, rclcpp::Node::SharedPtr node)
{
  Config config(path);
  std::string server_address =
    absl::StrFormat("0.0.0.0:%d", config.get_grpc_config()["port"].get<uint16_t>());

  ServerBuilder builder;
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  builder.RegisterService(&service_);

  cq_ = builder.AddCompletionQueue();
  server_ = builder.BuildAndStart();
  std::cout << "Server listening on " << server_address << std::endl;

  signal(SIGINT, SignIntHandler);
  thread_ = std::thread([&path, &node, this]() {
    new CallDataGetCaptureSetting(&service_, cq_.get(), path);
    new CallDataSaveCaptureSetting(&service_, cq_.get(), path);
    new CallDataSetCaptureSetting(&service_, cq_.get(), path, node);
    void * tag;  // uniquely identifies a request.
    bool ok = true;
    while (true) {
      this->cq_->Next(&tag, &ok);
      if (ok) {
        static_cast<CallDataBase *>(tag)->Proceed();
      }
    }
  });
  std::this_thread::sleep_for(200ms);
}

}  // namespace shisen_cpp
