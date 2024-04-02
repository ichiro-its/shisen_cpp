#include "shisen_cpp/config/grpc/call_data_get_capture_setting.hpp"

#include "shisen_cpp/config/utils/config.hpp"
#include "shisen_interfaces/shisen.grpc.pb.h"
#include "shisen_interfaces/shisen.pb.h"
#include "rclcpp/rclcpp.hpp"

namespace shisen {
CallDataGetCaptureSetting::CallDataGetCaptureSetting(
    shisen_interfaces::proto::Config::AsyncService* service,
    grpc::ServerCompletionQueue* cq, const std::string& path)
    : CallData(service, cq, path) {
    Proceed();
}

void CallDataGetCaptureSetting::AddNextToCompletionQueue() {
    new CallDataGetCaptureSetting(service_, cq_, path_);
}

void CallDataGetCaptureSetting::WaitForRequest() {
    service_->RequestGetCaptureSetting(&ctx_, &request_, &responder_, cq_, cq_, this);
}

void CallDataGetCaptureSetting::HandleRequest() {
    Config config(path_);
    reply_.set_json_capture(config.get_config("capture"));
    RCLCPP_INFO(rclcpp::get_logger("Get config"), "config has been sent!");
}
}  // namespace shisen
