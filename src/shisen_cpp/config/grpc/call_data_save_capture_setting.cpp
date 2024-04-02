#include "shisen_cpp/config/grpc/call_data_save_capture_setting.hpp"

#include "shisen_cpp/config/utils/config.hpp"
#include "shisen_interfaces/shisen.grpc.pb.h"
#include "shisen_interfaces/shisen.pb.h"
#include "rclcpp/rclcpp.hpp"

namespace shisen
{
CallDataSaveCaptureSetting::CallDataSaveCaptureSetting(
  shisen_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
  const std::string & path)
: CallData(service, cq, path)
{
  Proceed();
}

void CallDataSaveCaptureSetting::AddNextToCompletionQueue()
{
  new CallDataSaveCaptureSetting(service_, cq_, path_);
}

void CallDataSaveCaptureSetting::WaitForRequest()
{
  service_->RequestSaveConfig(&ctx_, &request_, &responder_, cq_, cq_, this);
}

void CallDataSaveCaptureSetting::HandleRequest()
{
  Config config(path_);
  try {
    nlohmann::json capture_data = nlohmann::json::parse(request_.json_capture());
    config.save_config(capture_data);
    RCLCPP_INFO(rclcpp::get_logger("Save config"), " config has been saved!  ");
  } catch (nlohmann::json::exception e) {
    RCLCPP_ERROR(rclcpp::get_logger("Save config"), e.what());
  }
}
}  // namespace shisen