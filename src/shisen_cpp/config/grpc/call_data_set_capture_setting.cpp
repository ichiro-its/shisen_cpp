#include "shisen_cpp/config/grpc/call_data_set_capture_setting.hpp"

#include "shisen_cpp/config/utils/config.hpp"
#include "shisen_interfaces/shisen.grpc.pb.h"
#include "shisen_interfaces/shisen.pb.h"
#include "rclcpp/rclcpp.hpp"

namespace shisen
{
CallDataSetCaptureSetting::CallDataSetCaptureSetting(
  shisen_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
  const std::string & path, rclcpp::Node::SharedPtr node)
: CallData(service, cq, path), node_(node)
{
  set_capture_publisher_ =
    node_->create_publisher<shisen_interfaces::msg::SetCapture>("shisen_cpp/config/set_capture", 10);
  Proceed();
}

void CallDataSetCaptureSetting::AddNextToCompletionQueue()
{
  new CallDataSetCaptureSetting(service_, cq_, path_, node_);
}

void CallDataSetCaptureSetting::WaitForRequest()
{
  service_->RequestPublishConfig(&ctx_, &request_, &responder_, cq_, cq_, this);
}

void CallDataSetCaptureSetting::HandleRequest()
{
  try {
    shisen_interfaces::msg::SetConfig msg;
    msg.json_capture = request_.json_capture();
    set_capture_publisher_->publish(msg);
    RCLCPP_INFO(rclcpp::get_logger("Publish config"), "config has been published!  ");
  } catch (nlohmann::json::exception e) {
    RCLCPP_ERROR(rclcpp::get_logger("Publish config"), e.what());
  }
}
}  // namespace shisen