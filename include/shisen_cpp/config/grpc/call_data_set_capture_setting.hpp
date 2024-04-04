#ifndef SHISEN_CPP_CONFIG__GRPC__CALL_DATA_SET_CAPTURE_SETTING_HPP__
#define SHISEN_CPP_CONFIG__GRPC__CALL_DATA_SET_CAPTURE_SETTING_HPP__

#include "shisen_cpp/config/grpc/call_data.hpp"
#include "shisen_interfaces/msg/capture_setting.hpp"
#include "rclcpp/rclcpp.hpp"

namespace shisen_cpp
{
class CallDataSetCaptureSetting
: CallData<shisen_interfaces::proto::CaptureSetting, shisen_interfaces::proto::Empty>
{
public:
  CallDataSetCaptureSetting(
    shisen_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
    const std::string & path, rclcpp::Node::SharedPtr node);

protected:
  void AddNextToCompletionQueue() override;
  void WaitForRequest();
  void HandleRequest();
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<shisen_interfaces::msg::CaptureSetting>::SharedPtr set_capture_publisher_;
};
}  // namespace shisen_cpp

#endif  // SHISEN_CPP__CONFIG__GRPC__CALL_DATA_SET_CAPTURE_SETTING_HPP__