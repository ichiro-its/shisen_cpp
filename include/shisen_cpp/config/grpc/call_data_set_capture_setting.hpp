#ifndef SHISEN_CPP_CONFIG__GRPC__CALL_DATA_SET_CAPTURE_SETTING_HPP__
#define SHISEN_CPP_CONFIG__GRPC__CALL_DATA_SET_CAPTURE_SETTING_HPP__

#include "shisen_cpp/config/grpc/call_data.hpp"
#include "shisen_interfaces/msg/set_capture.hpp"
#include "rclcpp/rclcpp.hpp"

namespace shisen
{
class CallDataSetCaptureSetting
: CallData<shisen_interfaces::proto::CaptureSetting, shisen_interfaces::proto::Empty>
{
public:
  CallDataSetCaptureSetting(
    shisen_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
    const std::string & path, rclcpp::Node::SharedPtr node);

protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<shisen_interfaces::msg::SetCapture>::SharedPtr set_capture_publisher_;
  void AddNextToCompletionQueue() override;
  void WaitForRequest();
  void HandleRequest();
};
}  // namespace shisen

#endif  // SHISEN_CPP__CONFIG__GRPC__CALL_DATA_SET_CAPTURE_SETTING_HPP__