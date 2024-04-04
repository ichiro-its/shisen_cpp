#ifndef SHISEN_CPP__CONFIG__GRPC__CALL_DATA_SAVE_CAPTURE_SETTING_HPP__
#define SHISEN_CPP__CONFIG__GRPC__CALL_DATA_SAVE_CAPTURE_SETTING_HPP__

#include "shisen_cpp/config/grpc/call_data.hpp"
#include "rclcpp/rclcpp.hpp"

namespace shisen_cpp
{
class CallDataSaveCaptureSetting
: CallData<shisen_interfaces::proto::ConfigCapture, shisen_interfaces::proto::Empty>
{
public:
  CallDataSaveCaptureSetting(
    shisen_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
    const std::string & path);

protected:
  void AddNextToCompletionQueue() override;
  void WaitForRequest();
  void HandleRequest();
};

}  // namespace shisen_cpp

#endif  // SHISEN_CPP__CONFIG__GRPC__CALL_DATA_SAVE_CAPTURE_SETTING_HPP__