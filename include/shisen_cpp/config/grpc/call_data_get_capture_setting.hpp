#ifndef SHISEN_CPP__CONFIG__GRPC__CALL_DATA_GET_CAPTURE_SETTING_HPP__
#define SHISEN_CPP__CONFIG__GRPC__CALL_DATA_GET_CAPTURE_SETTING_HPP__

#include "shisen_cpp/config/grpc/call_data.hpp"

namespace shisen
{
class CallDataGetCaptureSetting
  : CallData<shisen_interfaces::proto::Empty, shisen_interfaces::proto::CaptureSetting>
  {
  public:
    CallDataGetCaptureSetting(
      shisen_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
      const std::string & path);

  protected:
    void AddNextToCompletionQueue() override;
    void WaitForRequest() ;
    void HandleRequest() ;
  };
} // namespace shisen

#endif // SHISEN_CPP__CONFIG__GRPC__CALL_DATA_GET_CAPTURE_SETTING_HPP__
