// Copyright (c) 2024 Ichiro ITS
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

#ifndef __SHISEN_CPP__CONFIG__GRPC__CALL_DATA_HPP__
#define __SHISEN_CPP__CONFIG__GRPC__CALL_DATA_HPP__

#include <shisen_cpp/config/grpc/call_data_base.hpp>
#include <shisen_interfaces/shisen.grpc.pb.h>
#include <shisen_interfaces/shisen.pb.h>

enum class CallStatus
{
  CREATE,
  PROCESS,
  FINISH,
};

namespace shisen_cpp
{
template <class ConfigRequest, class ConfigReply>
class CallData : public CallDataBase
{
public:
  CallData(
    shisen_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
    const std::string & path);

  void Proceed() override;

protected:
  virtual void AddNextToCompletionQueue() = 0;

  CallStatus status_;  // The current serving state.

  shisen_interfaces::proto::Config::AsyncService * service_;

  const std::string path_;

  grpc::ServerCompletionQueue * cq_;
  grpc::ServerContext ctx_;
  ConfigRequest request_;
  ConfigReply reply_;
  grpc::ServerAsyncResponseWriter<ConfigReply> responder_;
};

template <class ConfigRequest, class ConfigReply>
CallData<ConfigRequest, ConfigReply>::CallData(
  shisen_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
  const std::string & path)
: status_(CallStatus::CREATE), service_(service), cq_(cq), responder_(&ctx_), path_(path)
{
}

template <class ConfigRequest, class ConfigReply>
void CallData<ConfigRequest, ConfigReply>::Proceed()
{
  switch (status_) {
    case CallStatus::CREATE:
      status_ = CallStatus::PROCESS;
      WaitForRequest();
      break;
    case CallStatus::PROCESS:
      AddNextToCompletionQueue();
      HandleRequest();
      status_ = CallStatus::FINISH;
      responder_.Finish(reply_, grpc::Status::OK, this);
      break;
    case CallStatus::FINISH:
      delete this;
      break;
  }
}

}  // namespace shisen_cpp

#endif  // __SHISEN_CPP__CONFIG__GRPC__CALL_DATA_HPP__
