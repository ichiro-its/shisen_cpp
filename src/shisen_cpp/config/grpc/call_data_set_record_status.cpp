// Copyright (c) 2024 ICHIRO ITS
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

#include <rclcpp/rclcpp.hpp>
#include <shisen_cpp/config/grpc/call_data_set_record_status.hpp>
#include <shisen_interfaces/shisen.grpc.pb.h>
#include <shisen_interfaces/shisen.pb.h>

namespace shisen_cpp
{
CallDataSetRecordStatus::CallDataSetRecordStatus(
  shisen_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
  const std::string & path, const std::shared_ptr<camera::CameraNode>& camera_node)
: CallData(service, cq, path), camera_node_(camera_node)
{
  Proceed();
}

void CallDataSetRecordStatus::AddNextToCompletionQueue()
{
  new CallDataSetRecordStatus(service_, cq_, path_, camera_node_);
}

void CallDataSetRecordStatus::WaitForRequest()
{
  service_->RequestSetRecordStatus(&ctx_, &request_, &responder_, cq_, cq_, this);
}

void CallDataSetRecordStatus::HandleRequest()
{
  try {
    camera_node_->set_record_status(request_.record());
    RCLCPP_INFO(rclcpp::get_logger("Set record status"), "status has been apply!");
  } catch(const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("Set Record Status"), e.what());
  }
}
}  // namespace shisen_cpp