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

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shisen_cpp/config/grpc/call_data_get_image.hpp>
#include <shisen_interfaces/shisen.grpc.pb.h>
#include <shisen_interfaces/shisen.pb.h>

namespace shisen_cpp
{
CallDataGetImage::CallDataGetImage(
  shisen_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
  const std::string & path, const std::shared_ptr<camera::CameraNode>& camera_node)
: CallData(service, cq, path), camera_node_(camera_node)
{
  Proceed();
}

void CallDataGetImage::AddNextToCompletionQueue()
{
  new CallDataGetImage(service_, cq_, path_, camera_node_);
}

void CallDataGetImage::WaitForRequest()
{
  service_->RequestGetImage(&ctx_, &request_, &responder_, cq_, cq_, this);
}

void CallDataGetImage::HandleRequest()
{
  cv::Mat image = camera_node_->image_provider->get_mat();
  if (image.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("Get image"), "Empty image!");
    return;
  }

  std::vector<uchar> image_bytes;
  cv::imencode(".jpg", image, image_bytes);

  reply_.set_data(image_bytes.data(), image_bytes.size());
  reply_.set_height(image.rows);
  reply_.set_width(image.cols);
}
}  // namespace shisen_cpp
