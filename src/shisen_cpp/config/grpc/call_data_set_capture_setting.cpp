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

#include "rclcpp/rclcpp.hpp"
#include "shisen_cpp/config/grpc/call_data_set_capture_setting.hpp"
#include "shisen_cpp/config/utils/config.hpp"
#include "shisen_interfaces/shisen.grpc.pb.h"
#include "shisen_interfaces/shisen.pb.h"

namespace shisen_cpp
{
CallDataSetCaptureSetting::CallDataSetCaptureSetting(
  shisen_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
  const std::string & path, rclcpp::Node::SharedPtr node, std::shared_ptr<camera::CameraNode> camera_node)
: CallData(service, cq, path), node_(node), camera_node_(camera_node)
{
  set_capture_publisher_ = node_->create_publisher<shisen_interfaces::msg::CaptureSetting>(
    "shisen_cpp/config/capture_setting", 10);
  Proceed();
}

void CallDataSetCaptureSetting::AddNextToCompletionQueue()
{
  new CallDataSetCaptureSetting(service_, cq_, path_, node_, camera_node_);
}

void CallDataSetCaptureSetting::WaitForRequest()
{
  service_->RequestSetCaptureSetting(&ctx_, &request_, &responder_, cq_, cq_, this);
}

void CallDataSetCaptureSetting::HandleRequest()
{
  Config config(path_);
  try {
    CaptureSetting capture_setting;

    int brightness = request_.brightness();
    int contrast = request_.contrast();
    int saturation = request_.saturation();
    int temperature = request_.temperature();
    int exposure = request_.exposure();
    int gain = request_.gain();

    capture_setting.brightness.set(brightness);
    capture_setting.contrast.set(contrast);
    capture_setting.saturation.set(saturation);
    capture_setting.temperature.set(temperature);
    capture_setting.exposure.set(exposure);
    capture_setting.gain.set(gain);

    camera_node_->configure_capture_setting(capture_setting);
    // shisen_interfaces::msg::CaptureSetting msg;
    // msg.brightness.clear();
    // msg.contrast.clear();
    // msg.saturation.clear();
    // msg.temperature.clear();
    // msg.exposure.clear();
    // msg.gain.clear();

    // msg.brightness.push_back(brightness);
    // msg.contrast.push_back(contrast);
    // msg.saturation.push_back(saturation);
    // msg.temperature.push_back(temperature);
    // msg.exposure.push_back(exposure);
    // msg.gain.push_back(gain);

    // set_capture_publisher_->publish(msg);
    RCLCPP_INFO(
      rclcpp::get_logger("Publish capture setting config"),
      "capture setting config has been applied!");
  } catch (nlohmann::json::exception e) {
    RCLCPP_ERROR(rclcpp::get_logger("Publish config"), e.what());
  }
}
}  // namespace shisen_cpp
