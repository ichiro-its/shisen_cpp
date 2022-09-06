// Copyright (c) 2021 ICHIRO ITS
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

#ifndef SHISEN_CPP__CAMERA__NODE__CAMERA_NODE_HPP_
#define SHISEN_CPP__CAMERA__NODE__CAMERA_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

#include "shisen_cpp/camera/provider/camera_config_provider.hpp"
#include "shisen_cpp/camera/provider/image_provider.hpp"

#include "shisen_cpp/utility.hpp"

namespace shisen_cpp
{
using CameraConfig = shisen_interfaces::msg::CameraConfig;

class CameraNode
{
public:
  explicit CameraNode(rclcpp::Node::SharedPtr node, const Options & options = Options());
  ~CameraNode();

  void update();
  void on_mat_captured(cv::Mat mat);
  void on_camera_config(int width, int height);
  CaptureSetting on_configure_capture_setting(const CaptureSetting & capture_setting);
  void configure_capture_setting(const CaptureSetting & capture_setting = CaptureSetting());

  cv::Mat get_mat();
  const std::string & get_camera_prefix() const;

  void set_provider(
    std::shared_ptr<ImageProvider> img_provider,
    std::shared_ptr<CameraConfigProvider> cam_config_provider);

  std::shared_ptr<ImageProvider> image_provider;
  std::shared_ptr<CameraConfigProvider> camera_config_provider;

private:
  rclcpp::Node::SharedPtr node;
  CaptureSetting current_capture_setting;

  rclcpp::Publisher<Image>::SharedPtr image_publisher;
  rclcpp::Publisher<CameraConfig>::SharedPtr camera_config_publisher;

  rclcpp::Publisher<CaptureSettingMsg>::SharedPtr capture_setting_event_publisher;
  rclcpp::Service<ConfigureCaptureSetting>::SharedPtr configure_capture_setting_service;

  Options options;
};

}  // namespace shisen_cpp

#endif  // SHISEN_CPP__CAMERA__NODE__CAMERA_NODE_HPP_
