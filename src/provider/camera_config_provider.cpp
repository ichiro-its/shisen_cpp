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

#include <shisen_cpp/provider/camera_config_provider.hpp>

namespace shisen_cpp
{

CameraConfigProvider::CameraConfigProvider(
  rclcpp::Node::SharedPtr node, const CameraConfigProvider::Options & options)
: CameraNode(node, options)
{
  // Initialize the camera config publisher
  {
    camera_config_publisher = get_node()->template create_publisher<CameraConfig>(
      get_camera_prefix() + CAMERA_CONFIG_SUFFIX, 10);

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Camera Config publisher initialized on `" << camera_config_publisher->get_topic_name() <<
        "`!");
  }

  // Initial data publish
  set_camera_config(get_camera_config());
}

CameraConfigProvider::~CameraConfigProvider()
{
}

void CameraConfigProvider::set_camera_config(const CameraConfig & config)
{
  camera_config = config;

  // Publish changes
  camera_config_publisher->publish(get_camera_config());
}

const CameraConfig & CameraConfigProvider::get_camera_config() const
{
  return camera_config;
}

}  // namespace shisen_cpp
