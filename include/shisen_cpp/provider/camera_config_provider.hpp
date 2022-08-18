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

#ifndef SHISEN_CPP__PROVIDER__CAMERA_CONFIG_PROVIDER_HPP_
#define SHISEN_CPP__PROVIDER__CAMERA_CONFIG_PROVIDER_HPP_

#include <keisan/keisan.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

#include "shisen_interfaces/msg/camera_config.hpp"
#include "../node/camera_node.hpp"

namespace shisen_cpp
{
using CameraConfig = shisen_interfaces::msg::CameraConfig;

class CameraConfigProvider : public CameraNode
{
public:
  struct Options : public virtual CameraNode::Options
  {
    int field_of_view;

    Options()
    : field_of_view(-1)
    {
    }
  };

  explicit CameraConfigProvider(
    rclcpp::Node::SharedPtr node, const Options & options = Options());
  ~CameraConfigProvider();

  void set_config(CameraConfig & config, int width, int height);

  const CameraConfig & get_camera_config() const;

private:
  rclcpp::Publisher<CameraConfig>::SharedPtr camera_config_publisher;

  CameraConfig camera_config;
  
  int field_of_view;
};

}  // namespace shisen_cpp

#endif  // SHISEN_CPP__PROVIDER__CAMERA_CONFIG_PROVIDER_HPP_
