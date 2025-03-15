// Copyright (c) 2025 Ichiro ITS
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

#ifndef SHISEN_CPP__CONFIG__NODE__CONFIG_NODE_HPP_
#define SHISEN_CPP__CONFIG__NODE__CONFIG_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <shisen_interfaces/srv/get_capture_setting.hpp>
#include <shisen_interfaces/srv/update_capture_setting.hpp>
#include <shisen_cpp/camera/node/camera_node.hpp>

#include <memory>
#include <string>

namespace shisen_cpp::camera
{

class ConfigNode
{
public:
  using GetCaptureSetting = shisen_interfaces::srv::GetCaptureSetting;
  using UpdateCaptureSetting = shisen_interfaces::srv::UpdateCaptureSetting;

  explicit ConfigNode(rclcpp::Node::SharedPtr node, const std::string & path,
    const std::shared_ptr<CameraNode> & camera_node);

private:
  std::string get_node_prefix() const;

  rclcpp::Service<GetCaptureSetting>::SharedPtr get_capture_setting_service;
  rclcpp::Service<UpdateCaptureSetting>::SharedPtr update_capture_setting_service;
};

}  // namespace shisen_cpp::camera

#endif  // SHISEN_CPP__CONFIG__NODE__CONFIG_NODE_HPP_
