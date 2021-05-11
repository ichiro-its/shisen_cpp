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

#ifndef SHISEN_CPP__PROVIDER__CAPTURE_SETTING_PROVIDER_HPP_
#define SHISEN_CPP__PROVIDER__CAPTURE_SETTING_PROVIDER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

#include "../node.hpp"

namespace shisen_cpp
{

class CaptureSettingProvider : public CameraNode
{
public:
  struct Options : public virtual CameraNode::Options
  {
  };

  inline explicit CaptureSettingProvider(
    rclcpp::Node::SharedPtr node, const Options & options = Options());

  inline virtual CaptureSetting on_configure_capture_setting(
    const CaptureSetting & capture_setting);

  inline void configure_capture_setting(const CaptureSetting & capture_setting = CaptureSetting());

  inline const CaptureSetting & get_capture_setting() const;

private:
  rclcpp::Publisher<CaptureSettingMsg>::SharedPtr capture_setting_event_publisher;
  rclcpp::Service<ConfigureCaptureSetting>::SharedPtr configure_capture_setting_service;

  CaptureSetting current_capture_setting;
};

CaptureSettingProvider::CaptureSettingProvider(
  rclcpp::Node::SharedPtr node, const CaptureSettingProvider::Options & options)
: CameraNode(node, options)
{
  // Initialize the capture setting event publisher
  {
    capture_setting_event_publisher = get_node()->create_publisher<CaptureSettingMsg>(
      get_camera_prefix() + CAPTURE_SETTING_EVENT_SUFFIX, 10);

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Capture setting event publisher initialized on `" <<
        capture_setting_event_publisher->get_topic_name() << "`!");
  }

  // Initialize the configure capture setting service
  {
    configure_capture_setting_service = get_node()->create_service<ConfigureCaptureSetting>(
      get_camera_prefix() + CONFIGURE_CAPTURE_SETTING_SUFFIX,
      [this](ConfigureCaptureSetting::Request::SharedPtr request,
      ConfigureCaptureSetting::Response::SharedPtr response) {
        // Configure capture setting if exist
        if (request->capture_setting.size() > 0) {
          configure_capture_setting((const CaptureSetting &)request->capture_setting.front());
        }

        response->capture_setting.push_back(get_capture_setting());
      });

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Configure capture setting service initialized on `" <<
        configure_capture_setting_service->get_service_name() << "`!");
  }

  // Initial data fetching
  ConfigureCaptureSetting();
}

CaptureSetting CaptureSettingProvider::on_configure_capture_setting(
  const CaptureSetting & capture_setting)
{
  return capture_setting;
}

void CaptureSettingProvider::configure_capture_setting(const CaptureSetting & capture_setting)
{
  // Update with configured data
  current_capture_setting.update_with(on_configure_capture_setting(capture_setting));
  capture_setting_event_publisher->publish(get_capture_setting());
}

const CaptureSetting & CaptureSettingProvider::get_capture_setting() const
{
  return current_capture_setting;
}

}  // namespace shisen_cpp

#endif  // SHISEN_CPP__PROVIDER__CAPTURE_SETTING_PROVIDER_HPP_
