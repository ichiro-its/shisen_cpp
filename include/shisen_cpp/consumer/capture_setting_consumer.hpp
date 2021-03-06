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

#ifndef SHISEN_CPP__CONSUMER__CAPTURE_SETTING_CONSUMER_HPP_
#define SHISEN_CPP__CONSUMER__CAPTURE_SETTING_CONSUMER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <future>
#include <memory>
#include <string>

#include "../node.hpp"

namespace shisen_cpp
{

class CaptureSettingConsumer : public CameraNode
{
public:
  using CaptureSettingCallback = std::function<void (const CaptureSetting &)>;

  struct Options : public virtual CameraNode::Options
  {
  };

  explicit CaptureSettingConsumer(
    rclcpp::Node::SharedPtr node, const Options & options = Options());

  ~CaptureSettingConsumer();

  virtual void on_capture_setting_changed(const CaptureSetting & capture_setting);

  void request_to_configure_capture_setting(
    ConfigureCaptureSetting::Request::SharedPtr request,
    const CaptureSettingCallback & callback = {});

  void configure_capture_setting(
    const CaptureSetting & capture_setting, const CaptureSettingCallback & callback = {});

  void fetch_capture_setting(const CaptureSettingCallback & callback = {});

  const CaptureSetting & get_capture_setting() const;

private:
  void change_capture_setting(const CaptureSetting & capture_setting);

  rclcpp::Subscription<CaptureSettingMsg>::SharedPtr capture_setting_event_subscription;
  rclcpp::Client<ConfigureCaptureSetting>::SharedPtr configure_capture_setting_client;

  CaptureSetting current_capture_setting;
};

}  // namespace shisen_cpp

#endif  // SHISEN_CPP__CONSUMER__CAPTURE_SETTING_CONSUMER_HPP_
