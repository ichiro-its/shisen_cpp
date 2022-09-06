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

#include <memory>
#include <string>

#include <shisen_cpp/viewer/node/viewer_node.hpp>

namespace shisen_cpp
{

ViewerNode::ViewerNode(rclcpp::Node::SharedPtr node, const Options & options)
: node(node), options(options)
{
}

ViewerNode::~ViewerNode()
{
}

const std::string & ViewerNode::get_camera_prefix() const
{
  return options.camera_prefix;
}

void ViewerNode::set_consumer(
  std::shared_ptr<ImageConsumer> img_consumer)
{
  image_consumer = img_consumer;

  // Initialize the image subscription
  {
    image_subscription = node->create_subscription<Image>(
      get_camera_prefix() + IMAGE_SUFFIX, 10,
      [this](const Image::SharedPtr msg) {
        auto current_image = *msg;

        // Call callback after image changed
        image_consumer->on_image_changed(current_image);
      });

    RCLCPP_INFO_STREAM(
      node->get_logger(),
      "Image subscription initialized on `" << image_subscription->get_topic_name() << "`!");
  }

  // Initialize the capture setting event subscription
  {
    capture_setting_event_subscription = node->create_subscription<CaptureSettingMsg>(
      get_camera_prefix() + CAPTURE_SETTING_EVENT_SUFFIX, 10,
      [this](const CaptureSettingMsg::SharedPtr msg) {
        change_capture_setting((const CaptureSetting &)*msg);
      });

    RCLCPP_INFO_STREAM(
      node->get_logger(),
      "Capture setting event subscription initialized on `" <<
        capture_setting_event_subscription->get_topic_name() << "`!");
  }

  // Initialize the configure capture setting client
  {
    configure_capture_setting_client = node->create_client<ConfigureCaptureSetting>(
      get_camera_prefix() + CONFIGURE_CAPTURE_SETTING_SUFFIX);

    RCLCPP_INFO(node->get_logger(), "Waiting for configure capture setting server...");
    if (!configure_capture_setting_client->wait_for_service(std::chrono::seconds(3))) {
      RCLCPP_ERROR(node->get_logger(), "Configure capture setting server is not ready!");
      throw std::runtime_error("configure capture setting server is not ready");
    }

    RCLCPP_INFO_STREAM(
      node->get_logger(),
      "Configure capture setting client initialized on `" <<
        configure_capture_setting_client->get_service_name() << "`!");
  }

  // Initial data fetch
  fetch_capture_setting();
}

void ViewerNode::on_capture_setting_changed(const CaptureSetting & /*capture_setting*/)
{
}

void ViewerNode::request_to_configure_capture_setting(
  ConfigureCaptureSetting::Request::SharedPtr request, const CaptureSettingCallback & callback)
{
  configure_capture_setting_client->async_send_request(
    request, [this, callback](rclcpp::Client<ConfigureCaptureSetting>::SharedFuture future) {
      auto response = future.get();
      if (response->capture_setting.size() > 0) {
        change_capture_setting((const CaptureSetting &)response->capture_setting.front());

        // Call callback after future completed
        if (callback) {
          callback(get_capture_setting());
        }
      } else {
        RCLCPP_WARN(
          node->get_logger(),
          "Configure capture setting service response is empty!");
      }
    });
}

void ViewerNode::configure_capture_setting(
  const CaptureSetting & capture_setting, const CaptureSettingCallback & callback)
{
  auto request = std::make_shared<ConfigureCaptureSetting::Request>();
  request->capture_setting.push_back(capture_setting);

  request_to_configure_capture_setting(request, callback);
}

void ViewerNode::fetch_capture_setting(const CaptureSettingCallback & callback)
{
  request_to_configure_capture_setting(
    std::make_shared<ConfigureCaptureSetting::Request>(), callback);
}

const CaptureSetting & ViewerNode::get_capture_setting() const
{
  return current_capture_setting;
}

void ViewerNode::change_capture_setting(const CaptureSetting & capture_setting)
{
  current_capture_setting = capture_setting;

  // Call callback after capture setting changed
  on_capture_setting_changed(get_capture_setting());
}

}  // namespace shisen_cpp
