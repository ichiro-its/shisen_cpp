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

#include <shisen_cpp/config/node/config_node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <jitsuyo/config.hpp>
#include <shisen_cpp/utility/capture_setting.hpp>
#include <shisen_cpp/viewer/node/viewer_node.hpp>

#include <memory>
#include <string>

namespace shisen_cpp::camera
{

ConfigNode::ConfigNode(rclcpp::Node::SharedPtr node, const std::string & path,
  const std::shared_ptr<camera::CameraNode> & camera_node)
{
  get_capture_setting_service = node->create_service<GetCaptureSetting>(
    get_node_prefix() + "/get_capture_setting",
    [this, path, camera_node](std::shared_ptr<GetCaptureSetting::Request> request,
    std::shared_ptr<GetCaptureSetting::Response> response) {
      camera_node->load_configuration(path);
      CaptureSetting capture_setting = camera_node->get_capture_setting();
      response->brightness = capture_setting.brightness;
      response->contrast = capture_setting.contrast;
      response->saturation = capture_setting.saturation;
      response->temperature = capture_setting.temperature;
      response->exposure = capture_setting.exposure;
      response->gain = capture_setting.gain;
    }
  );

  update_capture_setting_service = node->create_service<UpdateCaptureSetting>(
    get_node_prefix() + "/update_capture_setting",
    [this, node, path, camera_node](std::shared_ptr<UpdateCaptureSetting::Request> request,
    std::shared_ptr<UpdateCaptureSetting::Response> response) {
      try {
        CaptureSetting capture_setting;
        capture_setting.brightness.set(request->brightness);
        capture_setting.contrast.set(request->contrast);
        capture_setting.saturation.set(request->saturation);
        capture_setting.temperature.set(request->temperature);
        capture_setting.exposure.set(request->exposure);
        capture_setting.gain.set(request->gain);

        camera_node->configure_capture_setting(capture_setting);

        if (request->save) {
          nlohmann::json config;

          config["brightness"] = request->brightness;
          config["contrast"] = request->contrast;
          config["saturation"] = request->saturation;
          config["temperature"] = request->temperature;
          config["exposure"] = request->exposure;
          config["gain"] = request->gain;

          jitsuyo::save_config(path, "capture_settings.json", config);
        }

        response->success = true;
      } catch (const std::exception & e) {
        RCLCPP_ERROR(node->get_logger(), "Exception occurred: %s", e.what());
        response->success = false;
      } catch (...) {
        RCLCPP_ERROR(node->get_logger(), "Unknown exception occurred");
        response->success = false;
      }
    }
  );
}

std::string ConfigNode::get_node_prefix() const
{
  return "shisen_cpp/config";
}

}  // namespace shisen_cpp::camera
