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


#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <shisen_cpp/camera/node/camera_node.hpp>
#include <jitsuyo/config.hpp>

#include <chrono>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>

namespace shisen_cpp::camera
{

CameraNode::CameraNode(rclcpp::Node::SharedPtr node, const Options & options)
: node(node), options(options)
{
}

CameraNode::~CameraNode()
{
}

void CameraNode::update()
{
  image_provider->update_mat();

  // Get captured mat
  cv::Mat captured_mat = image_provider->get_mat();

  // Ensure the captured mat is not empty
  if (!captured_mat.empty()) {
    on_mat_captured(captured_mat);
    on_camera_config(captured_mat.cols, captured_mat.rows);
  } else {
    RCLCPP_WARN_ONCE(node->get_logger(), "Once, captured an empty mat!");
  }
}

void CameraNode::on_mat_captured(cv::Mat mat)
{
  image_publisher->publish(image_provider->get_image());
}

void CameraNode::on_camera_config(int width, int height)
{
  camera_config_provider->set_config(width, height);
  camera_config_publisher->publish(camera_config_provider->get_camera_config());
}

void CameraNode::save_image(cv::Mat mat)
{
  if (!std::filesystem::exists("image")) {
    if (!std::filesystem::create_directory("image")) {
      RCLCPP_ERROR(node->get_logger(), "Error creating `image` directory!");
      return;
    }
  }

  auto now = std::chrono::system_clock::now();
  std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);
  auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

  std::stringstream ss;
  ss << std::put_time(std::localtime(&now_time_t), "%Y-%m-%d_%H-%M-%S");
  ss << '-' << std::setfill('0') << std::setw(3) << now_ms.count();
  std::string timestamp = ss.str();

  std::string filename = "image/" + timestamp + ".jpg";

  bool result = cv::imwrite(filename, mat);
  if (!result) {
    RCLCPP_ERROR(node->get_logger(), "Failed to save image!");
  }
}

cv::Mat CameraNode::get_mat()
{
  update();
  return image_provider->get_mat();
}

const std::string & CameraNode::get_camera_prefix() const
{
  return options.camera_prefix;
}

void CameraNode::set_provider(
  std::shared_ptr<ImageProvider> img_provider,
  std::shared_ptr<CameraConfigProvider> cam_config_provider)
{
  image_provider = img_provider;
  camera_config_provider = cam_config_provider;

  // Initialize the image publisher
  {
    image_publisher = node->create_publisher<Image>(
      get_camera_prefix() + IMAGE_SUFFIX, 10);

    RCLCPP_INFO_STREAM(
      node->get_logger(),
      "Image publisher initialized on `" << image_publisher->get_topic_name() << "`!");
  }
  // Logging for opening camera
  RCLCPP_INFO_STREAM(
    node->get_logger(),
    "Camera capture opened on `" << image_provider->options.camera_file_name << "`!");

  // Initialize the camera config publisher
  {
    camera_config_publisher = node->create_publisher<CameraConfig>(
      get_camera_prefix() + CAMERA_CONFIG_SUFFIX, 10);

    RCLCPP_INFO_STREAM(
      node->get_logger(),
      "Camera Config publisher initialized on `" << camera_config_publisher->get_topic_name() <<
        "`!");
  }

  // Initialize the capture setting event publisher
  {
    capture_setting_event_publisher = node->create_publisher<CaptureSettingMsg>(
      get_camera_prefix() + CAPTURE_SETTING_EVENT_SUFFIX, 10);

    RCLCPP_INFO_STREAM(
      node->get_logger(),
      "Capture setting event publisher initialized on `" <<
        capture_setting_event_publisher->get_topic_name() << "`!");
  }

  // Initialize the configure capture setting service
  {
    configure_capture_setting_service = node->create_service<ConfigureCaptureSetting>(
      get_camera_prefix() + CONFIGURE_CAPTURE_SETTING_SUFFIX,
      [this](ConfigureCaptureSetting::Request::SharedPtr request,
        ConfigureCaptureSetting::Response::SharedPtr response) {
        // Configure capture setting if exist
        if (request->capture_setting.size() > 0) {
          configure_capture_setting(((CaptureSetting)request->capture_setting.front()));
        }

        response->capture_setting.push_back(current_capture_setting);
      });

    RCLCPP_INFO_STREAM(
      node->get_logger(),
      "Configure capture setting service initialized on `" <<
        configure_capture_setting_service->get_service_name() << "`!");
  }
}

CaptureSetting CameraNode::on_configure_capture_setting(
  const CaptureSetting & capture_setting)
{
  auto new_capture_setting = capture_setting;

  auto video_capture = image_provider->get_video_capture();

  // Set the capture setting to the camera
  {
    if (new_capture_setting.brightness.is_not_empty()) {
      video_capture->set(cv::CAP_PROP_BRIGHTNESS, new_capture_setting.brightness);
    }

    if (new_capture_setting.contrast.is_not_empty()) {
      video_capture->set(cv::CAP_PROP_CONTRAST, new_capture_setting.contrast);
    }

    if (new_capture_setting.saturation.is_not_empty()) {
      video_capture->set(cv::CAP_PROP_SATURATION, new_capture_setting.saturation);
    }

    if (new_capture_setting.temperature.is_not_empty()) {
      video_capture->set(cv::CAP_PROP_WB_TEMPERATURE, new_capture_setting.temperature);
    }

    if (new_capture_setting.exposure.is_not_empty()) {
      video_capture->set(cv::CAP_PROP_EXPOSURE, new_capture_setting.exposure);
    }

    if (new_capture_setting.gain.is_not_empty()) {
      video_capture->set(cv::CAP_PROP_GAIN, new_capture_setting.gain);
    }
  }

  // Get new capture setting from the camera
  new_capture_setting.brightness = video_capture->get(cv::CAP_PROP_BRIGHTNESS);
  new_capture_setting.contrast = video_capture->get(cv::CAP_PROP_CONTRAST);
  new_capture_setting.saturation = video_capture->get(cv::CAP_PROP_SATURATION);
  new_capture_setting.temperature = video_capture->get(cv::CAP_PROP_WB_TEMPERATURE);
  new_capture_setting.exposure = video_capture->get(cv::CAP_PROP_EXPOSURE);
  new_capture_setting.gain = video_capture->get(cv::CAP_PROP_GAIN);

  return new_capture_setting;
}

void CameraNode::configure_capture_setting(const CaptureSetting & capture_setting)
{
  // Update with configured data
  current_capture_setting.update_with(on_configure_capture_setting(capture_setting));
  capture_setting_event_publisher->publish(static_cast<CaptureSettingMsg>(current_capture_setting));
}

void CameraNode::load_configuration(const std::string & path)
{
  nlohmann::json config;
  if (!jitsuyo::load_config(path, "capture_settings.json", config)) {
    throw std::runtime_error("Unable to open `" + path + "capture_settings.json`!");
  }

  CaptureSetting capture_setting;

  int setting_brightness;
  int setting_contrast;
  int setting_saturation;
  int setting_temperature;
  int setting_exposure;
  int setting_gain;
  int setting_width;
  int setting_height;

  if (!jitsuyo::assign_val(config, "brightness", setting_brightness) ||
    !jitsuyo::assign_val(config, "contrast", setting_contrast) ||
    !jitsuyo::assign_val(config, "saturation", setting_saturation) ||
    !jitsuyo::assign_val(config, "temperature", setting_temperature) ||
    !jitsuyo::assign_val(config, "exposure", setting_exposure) ||
    !jitsuyo::assign_val(config, "gain", setting_gain) ||
    !jitsuyo::assign_val(config, "width", setting_width) ||
    !jitsuyo::assign_val(config, "height", setting_height))
  {
    std::cout << "Error found at section `capture_settings`" << std::endl;
    throw std::runtime_error("Failed to load config file `" + path + "capture_settings.json`");
  }

  capture_setting.brightness.set(setting_brightness);
  capture_setting.contrast.set(setting_contrast);
  capture_setting.saturation.set(setting_saturation);
  capture_setting.temperature.set(setting_temperature);
  capture_setting.exposure.set(setting_exposure);
  capture_setting.gain.set(setting_gain);

  configure_capture_setting(capture_setting);

  options.width = setting_width;
  options.height = setting_height;

  image_provider->initialize_video_capture(options);
}

}  // namespace shisen_cpp::camera
