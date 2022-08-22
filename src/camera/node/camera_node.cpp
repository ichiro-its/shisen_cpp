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

#include <shisen_cpp/camera/node/camera_node.hpp>

namespace shisen_cpp
{

using namespace std::chrono_literals;

CameraNode::CameraNode(
  rclcpp::Node::SharedPtr node, std::shared_ptr<ImageProvider> img_provider)
: image_provider(img_provider), BaseNode(node, image_provider->options)
{
  // Initialize the image publisher
  {
    image_publisher = get_node()->create_publisher<Image>(
      get_camera_prefix() + IMAGE_SUFFIX, 10);

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Image publisher initialized on `" << image_publisher->get_topic_name() << "`!");
  }

  // Try to open the camera
  if (!image_provider->get_video_capture()->open(image_provider->options.camera_file_name)) {
    throw std::runtime_error("unable to open camera on `" + image_provider->options.camera_file_name + "`");
  }

  RCLCPP_INFO_STREAM(
    get_node()->get_logger(),
    "Camera capture opened on `" << image_provider->options.camera_file_name << "`!");
  
    // Initialize the capture timer
  node_timer = get_node()->create_wall_timer(
    1s / image_provider->options.capture_fps, [this]() {
      update();
    });
}

void CameraNode::update()
{
  // Ensure the camera is opened
  if (!image_provider->get_video_capture()->isOpened()) {
    RCLCPP_WARN_ONCE(get_node()->get_logger(), "Once, camera capture had not been opened!");
    return;
  }

  // Read captured mat
  cv::Mat captured_mat;
  image_provider->get_video_capture()->read(captured_mat);

  shisen_interfaces::msg::CameraConfig config;

  // Ensure the captured mat is not empty
  if (!captured_mat.empty()) {
    image_provider->set_mat(captured_mat);
    // on_mat_captured(captured_mat);
    // on_camera_config(config, captured_mat.cols, captured_mat.rows);
  } else {
    RCLCPP_WARN_ONCE(get_node()->get_logger(), "Once, captured an empty mat!");
  }
}

}  // namespace shisen_cpp
