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

CameraNode::CameraNode(rclcpp::Node::SharedPtr node, const Options & options)
: node(node), options(options)
{
}

CameraNode::~CameraNode()
{
}

void CameraNode::update()
{
  // Ensure the camera is opened
  if (!image_provider->get_video_capture()->isOpened()) {
    RCLCPP_WARN_ONCE(node->get_logger(), "Once, camera capture had not been opened!");
    return;
  }

  // Read captured mat
  cv::Mat captured_mat;
  image_provider->get_video_capture()->read(captured_mat);

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
  image_provider->set_mat(mat);

  if (image_provider->options.publish_image) {
    image_publisher->publish(image_provider->get_image());
  }
}

void CameraNode::on_camera_config(int width, int height)
{
  camera_config_provider->set_config(width, height);
  camera_config_publisher->publish(camera_config_provider->get_camera_config());
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
  if (image_provider->options.publish_image) {
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
}

}  // namespace shisen_cpp
