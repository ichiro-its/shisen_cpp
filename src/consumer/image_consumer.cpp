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

#include <shisen_cpp/consumer/image_consumer.hpp>

namespace shisen_cpp
{

ImageConsumer::ImageConsumer(
  rclcpp::Node::SharedPtr node, const ImageConsumer::Options & options)
: CameraNode(node, options)
{
  // Initialize the image subscription
  {
    image_subscription = get_node()->template create_subscription<Image>(
      get_camera_prefix() + IMAGE_SUFFIX, 10,
      [this](const Image::SharedPtr msg) {
        current_image = *msg;

        // Call callback after image changed
        on_image_changed(get_image());
      });

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Image subscription initialized on `" << image_subscription->get_topic_name() << "`!");
  }
}

ImageConsumer::~ImageConsumer()
{
}

void ImageConsumer::on_image_changed(const shisen_cpp::Image & image)
{
  current_mat_image = image;

  // Call virtual callback
  on_mat_changed(get_mat());
}

void ImageConsumer::on_mat_changed(cv::Mat /*mat*/)
{
}

const Image & ImageConsumer::get_image() const
{
  return current_image;
}

cv::Mat ImageConsumer::get_mat() const
{
  return (cv::Mat)current_mat_image;
}

}  // namespace shisen_cpp
