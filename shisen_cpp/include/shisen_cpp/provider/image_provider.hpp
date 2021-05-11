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

#ifndef SHISEN_CPP__PROVIDER__IMAGE_PROVIDER_HPP_
#define SHISEN_CPP__PROVIDER__IMAGE_PROVIDER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

#include "../node.hpp"

namespace shisen_cpp
{

class ImageProvider : public CameraNode
{
public:
  struct Options : public virtual CameraNode::Options
  {
  };

  inline explicit ImageProvider(
    rclcpp::Node::SharedPtr node, const Options & options = Options());

  inline void set_image(const Image & image);

  inline const Image & get_image() const;

private:
  typename rclcpp::Publisher<Image>::SharedPtr image_publisher;

  Image current_image;
};

ImageProvider::ImageProvider(
  rclcpp::Node::SharedPtr node, const ImageProvider::Options & options)
: CameraNode(node, options)
{
  // Initialize the image publisher
  {
    image_publisher = get_node()->template create_publisher<Image>(
      get_camera_prefix() + IMAGE_SUFFIX, 10);

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Image publisher initialized on `" << image_publisher->get_topic_name() << "`!");
  }

  // Initial data publish
  set_image(get_image());
}

void ImageProvider::set_image(const Image & image)
{
  current_image = image;

  // Publish changes
  image_publisher->publish(get_image());
}

const Image & ImageProvider::get_image() const
{
  return current_image;
}

}  // namespace shisen_cpp

#endif  // SHISEN_CPP__PROVIDER__IMAGE_PROVIDER_HPP_
