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

#ifndef SHISEN_CPP__CONSUMER__IMAGE_CONSUMER_HPP_
#define SHISEN_CPP__CONSUMER__IMAGE_CONSUMER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

#include "../utility.hpp"

namespace shisen_cpp
{

class ImageConsumer
{
public:
  using ImageCallback = std::function<void (const Image &)>;

  struct Options : public virtual CameraPrefixOptions
  {
  };

  inline explicit ImageConsumer(
    rclcpp::Node::SharedPtr node, const Options & options = Options());

  inline virtual void on_image_changed(const Image & image);

  inline rclcpp::Node::SharedPtr get_node() const;

  inline const Image & get_image() const;

private:
  rclcpp::Node::SharedPtr node;

  rclcpp::Subscription<Image>::SharedPtr image_subscription;

  Image current_image;
};

ImageConsumer::ImageConsumer(
  rclcpp::Node::SharedPtr node, const ImageConsumer::Options & options)
{
  // Initialize the node
  this->node = node;

  // Initialize the image subscription
  {
    image_subscription = get_node()->template create_subscription<Image>(
      options.camera_prefix + IMAGE_SUFFIX, 10,
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

void ImageConsumer::on_image_changed(const Image & /*image*/)
{
}

rclcpp::Node::SharedPtr ImageConsumer::get_node() const
{
  return node;
}

const Image & ImageConsumer::get_image() const
{
  return current_image;
}

}  // namespace shisen_cpp

#endif  // SHISEN_CPP__CONSUMER__IMAGE_CONSUMER_HPP_
