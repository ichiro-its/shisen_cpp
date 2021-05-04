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

#ifndef SHISEN_CPP__CONSUMER__COMPRESSED_IMAGE_CONSUMER_HPP_
#define SHISEN_CPP__CONSUMER__COMPRESSED_IMAGE_CONSUMER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

#include "../interface.hpp"

namespace shisen_cpp
{

class CompressedImageConsumer
{
public:
  using CompressedImageCallback = std::function<void (const CompressedImage &)>;

  inline CompressedImageConsumer();

  inline explicit CompressedImageConsumer(
    rclcpp::Node::SharedPtr node, const std::string & prefix = CAMERA_PREFIX);

  inline void set_node(
    rclcpp::Node::SharedPtr node, const std::string & prefix = CAMERA_PREFIX);

  inline void set_on_compressed_image_changed(const CompressedImageCallback & callback);

  inline rclcpp::Node::SharedPtr get_node() const;

  inline const CompressedImage & get_compressed_image() const;

private:
  inline void change_compressed_image(const CompressedImage & compressed_image);

  rclcpp::Node::SharedPtr node;

  rclcpp::Subscription<CompressedImage>::SharedPtr compressed_image_subscription;

  CompressedImageCallback on_compressed_image_changed;

  CompressedImage current_compressed_image;
};

CompressedImageConsumer::CompressedImageConsumer()
{
}

CompressedImageConsumer::CompressedImageConsumer(
  rclcpp::Node::SharedPtr node, const std::string & prefix)
{
  set_node(node, prefix);
}

void CompressedImageConsumer::set_node(rclcpp::Node::SharedPtr node, const std::string & prefix)
{
  // Initialize the node
  this->node = node;

  // Initialize the compressed image subscription
  {
    compressed_image_subscription = get_node()->create_subscription<CompressedImage>(
      prefix + COMPRESSED_IMAGE_SUFFIX, 10,
      [this](const CompressedImage::SharedPtr msg) {
        change_compressed_image(*msg);
      });

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Compressed image subscription initialized on `" <<
        compressed_image_subscription->get_topic_name() << "`!");
  }
}

void CompressedImageConsumer::set_on_compressed_image_changed(
  const CompressedImageCallback & callback)
{
  on_compressed_image_changed = callback;
}

rclcpp::Node::SharedPtr CompressedImageConsumer::get_node() const
{
  return node;
}

const CompressedImage & CompressedImageConsumer::get_compressed_image() const
{
  return current_compressed_image;
}

void CompressedImageConsumer::change_compressed_image(const CompressedImage & compressed_image)
{
  current_compressed_image = compressed_image;
  if (on_compressed_image_changed) {
    on_compressed_image_changed(get_compressed_image());
  }
}

}  // namespace shisen_cpp

#endif  // SHISEN_CPP__CONSUMER__COMPRESSED_IMAGE_CONSUMER_HPP_
