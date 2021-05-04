// Copyright (c) 2021 Alfi Maulana
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

#ifndef SHISEN_CPP__PROVIDER__COMPRESSED_IMAGE_PROVIDER_HPP_
#define SHISEN_CPP__PROVIDER__COMPRESSED_IMAGE_PROVIDER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

#include "../interface.hpp"

namespace shisen_cpp
{

class CompressedImageProvider
{
public:
  inline CompressedImageProvider();

  inline explicit CompressedImageProvider(
    rclcpp::Node::SharedPtr node, const std::string & prefix = CAMERA_PREFIX);

  inline void set_node(
    rclcpp::Node::SharedPtr node, const std::string & prefix = CAMERA_PREFIX);

  inline void set_compressed_image(const CompressedImage & compressed_image);

  inline rclcpp::Node::SharedPtr get_node() const;

  inline const CompressedImage & get_compressed_image() const;

private:
  rclcpp::Node::SharedPtr node;

  rclcpp::Publisher<CompressedImage>::SharedPtr compressed_image_publisher;

  CompressedImage current_compressed_image;
};

CompressedImageProvider::CompressedImageProvider()
{
}

CompressedImageProvider::CompressedImageProvider(
  rclcpp::Node::SharedPtr node, const std::string & prefix)
{
  set_node(node, prefix);
}

void CompressedImageProvider::set_node(rclcpp::Node::SharedPtr node, const std::string & prefix)
{
  // Initialize the node
  this->node = node;

  // Initialize the compressed image publisher
  {
    compressed_image_publisher = get_node()->create_publisher<CompressedImage>(
      prefix + COMPRESSED_IMAGE_SUFFIX, 10);

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Compressed image publisher initialized on `" <<
        compressed_image_publisher->get_topic_name() << "`!");
  }

  // Initial data publish
  set_compressed_image(get_compressed_image());
}

void CompressedImageProvider::set_compressed_image(const CompressedImage & compressed_image)
{
  current_compressed_image = compressed_image;
  compressed_image_publisher->publish(get_compressed_image());
}

rclcpp::Node::SharedPtr CompressedImageProvider::get_node() const
{
  return node;
}

const CompressedImage & CompressedImageProvider::get_compressed_image() const
{
  return current_compressed_image;
}

}  // namespace shisen_cpp

#endif  // SHISEN_CPP__PROVIDER__COMPRESSED_IMAGE_PROVIDER_HPP_
