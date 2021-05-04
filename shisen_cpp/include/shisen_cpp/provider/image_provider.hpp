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

#include "../utility.hpp"

namespace shisen_cpp
{

template<typename T>
class ImageProvider;

using CompressedImageProvider = ImageProvider<CompressedImage>;
using RawImageProvider = ImageProvider<RawImage>;

template<typename T>
class ImageProvider
{
public:
  inline ImageProvider();

  inline explicit ImageProvider(
    rclcpp::Node::SharedPtr node, const std::string & prefix = CAMERA_PREFIX);

  inline void set_node(
    rclcpp::Node::SharedPtr node, const std::string & prefix = CAMERA_PREFIX);

  inline void set_image(const T & image);

  inline rclcpp::Node::SharedPtr get_node() const;

  inline const T & get_image() const;

private:
  rclcpp::Node::SharedPtr node;

  typename rclcpp::Publisher<T>::SharedPtr image_publisher;

  T current_image;
};

template<typename T>
ImageProvider<T>::ImageProvider()
{
}

template<typename T>
ImageProvider<T>::ImageProvider(
  rclcpp::Node::SharedPtr node, const std::string & prefix)
{
  set_node(node, prefix);
}

template<typename T>
void ImageProvider<T>::set_node(rclcpp::Node::SharedPtr node, const std::string & prefix)
{
  // Initialize the node
  this->node = node;

  // Initialize the image publisher
  {
    std::string image_suffix = IMAGE_SUFFIX;
    if (std::is_same<T, CompressedImage>::value) {
      image_suffix = COMPRESSED_IMAGE_SUFFIX;
    } else if (std::is_same<T, RawImage>::value) {
      image_suffix = RAW_IMAGE_SUFFIX;
    }

    image_publisher = get_node()->template create_publisher<T>(
      prefix + image_suffix, 10);

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Image publisher initialized on `" << image_publisher->get_topic_name() << "`!");
  }

  // Initial data publish
  set_image(get_image());
}

template<typename T>
void ImageProvider<T>::set_image(const T & image)
{
  current_image = image;
  image_publisher->publish(get_image());
}

template<typename T>
rclcpp::Node::SharedPtr ImageProvider<T>::get_node() const
{
  return node;
}

template<typename T>
const T & ImageProvider<T>::get_image() const
{
  return current_image;
}

}  // namespace shisen_cpp

#endif  // SHISEN_CPP__PROVIDER__IMAGE_PROVIDER_HPP_
