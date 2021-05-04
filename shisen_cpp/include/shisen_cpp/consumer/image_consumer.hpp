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

#include "../interface.hpp"

namespace shisen_cpp
{

template<typename T>
class ImageConsumer;

using CompressedImageConsumer = ImageConsumer<CompressedImage>;
using RawImageConsumer = ImageConsumer<RawImage>;

template<typename T>
class ImageConsumer
{
public:
  using ImageCallback = std::function<void (const T &)>;

  inline ImageConsumer();

  inline explicit ImageConsumer(
    rclcpp::Node::SharedPtr node, const std::string & prefix = CAMERA_PREFIX);

  inline void set_node(
    rclcpp::Node::SharedPtr node, const std::string & prefix = CAMERA_PREFIX);

  inline void set_on_image_changed(const ImageCallback & callback);

  inline rclcpp::Node::SharedPtr get_node() const;

  inline const T & get_image() const;

private:
  inline void change_image(const T & image);

  rclcpp::Node::SharedPtr node;

  typename rclcpp::Subscription<T>::SharedPtr image_subscription;

  ImageCallback on_image_changed;

  T current_image;
};

template<typename T>
ImageConsumer<T>::ImageConsumer()
{
}

template<typename T>
ImageConsumer<T>::ImageConsumer(
  rclcpp::Node::SharedPtr node, const std::string & prefix)
{
  set_node(node, prefix);
}

template<typename T>
void ImageConsumer<T>::set_node(rclcpp::Node::SharedPtr node, const std::string & prefix)
{
  // Initialize the node
  this->node = node;

  // Initialize the image subscription
  {
    std::string image_suffix = IMAGE_SUFFIX;
    if (std::is_same<T, CompressedImage>::value) {
      image_suffix = COMPRESSED_IMAGE_SUFFIX;
    } else if (std::is_same<T, RawImage>::value) {
      image_suffix = RAW_IMAGE_SUFFIX;
    }

    image_subscription = get_node()->template create_subscription<T>(
      prefix + image_suffix, 10,
      [this](const typename T::SharedPtr msg) {
        change_image(*msg);
      });

    RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Image subscription initialized on `" << image_subscription->get_topic_name() << "`!");
  }
}

template<typename T>
void ImageConsumer<T>::set_on_image_changed(const ImageConsumer<T>::ImageCallback & callback)
{
  on_image_changed = callback;
}

template<typename T>
rclcpp::Node::SharedPtr ImageConsumer<T>::get_node() const
{
  return node;
}

template<typename T>
const T & ImageConsumer<T>::get_image() const
{
  return current_image;
}

template<typename T>
void ImageConsumer<T>::change_image(const T & image)
{
  current_image = image;
  if (on_image_changed) {
    on_image_changed(get_image());
  }
}

}  // namespace shisen_cpp

#endif  // SHISEN_CPP__CONSUMER__IMAGE_CONSUMER_HPP_
