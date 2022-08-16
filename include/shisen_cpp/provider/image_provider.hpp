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
#include "../utility/mat_image.hpp"

namespace shisen_cpp
{

class ImageProvider : public CameraNode
{
public:
  struct Options : public virtual CameraNode::Options
  {
    int compression_quality;

    Options()
    : compression_quality(-1)
    {
    }
  };

  explicit ImageProvider(
    rclcpp::Node::SharedPtr node, const Options & options = Options());
  ~ImageProvider();

  void set_image(const Image & image);
  void set_mat(cv::Mat mat);

  const Image & get_image() const;
  cv::Mat get_mat() const;

private:
  typename rclcpp::Publisher<Image>::SharedPtr image_publisher;

  Image current_image;
  shisen_cpp::MatImage current_mat_image;

  int compression_quality;
};

}  // namespace shisen_cpp

#endif  // SHISEN_CPP__PROVIDER__IMAGE_PROVIDER_HPP_
