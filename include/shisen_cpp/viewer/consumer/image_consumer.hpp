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

#ifndef SHISEN_CPP__VIEWER__CONSUMER__IMAGE_CONSUMER_HPP_
#define SHISEN_CPP__VIEWER__CONSUMER__IMAGE_CONSUMER_HPP_

#include <shisen_interfaces/msg/image.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <memory>
#include <string>

#include "shisen_cpp/utility.hpp"


namespace shisen_cpp::viewer
{

class ImageConsumer
{
public:
  using Image = shisen_interfaces::msg::Image;

  virtual void on_image_changed(const Image & image);

  const Image & get_image() const;
  cv::Mat get_mat() const;

private:
  Image current_image;
  MatImage current_mat_image;
};

}  // namespace shisen_cpp::viewer

#endif  // SHISEN_CPP__VIEWER__CONSUMER__IMAGE_CONSUMER_HPP_
