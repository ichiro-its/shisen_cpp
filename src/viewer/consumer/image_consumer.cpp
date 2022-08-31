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

#include <shisen_cpp/viewer/consumer/image_consumer.hpp>

namespace shisen_cpp
{

ImageConsumer::ImageConsumer()
{
}

ImageConsumer::~ImageConsumer()
{
}

void ImageConsumer::on_image_changed(const shisen_cpp::Image & image)
{
  current_mat_image = image;
  if (!get_mat().empty()) {
    cv::imshow("viewer", get_mat());
    cv::waitKey(1);
  } else {
    throw std::runtime_error("Once, received an empty mat!");
  }
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
