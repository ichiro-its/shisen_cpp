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

#include <shisen_cpp/camera/provider/image_provider.hpp>

namespace shisen_cpp
{
using Image = shisen_interfaces::msg::Image;

ImageProvider::ImageProvider(const ImageProvider::Options & options)
: options(options), compression_quality(options.compression_quality),
  video_capture(std::make_shared<cv::VideoCapture>())
{
}

ImageProvider::~ImageProvider()
{
}

void ImageProvider::set_image(const Image & image)
{
  current_image_msg = image;
}

void ImageProvider::set_mat(cv::Mat mat)
{
  current_mat_image = mat;

  // Set image according to the compression quality
  if (options.publish_image) {
    if (compression_quality > 0) {
      set_image(current_mat_image.compress(compression_quality));
    } else {
      set_image(current_mat_image);
    }
  }
}

const Image & ImageProvider::get_image() const
{
  return current_image_msg;
}

cv::Mat ImageProvider::get_mat() const
{
  return (cv::Mat)current_mat_image;
}

std::shared_ptr<cv::VideoCapture> ImageProvider::get_video_capture() const
{
  return video_capture;
}

}  // namespace shisen_cpp
