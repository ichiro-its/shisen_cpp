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

#ifndef SHISEN_CPP__CAMERA__PROVIDER__IMAGE_PROVIDER_HPP_
#define SHISEN_CPP__CAMERA__PROVIDER__IMAGE_PROVIDER_HPP_

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <shisen_cpp/utility.hpp>
#include <std_msgs/msg/header.hpp>

#include <memory>
#include <string>

namespace shisen_cpp::camera
{

class ImageProvider
{
public:
  using Image = sensor_msgs::msg::Image;

  explicit ImageProvider(const Options & options = Options());
  ~ImageProvider();

  void set_image(const Image & image);
  void set_mat(cv::Mat mat);
  void update_mat();

  const Image & get_image() const;
  cv::Mat get_mat() const;

  std::shared_ptr<cv::VideoCapture> get_video_capture() const;

  Options options;

private:
  Image current_image_msg;
  cv::Mat current_mat_image;

  int compression_quality;

  std::shared_ptr<cv::VideoCapture> video_capture;
};

}  // namespace shisen_cpp::camera

#endif  // SHISEN_CPP__CAMERA__PROVIDER__IMAGE_PROVIDER_HPP_
