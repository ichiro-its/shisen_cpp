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

#include <memory>
#include <opencv2/opencv.hpp>

namespace shisen_cpp::camera
{
using Image = sensor_msgs::msg::Image;

ImageProvider::ImageProvider(const Options & options)
: options(options), compression_quality(options.compression_quality),
  video_capture(std::make_shared<cv::VideoCapture>())
{
  // Try to open the camera
  if (!video_capture->open(options.camera_file_name)) {
    throw std::runtime_error("unable to open camera on `" + options.camera_file_name + "`");
  }

  video_capture->set(cv::CAP_PROP_FRAME_WIDTH, options.width);
  video_capture->set(cv::CAP_PROP_FRAME_HEIGHT, options.height);
}

ImageProvider::~ImageProvider()
{
}

void ImageProvider::set_image(const Image & image)
{
  current_image_msg = image;
}

void ImageProvider::update_mat()
{
  // Ensure the camera is opened
  if (!video_capture->isOpened()) {
    throw std::runtime_error("Once, camera capture had not been opened!");
    return;
  }

  // Read captured mat
  cv::Mat captured_mat;
  video_capture->read(captured_mat);

  // Ensure the captured mat is not empty
  if (!captured_mat.empty()) {
    set_mat(captured_mat);
  } else {
    throw std::runtime_error("Once, captured an empty mat!");
  }
}

void ImageProvider::set_mat(cv::Mat mat)
{
  current_mat_image = mat;
  const auto ptr = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", mat);
  set_image(*(ptr.toImageMsg()));
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

}  // namespace shisen_cpp::camera
