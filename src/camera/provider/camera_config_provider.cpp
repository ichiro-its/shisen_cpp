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

#include <shisen_cpp/camera/provider/camera_config_provider.hpp>

namespace shisen_cpp::camera
{

CameraConfigProvider::CameraConfigProvider(const Options & options)
{
  // Initial data publish
  field_of_view = options.field_of_view;
}

CameraConfigProvider::~CameraConfigProvider()
{
}

void CameraConfigProvider::set_config(int width, int height)
{
  CameraConfig config;

  float diagonal = pow(width * width + height * height, 0.5);
  float depth = (diagonal / 2) / keisan::make_degree(field_of_view / 2).tan();

  float view_v_angle =
    2 * keisan::signed_arctan(static_cast<float>(height / 2), depth).degree();
  float view_h_angle =
    2 * keisan::signed_arctan(static_cast<float>(width / 2), depth).degree();

  config.width = width;
  config.height = height;
  config.v_angle = view_v_angle;
  config.h_angle = view_h_angle;

  camera_config = config;
}

const CameraConfig & CameraConfigProvider::get_camera_config() const
{
  return camera_config;
}

}  // namespace shisen_cpp::camera
