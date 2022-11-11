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

#ifndef SHISEN_CPP__CAMERA__PROVIDER__CAMERA_CONFIG_PROVIDER_HPP_
#define SHISEN_CPP__CAMERA__PROVIDER__CAMERA_CONFIG_PROVIDER_HPP_

#include <keisan/keisan.hpp>

#include <memory>
#include <string>

#include "shisen_cpp/utility/options.hpp"
#include "shisen_interfaces/msg/camera_config.hpp"

namespace shisen_cpp::camera
{

class CameraConfigProvider
{
public:
  using CameraConfig = shisen_interfaces::msg::CameraConfig;

  explicit CameraConfigProvider(const Options & options = Options());
  ~CameraConfigProvider();

  void set_config(int width, int height);

  const CameraConfig & get_camera_config() const;

private:
  CameraConfig camera_config;

  int field_of_view;
};

}  // namespace shisen_cpp::camera

#endif  // SHISEN_CPP__CAMERA__PROVIDER__CAMERA_CONFIG_PROVIDER_HPP_
