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

#ifndef SHISEN_CPP__UTILITY__CAPTURE_SETTING_HPP_
#define SHISEN_CPP__UTILITY__CAPTURE_SETTING_HPP_

#include <shisen_interfaces/msg/capture_setting.hpp>

#include "./emptiable.hpp"

namespace shisen_cpp
{

using CaptureSettingMsg = shisen_interfaces::msg::CaptureSetting;

struct CaptureSetting
{
  CaptureSetting();
  explicit CaptureSetting(const CaptureSettingMsg & msg);

  operator CaptureSettingMsg() const;

  const CaptureSetting & operator=(const CaptureSettingMsg & msg);

  void update_with(const CaptureSetting & capture_setting);

  Emptiable<int> brightness;
  Emptiable<int> contrast;
  Emptiable<int> saturation;
  Emptiable<int> temperature;
  Emptiable<int> exposure;
  Emptiable<int> gain;
};

}  // namespace shisen_cpp

#endif  // SHISEN_CPP__UTILITY__CAPTURE_SETTING_HPP_
