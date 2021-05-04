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

#include <shisen_cpp/utility/capture_setting.hpp>

namespace shisen_cpp
{

CaptureSetting::CaptureSetting()
{
}

CaptureSetting::CaptureSetting(const CaptureSettingMsg & msg)
{
  *this = msg;
}

const CaptureSetting & CaptureSetting::operator=(const CaptureSettingMsg & msg)
{
  if (msg.brightness.size() > 0) {
    brightness.set(msg.brightness.front());
  }

  if (msg.contrast.size() > 0) {
    contrast.set(msg.contrast.front());
  }

  if (msg.saturation.size() > 0) {
    saturation.set(msg.saturation.front());
  }

  if (msg.temperature.size() > 0) {
    temperature.set(msg.temperature.front());
  }

  if (msg.hue.size() > 0) {
    hue.set(msg.hue.front());
  }

  if (msg.gain.size() > 0) {
    gain.set(msg.gain.front());
  }

  return *this;
}

CaptureSetting::operator CaptureSettingMsg() const
{
  CaptureSettingMsg msg;

  if (brightness.is_not_empty()) {
    msg.brightness.push_back(brightness.get());
  }

  if (contrast.is_not_empty()) {
    msg.contrast.push_back(contrast.get());
  }

  if (saturation.is_not_empty()) {
    msg.saturation.push_back(saturation.get());
  }

  if (temperature.is_not_empty()) {
    msg.temperature.push_back(temperature.get());
  }

  if (hue.is_not_empty()) {
    msg.hue.push_back(hue.get());
  }

  if (gain.is_not_empty()) {
    msg.gain.push_back(gain.get());
  }

  return msg;
}

}  // namespace shisen_cpp
