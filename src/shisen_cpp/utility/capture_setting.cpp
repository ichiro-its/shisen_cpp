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
    brightness = msg.brightness.front();
  }

  if (msg.contrast.size() > 0) {
    contrast = msg.contrast.front();
  }

  if (msg.saturation.size() > 0) {
    saturation = msg.saturation.front();
  }

  if (msg.temperature.size() > 0) {
    temperature = msg.temperature.front();
  }

  if (msg.exposure.size() > 0) {
    exposure = msg.exposure.front();
  }

  if (msg.gain.size() > 0) {
    gain = msg.gain.front();
  }

  return *this;
}

CaptureSetting::operator CaptureSettingMsg() const
{
  CaptureSettingMsg msg;

  if (brightness.is_not_empty()) {
    msg.brightness.push_back(brightness);
  }

  if (contrast.is_not_empty()) {
    msg.contrast.push_back(contrast);
  }

  if (saturation.is_not_empty()) {
    msg.saturation.push_back(saturation);
  }

  if (temperature.is_not_empty()) {
    msg.temperature.push_back(temperature);
  }

  if (exposure.is_not_empty()) {
    msg.exposure.push_back(exposure);
  }

  if (gain.is_not_empty()) {
    msg.gain.push_back(gain);
  }

  return msg;
}

void CaptureSetting::update_with(const CaptureSetting & capture_setting)
{
  if (capture_setting.brightness.is_not_empty()) {
    brightness = capture_setting.brightness;
  }

  if (capture_setting.contrast.is_not_empty()) {
    contrast = capture_setting.contrast;
  }

  if (capture_setting.saturation.is_not_empty()) {
    saturation = capture_setting.saturation;
  }

  if (capture_setting.temperature.is_not_empty()) {
    temperature = capture_setting.temperature;
  }

  if (capture_setting.exposure.is_not_empty()) {
    exposure = capture_setting.exposure;
  }

  if (capture_setting.gain.is_not_empty()) {
    gain = capture_setting.gain;
  }
}

}  // namespace shisen_cpp
