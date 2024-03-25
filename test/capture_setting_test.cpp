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

#include <gtest/gtest.h>
#include <shisen_cpp/shisen_cpp.hpp>

TEST(CaptureSettingTest, FromMsg) {
  shisen_cpp::CaptureSettingMsg msg;

  msg.brightness.push_back(10);
  msg.contrast.push_back(20);
  msg.temperature.push_back(30);
  msg.exposure.push_back(40);

  shisen_cpp::CaptureSetting capture_setting(msg);

  ASSERT_TRUE(capture_setting.brightness.is_not_empty());
  ASSERT_TRUE(capture_setting.contrast.is_not_empty());
  ASSERT_TRUE(capture_setting.saturation.is_empty());
  ASSERT_TRUE(capture_setting.temperature.is_not_empty());
  ASSERT_TRUE(capture_setting.exposure.is_not_empty());
  ASSERT_TRUE(capture_setting.gain.is_empty());

  ASSERT_EQ(10, capture_setting.brightness);
  ASSERT_EQ(20, capture_setting.contrast);
  ASSERT_EQ(30, capture_setting.temperature);
  ASSERT_EQ(40, capture_setting.exposure);
}

TEST(CaptureSettingTest, ToMsg) {
  shisen_cpp::CaptureSetting capture_setting;

  capture_setting.brightness = 10;
  capture_setting.contrast = 20;
  capture_setting.saturation = 30;
  capture_setting.gain = 40;

  shisen_cpp::CaptureSettingMsg msg = capture_setting;

  ASSERT_EQ(1u, msg.brightness.size());
  ASSERT_EQ(1u, msg.contrast.size());
  ASSERT_EQ(1u, msg.saturation.size());
  ASSERT_EQ(0u, msg.temperature.size());
  ASSERT_EQ(0u, msg.exposure.size());
  ASSERT_EQ(1u, msg.gain.size());

  ASSERT_EQ(10, msg.brightness.front());
  ASSERT_EQ(20, msg.contrast.front());
  ASSERT_EQ(30, msg.saturation.front());
  ASSERT_EQ(40, msg.gain.front());
}

TEST(CaptureSettingTest, UpdateWith) {
  shisen_cpp::CaptureSetting a;

  a.brightness = 10;
  a.contrast = 20;
  a.temperature = 30;
  a.exposure = 40;

  shisen_cpp::CaptureSetting b;

  b.brightness = 40;
  b.contrast = 30;
  b.saturation = 20;
  b.gain = 10;

  a.update_with(b);

  ASSERT_EQ(40, a.brightness);
  ASSERT_EQ(30, a.contrast);
  ASSERT_EQ(20, a.saturation);
  ASSERT_EQ(30, a.temperature);
  ASSERT_EQ(40, a.exposure);
  ASSERT_EQ(10, a.gain);
}
