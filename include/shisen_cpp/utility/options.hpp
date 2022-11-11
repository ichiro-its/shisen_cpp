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

#ifndef SHISEN_CPP__UTILITY__OPTIONS_HPP_
#define SHISEN_CPP__UTILITY__OPTIONS_HPP_

#include <string>

namespace shisen_cpp
{

struct Options
{
  std::string camera_file_name;
  std::string camera_prefix;
  int capture_fps;
  int compression_quality;
  int field_of_view;
  bool publish_image;

  Options()
  : camera_file_name("/dev/video0"),
    camera_prefix("camera"),
    capture_fps(60),
    compression_quality(-1),
    field_of_view(-1)
  {
  }
};

}  // namespace shisen_cpp

#endif  // SHISEN_CPP__UTILITY__OPTIONS_HPP_
