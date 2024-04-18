// Copyright (c) 2024 ICHIRO ITS
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

#ifndef SHISEN_CPP__CONFIG__UTILS__CONFIG_HPP_
#define SHISEN_CPP__CONFIG__UTILS__CONFIG_HPP_

#include <fstream>
#include <map>
#include <string>

#include "nlohmann/json.hpp"

namespace shisen_cpp
{

class Config
{
public:
  explicit Config(const std::string & path);

  std::string get_capture_setting(const std::string & key) const;
  void save_capture_setting(const nlohmann::json & capture_data);
  nlohmann::json get_grpc_config() const;

private:
  std::string path;
};

}  // namespace shisen_cpp

#endif  // SHISEN_CPP__CONFIG__UTILS__CONFIG_HPP_
