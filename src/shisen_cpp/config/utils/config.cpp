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

#include "nlohmann/json.hpp"
#include "shisen_cpp/config/utils/config.hpp"

#include <fstream>
#include <iomanip>
#include <string>

namespace shisen_cpp
{
Config::Config(const std::string & path) : path(path) {}

std::string Config::get_capture_setting(const std::string & key) const
{
  if (key == "capture") {
    std::ifstream capture_file(path + "capture_settings.json");
    nlohmann::json capture_data = nlohmann::json::parse(capture_file);
    return capture_data.dump();
  }

  return "";
}

nlohmann::json Config::get_grpc_config() const
{
  std::ifstream grpc_file(path + "grpc.json");
  nlohmann::json grpc_data = nlohmann::json::parse(grpc_file);
  grpc_file.close();
  return grpc_data;
}

void Config::save_capture_setting(const nlohmann::json & capture_data)
{
  std::ofstream capture_file(path + "capture_settings.json", std::ios::out | std::ios::trunc);
  capture_file << std::setw(2) << capture_data << std::endl;
  capture_file.close();
}

}  // namespace shisen_cpp
