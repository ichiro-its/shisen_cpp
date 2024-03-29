// Copyright (c) 2020-2021 ICHIRO ITS
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

#include <rclcpp/rclcpp.hpp>
#include <shisen_cpp/shisen_cpp.hpp>

#include <memory>
#include <string>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  shisen_cpp::Options options;
  options.field_of_view = 78;

  const char * help_message =
    "Usage: ros2 run shisen_cpp camera [options] [--camera-prefix prefix]\n"
    "       [--compression quality] [--captured-fps fps] [camera_file_name]\n"
    "\n"
    "Positional arguments:\n"
    "camera_file_name     camera's device file name\n"
    "\n"
    "Optional arguments:\n"
    "-h, --help           show this help message and exit\n"
    "--camera-prefix      prefix name for camera's topics and services\n"
    "--compression        compression quality to be used\n"
    "--capture-fps        captured frames per second"
    "--field-of-view      for calculation view_v_angle and view_h_angle";

  // Handle arguments
  try {
    int i = 1;
    int pos = 0;
    while (i < argc) {
      std::string arg = argv[i++];
      if (arg[0] == '-') {
        if (arg == "-h" || arg == "--help") {
          std::cout << help_message << std::endl;
          return 1;
        } else if (arg == "--camera-prefix") {
          options.camera_prefix = argv[i++];
        } else if (arg == "--compression") {
          options.compression_quality = atoi(argv[i++]);
        } else if (arg == "--capture-fps") {
          options.capture_fps = atoi(argv[i++]);
        } else if (arg == "--field-of-view") {
          options.field_of_view = atoi(argv[i++]);
        } else {
          std::cout << "Unknown option `" << arg << "`!\n\n" << help_message << std::endl;
          return 1;
        }
      } else if (pos == 0) {
        options.camera_file_name = arg;
        ++pos;
      }
    }
  } catch (...) {
    std::cout << "Invalid arguments!\n\n" << help_message << std::endl;
    return 1;
  }

  auto node = std::make_shared<rclcpp::Node>("camera");

  try {
    auto camera = std::make_shared<shisen_cpp::ShisenCppNode>(node, options);
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Exception! " << e.what());
  }

  rclcpp::shutdown();

  return 0;
}
