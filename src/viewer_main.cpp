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

  const char * help_message =
    "Usage: ros2 run shisen_cpp viewer [options] [--camera-prefix prefix]\n"
    "\n"
    "Optional arguments:\n"
    "-h, --help             show this help message and exit\n"
    "--camera-prefix        prefix name for camera's topics and services\n";

  // Handle arguments
  try {
    int i = 1;
    while (i < argc) {
      std::string arg = argv[i++];
      if (arg[0] == '-') {
        if (arg == "-h" || arg == "--help") {
          std::cout << help_message << std::endl;
          return 1;
        } else if (arg == "--camera-prefix") {
          options.camera_prefix = argv[i++];
        } else {
          std::cout << "Unknown option `" << arg << "`!\n\n" << help_message << std::endl;
          return 1;
        }
      }
    }
  } catch (...) {
    std::cout << "Invalid arguments!\n\n" << help_message << std::endl;
    return 1;
  }

  auto node = std::make_shared<rclcpp::Node>("viewer");

  try {
    // auto viewer = std::make_shared<shisen_cpp::ShisenCppViewerNode>(node, options);
    auto image_consumer = std::make_shared<shisen_cpp::viewer::ImageConsumer>();
    auto viewer_node = std::make_shared<shisen_cpp::viewer::ViewerNode>(node, options);
    viewer_node->set_consumer(image_consumer);
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Exception! " << e.what());
  }

  rclcpp::shutdown();

  return 0;
}
