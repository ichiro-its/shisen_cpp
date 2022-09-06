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

#include <memory>

#include <shisen_cpp/node/shisen_cpp_viewer_node.hpp>

namespace shisen_cpp
{
using namespace std::chrono_literals;

ShisenCppViewerNode::ShisenCppViewerNode(rclcpp::Node::SharedPtr node, const Options & options)
: node(node)
{
  viewer_node = std::make_shared<shisen_cpp::ViewerNode>(node, options);
  auto image_consumer = std::make_shared<shisen_cpp::ImageConsumer>();
  viewer_node->set_consumer(image_consumer);
}

ShisenCppViewerNode::~ShisenCppViewerNode()
{
}

}  // namespace shisen_cpp
