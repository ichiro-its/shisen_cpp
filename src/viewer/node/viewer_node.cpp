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

#include <shisen_cpp/viewer/node/viewer_node.hpp>

namespace shisen_cpp
{

ViewerNode::ViewerNode(rclcpp::Node::SharedPtr node, const Options & options)
: node(node), options(options)
{
}

ViewerNode::~ViewerNode()
{
}

const std::string & ViewerNode::get_camera_prefix() const
{
  return options.camera_prefix;
}

void ViewerNode::set_consumer(
  std::shared_ptr<ImageConsumer> img_consumer)
{
  image_consumer = img_consumer;

  // Initialize the image subscription
  {
    image_subscription = node->create_subscription<Image>(
      get_camera_prefix() + IMAGE_SUFFIX, 10,
      [this](const Image::SharedPtr msg) {
        auto current_image = *msg;

        // Call callback after image changed
        image_consumer->on_image_changed(current_image);
      });

    RCLCPP_INFO_STREAM(
      node->get_logger(),
      "Image subscription initialized on `" << image_subscription->get_topic_name() << "`!");
  }
}

}  // namespace shisen_cpp
