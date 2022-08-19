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

#ifndef SHISEN_CPP__CAMERA__IMAGE__NODE__IMAGE_NODE_HPP_
#define SHISEN_CPP__CAMERA__IMAGE__NODE__IMAGE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

#include "shisen_cpp/camera/image/provider/image_provider.hpp"
#include "shisen_cpp/node/base_node.hpp"

namespace shisen_cpp
{

class ImageNode : public BaseNode
{
public:
  explicit ImageNode(
    rclcpp::Node::SharedPtr node, std::shared_ptr<ImageProvider> img_provider);
  ~ImageNode();

  void update();

private:
  rclcpp::Publisher<Image>::SharedPtr image_publisher;

  std::shared_ptr<ImageProvider> image_provider;
};

}  // namespace shisen_cpp

#endif  // SHISEN_CPP__CAMERA__IMAGE__NODE__IMAGE_NODE_HPP_
