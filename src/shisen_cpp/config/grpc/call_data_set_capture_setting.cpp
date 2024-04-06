#include "shisen_cpp/config/grpc/call_data_set_capture_setting.hpp"

#include "shisen_cpp/config/utils/config.hpp"
#include "shisen_interfaces/shisen.grpc.pb.h"
#include "shisen_interfaces/shisen.pb.h"
#include "rclcpp/rclcpp.hpp"

namespace shisen_cpp
{
CallDataSetCaptureSetting::CallDataSetCaptureSetting(
  shisen_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
  const std::string & path, rclcpp::Node::SharedPtr node)
: CallData(service, cq, path), node_(node)
{
  set_capture_publisher_ = node_->create_publisher<shisen_interfaces::msg::CaptureSetting>(
    "shisen_cpp/config/capture_setting", 10);
  Proceed();
}

void CallDataSetCaptureSetting::AddNextToCompletionQueue()
{
  new CallDataSetCaptureSetting(service_, cq_, path_, node_);
}

void CallDataSetCaptureSetting::WaitForRequest()
{
  service_->RequestSetCaptureSetting(&ctx_, &request_, &responder_, cq_, cq_, this);
}

void CallDataSetCaptureSetting::HandleRequest()
{
  Config config(path_);
  try {
    int brightness = request_.brightness();
    int contrast = request_.contrast();
    int saturation = request_.saturation();
    int temperature = request_.temperature();
    int exposure = request_.exposure();
    int gain = request_.gain();

    std::cout << "brightness: " << brightness << std::endl;
    std::cout << "contrast: " << contrast << std::endl;
    std::cout << "saturation: " << saturation << std::endl;
    std::cout << "temperature: " << temperature << std::endl;
    std::cout << "exposure: " << exposure << std::endl;
    std::cout << "gain: " << gain << std::endl;

    shisen_interfaces::msg::CaptureSetting msg;
    msg.brightness.clear();
    msg.contrast.clear();
    msg.saturation.clear();
    msg.temperature.clear();
    msg.exposure.clear();
    msg.gain.clear();

    msg.brightness.push_back(brightness);
    msg.contrast.push_back(contrast);
    msg.saturation.push_back(saturation);
    msg.temperature.push_back(temperature);
    msg.exposure.push_back(exposure);
    msg.gain.push_back(gain);

    // msg.brightness = brightness;
    // msg.contrast = contrast;
    // msg.saturation = saturation;
    // msg.temperature = temperature;
    // msg.exposure = exposure;
    // msg.gain = gain;

    set_capture_publisher_->publish(msg);
    RCLCPP_INFO(
      rclcpp::get_logger("Publish capture setting config"),
      "capture setting config has been applied!");
  } catch (nlohmann::json::exception e) {
    RCLCPP_ERROR(rclcpp::get_logger("Publish config"), e.what());
  }
}
}  // namespace shisen_cpp