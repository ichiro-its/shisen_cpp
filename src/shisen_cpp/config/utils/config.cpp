#include "shisen_cpp/config/utils/config.hpp"

#include <fstream>
#include <iomanip>
#include <string>

#include "nlohmann/json.hpp"

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
