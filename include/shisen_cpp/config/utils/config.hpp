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
