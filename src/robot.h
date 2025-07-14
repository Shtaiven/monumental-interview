#pragma once

#include <chrono>
#include <ctime>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace robot {

struct Sensor {
  std::string name;
  std::vector<double> data;
  std::string unit;
  std::string timestamp;
};

struct Sensors {
  std::string message_type;
  std::vector<Sensor> sensors;
};

struct Input {
  double v_left;
  double v_right;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Sensor, name, data, unit,
                                                timestamp)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Sensors, message_type, sensors)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Input, v_left, v_right)

}  // namespace robot
