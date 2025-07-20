#include "robot_model.h"

namespace robot_model {

void RobotModel::update(double dt,
                        std::optional<robot_client::Sensors> sensors) {
    // TODO: Update robot model

    // Update position based on gps
    if (sensors.has_value()) {
        // Process sensor data if available
        for (const auto &sensor : sensors->sensors) {
            if (sensor.name == "gps" && sensor.data.size() >= 2) {
                pos_.x = sensor.data[0];  // Update x position
                pos_.y = sensor.data[1];  // Update y position
                break;
            }
        }
    }
}

}  // namespace robot_model
