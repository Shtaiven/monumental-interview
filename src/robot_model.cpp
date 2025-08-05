#include "robot_model.h"

#include <chrono>
#include <cmath>
#include <regex>
#include <string>

#include "particle_filter.h"

namespace robot_model {

static std::optional<int64_t> stringToTimestampMillis(
    const std::string &str_time) {
    // Regex to extract date, time, fractional seconds
    // Parses strings of the form 2025-07-20T10:55:58.902217258+00:00
    std::regex re(
        R"((\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2})(\.\d+)?(?:\+\d{2}:\d{2}|Z)?)");
    std::smatch match;
    if (!std::regex_search(str_time, match, re)) {
        return std::nullopt;
    }

    std::string datetime = match[1];
    std::string fractional = match[2].matched ? match[2].str() : ".0";

    // Parse date and time
    std::tm tm = {};
    std::istringstream ss(datetime);
    ss >> std::get_time(&tm, "%Y-%m-%dT%H:%M:%S");
    if (ss.fail()) return std::nullopt;

    // Convert to time_t (seconds since epoch)
    time_t seconds = timegm(&tm);  // Use timegm for UTC

    // Parse fractional seconds
    double fraction = std::stod(fractional);

    // Combine seconds and fractional part
    int64_t millis = static_cast<int64_t>(seconds * 1000 + fraction * 1000);

    return millis;
}

RobotModel::RobotModel() {
    double std_pos[3] = {0.2, 0.2, 0.05};  // initial uncertainty
    pf_.init(0, 0, 0, std_pos);
}

double RobotModel::getLifetimeSeconds() const {
    if (!angular_vel_time_.has_value() || !gps_pos_time_.has_value() ||
        !start_time_.has_value()) {
        return 0.0;
    }

    return (std::max(angular_vel_time_.value_or(0.0),
                     gps_pos_time_.value_or(0.0)) -
            start_time_.value_or(0.0)) /
           1000.0;
}

void RobotModel::update(const robot_client::Sensors &sensors) {
    // Update robot state based on sensor data
    for (const auto &sensor : sensors.sensors) {
        if (sensor.name == "accelerometer" && sensor.data.size() >= 2) {
            linear_acc_.x = sensor.data[0];
            linear_acc_.y = sensor.data[1];
            last_linear_acc_time_ = linear_acc_time_;
            linear_acc_time_ = stringToTimestampMillis(sensor.timestamp);
        } else if (sensor.name == "gyro" && sensor.data.size() >= 1) {
            angular_vel_ = sensor.data[0];
            last_angular_vel_time_ = angular_vel_time_;
            angular_vel_time_ = stringToTimestampMillis(sensor.timestamp);
        } else if (sensor.name == "gps" && sensor.data.size() >= 2) {
            gps_pos_.x = sensor.data[0];
            gps_pos_.y = sensor.data[1];
            last_gps_pos_time_ = gps_pos_time_;
            gps_pos_time_ = stringToTimestampMillis(sensor.timestamp);
        }
    }

    // Get the start time if it wasn't already obtained
    if (!start_time_.has_value() && angular_vel_time_.has_value() &&
        gps_pos_time_.has_value()) {
        start_time_ = std::min(angular_vel_time_.value_or(0.0),
                               gps_pos_time_.value_or(0.0));
    }

    double dt = 0.05;  // Default to 20Hz
    if (last_linear_acc_time_.has_value() && linear_acc_time_.has_value()) {
        dt = (*linear_acc_time_ - *last_linear_acc_time_) / 1000.0;
        if (dt <= 0.0) dt = 0.05;
    }

    double std_motion[3] = {0.1, 0.1, 0.1};  // IMU noise
    pf_.predict(dt, angular_vel_, linear_acc_.x, linear_acc_.y, std_motion);

    double std_gps[2] = {0.05, 0.05};  // GPS noise
    pf_.updateGPS(gps_pos_.x, gps_pos_.y, std_gps);
}

}  // namespace robot_model
