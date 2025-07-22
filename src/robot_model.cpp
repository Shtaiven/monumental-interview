#include "robot_model.h"

#include <chrono>
#include <cmath>
#include <regex>
#include <string>

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
    state_.x.setZero();
    state_.P.setIdentity();
    state_.P *= 10.0;  // Initial uncertainty
}

Vec2 RobotModel::getPosition() const { return Vec2{state_.x(0), state_.x(1)}; }

double RobotModel::getOrientation() const { return state_.x(2); }

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

    // Prediction step
    double dt = 0.05;  // Default to 50Hz
    if (last_linear_acc_time_.has_value() && linear_acc_time_.has_value()) {
        dt = (*linear_acc_time_ - *last_linear_acc_time_) / 1000.0;
        if (dt <= 0.0) dt = 0.05;
    }
    if (linear_acc_time_.has_value()) {
        ekfPredict(dt, angular_vel_, linear_acc_.x, linear_acc_.y);
    }

    // Correction step (GPS)
    if (gps_pos_time_.has_value() &&
        (!last_gps_pos_time_ || gps_pos_time_ != last_gps_pos_time_)) {
        ekfUpdateGPS(gps_pos_.x, gps_pos_.y);
    }
}

void RobotModel::ekfPredict(double dt, double gyro_z, double acc_x,
                            double acc_y) {
    // State: [x, y, theta, v_x, v_y]
    double x = state_.x(0), y = state_.x(1), theta = state_.x(2),
           v_x = state_.x(3), v_y = state_.x(4);

    // Rotate acceleration to global frame
    double ax_global = std::cos(theta) * acc_x - std::sin(theta) * acc_y;
    double ay_global = std::sin(theta) * acc_x + std::cos(theta) * acc_y;

    // Predict state
    Eigen::Matrix<double, 5, 1> x_pred;
    x_pred(0) = x + v_x * dt + 0.5 * ax_global * dt * dt;
    x_pred(1) = y + v_y * dt + 0.5 * ay_global * dt * dt;
    x_pred(2) = theta + gyro_z * dt;
    x_pred(3) = v_x + ax_global * dt;
    x_pred(4) = v_y + ay_global * dt;

    // Jacobian F (linearized motion model)
    Eigen::Matrix<double, 5, 5> F = Eigen::Matrix<double, 5, 5>::Identity();
    F(0, 2) =
        -v_x * dt * std::sin(theta) - 0.5 * acc_x * dt * dt * std::sin(theta) -
        v_y * dt * std::cos(theta) - 0.5 * acc_y * dt * dt * std::cos(theta);
    F(1, 2) =
        v_x * dt * std::cos(theta) + 0.5 * acc_x * dt * dt * std::cos(theta) -
        v_y * dt * std::sin(theta) - 0.5 * acc_y * dt * dt * std::sin(theta);
    F(2, 2) = 1;
    F(3, 2) = -acc_x * dt * std::sin(theta) - acc_y * dt * std::cos(theta);
    F(4, 2) = acc_x * dt * std::cos(theta) - acc_y * dt * std::sin(theta);

    // Process noise covariance Q (tune as needed)
    Eigen::Matrix<double, 5, 5> Q = Eigen::Matrix<double, 5, 5>::Zero();
    Q(0, 0) = 0.2;   // x position noise (m^2)
    Q(1, 1) = 0.2;   // y position noise (m^2)
    Q(2, 2) = 0.05;  // theta noise (rad^2)
    Q(3, 3) = 0.05;  // v_x noise (m^2/s^2)
    Q(4, 4) = 0.05;  // v_y noise (m^2/s^2)

    // Covariance prediction
    state_.x = x_pred;
    state_.P = F * state_.P * F.transpose() + Q;
}

void RobotModel::ekfUpdateGPS(double gps_x, double gps_y) {
    // Measurement model: z = [gps_x, gps_y]
    Eigen::Vector2d z(gps_x, gps_y);
    Eigen::Vector2d z_pred(state_.x(0), state_.x(1));

    // Measurement Jacobian H
    Eigen::Matrix<double, 2, 5> H = Eigen::Matrix<double, 2, 5>::Zero();
    H(0, 0) = 1;
    H(1, 1) = 1;

    // Measurement noise covariance R
    Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * gps_noise_;

    // Kalman gain
    Eigen::Matrix<double, 5, 2> K =
        state_.P * H.transpose() * (H * state_.P * H.transpose() + R).inverse();

    // Update state and covariance
    state_.x += K * (z - z_pred);
    state_.P = (Eigen::Matrix<double, 5, 5>::Identity() - K * H) * state_.P;
}

// void RobotModel::computePosition() {
//     // Only update gps position data if we have new data
//     // TODO: GPS data is crap, use an Extended Kalman Filter
//     bool gps_data_updated =
//         gps_pos_time_.has_value() && last_gps_pos_time_ != gps_pos_time_;
//     if (gps_data_updated) {
//         pos_ = gps_pos_;
//     }

//     // Early exit if we don't have a new sensor value
//     if (!angular_vel_time_.has_value() || !last_angular_vel_time_.has_value()
//     ||
//         (last_angular_vel_time_ == angular_vel_time_)) {
//         std::cout << "[WARN] No new gyro sensor values" << std::endl;
//         return;
//     }

//     // Forward kinematics computations explained by the paper below:
//     // https://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf

//     // Get change in time since last measurement
//     double dt = 0.0;
//     if (last_angular_vel_time_.has_value()) {
//         dt = (*angular_vel_time_ - *last_angular_vel_time_) /
//              1000.0;  // Convert ms to seconds
//     }

//     if (v_left_ == v_right_) {
//         // Case 1: Moving forward
//         pos_.x += v_left_ * std::cos(theta_) * dt;
//         pos_.y += v_left_ * std::sin(theta_) * dt;
//         // theta_ remains the same

//     } else if (v_left_ == -v_right_) {
//         // Case 2: Rotating in place
//         theta_ += 2 * v_right_ * dt / axle_length_;
//         // pos_ remains the same
//     } else {
//         // Case 3: Velocities are completely different
//         // Compute the distance to the instantaneous center of curvature
//         double icc_distance =
//             (axle_length_ / 2.0) * (v_left_ + v_right_) / (v_right_ -
//             v_left_);

//         // Compute the instantaneous center of curvature
//         Vec2 icc{pos_.x - icc_distance * std::sin(theta_),
//                  pos_.y + icc_distance * std::cos(theta_)};

//         double theta_diff = angular_vel_ * dt;

//         std::cout << "[DEBUG] dt: " << dt << "s" << std::endl;
//         std::cout << "[DEBUG] theta_diff: " << theta_diff << "rad" <<
//         std::endl; std::cout << "[DEBUG] icc: " << icc.x << " " << icc.y <<
//         std::endl;

//         // Update the position
//         pos_.x = std::cos(theta_diff) * (pos_.x - icc.x) -
//                  std::sin(theta_diff) * (pos_.y - icc.y) + icc.x;
//         pos_.y = std::sin(theta_diff) * (pos_.x - icc.x) +
//                  std::cos(theta_diff) * (pos_.y - icc.y) + icc.y;
//         theta_ += theta_diff;
//     }
// }

}  // namespace robot_model
