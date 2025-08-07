#pragma once

#include <functional>
#include <iostream>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>
#include <websocketpp/client.hpp>
#include <websocketpp/config/asio_no_tls_client.hpp>

namespace robot_client {

struct Sensor {
    std::string name;
    std::vector<double> data;
    std::string unit;
    std::string timestamp;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Sensor, name, data, unit,
                                                timestamp)

struct Sensors {
    std::string message_type;
    std::vector<Sensor> sensors;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Sensors, message_type, sensors)

struct Input {
    double v_left;
    double v_right;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Input, v_left, v_right)

struct Score {
    std::string message_type;
    double score;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Score, message_type, score)

class RobotClient;  // forward declaration for typedef

using MessageCallback = std::function<void(RobotClient *, const Sensors)>;

class RobotClient {
   public:
    RobotClient();
    ~RobotClient();

    bool connect(const std::string &uri);
    void run();
    void set_message_cb(MessageCallback cb) { message_cb_ = cb; };
    void sendInputMessage(Input input);
    std::optional<Score> getScore() const;

   private:
    void onMessage(websocketpp::connection_hdl hdl,
                   websocketpp::config::asio_client::message_type::ptr msg);

    websocketpp::client<websocketpp::config::asio_client> client_;
    websocketpp::connection_hdl connection_hdl_;
    bool connected_;
    std::string uri_;
    nlohmann::json sensors_data_;
    std::optional<robot_client::Score> score_{std::nullopt};
    MessageCallback message_cb_ = NULL;
};

}  // namespace robot_client
