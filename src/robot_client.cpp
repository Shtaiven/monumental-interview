#include "robot_client.h"

#include <iostream>
#include <nlohmann/json.hpp>
#include <websocketpp/client.hpp>
#include <websocketpp/config/asio_no_tls_client.hpp>

using websocketpp::lib::bind;
using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using json = nlohmann::json;

namespace robot_client {

RobotClient::RobotClient() : connected_(false) {
    // Websocket client setup
    client_.clear_access_channels(websocketpp::log::alevel::all);
    client_.set_access_channels(websocketpp::log::alevel::connect);
    client_.set_access_channels(websocketpp::log::alevel::disconnect);
    client_.set_access_channels(websocketpp::log::alevel::app);

    client_.init_asio();

    client_.set_message_handler(bind(&RobotClient::onMessage, this, _1, _2));
}

RobotClient::~RobotClient() {
    if (connected_) {
        client_.close(connection_hdl_, websocketpp::close::status::normal,
                      "RobotClient connection closed");
    }
}

bool RobotClient::connect(const std::string &uri) {
    uri_ = uri;
    websocketpp::lib::error_code ec;

    auto con = client_.get_connection(uri_, ec);
    if (ec) {
        std::cout << "[ERROR] Could not create connection: " << ec.message()
                  << std::endl;
        return false;
    }

    connection_hdl_ = con->get_handle();

    client_.connect(con);
    connected_ = true;
    return true;
}

// Called on every message from the websocket server
void RobotClient::onMessage(
    websocketpp::connection_hdl hdl,
    websocketpp::config::asio_client::message_type::ptr msg) {
    json j = json::parse(msg->get_payload(), nullptr, false);

    // Check if the JSON is valid
    if (j.is_discarded()) {
        std::cout << "[ERROR] Received invalid JSON: " << msg->get_payload()
                  << std::endl;
        return;
    }
    // else {
    //     std::cout << "[DEBUG] Received message: " << msg->get_payload() <<
    //     std::endl;
    // }

    try {
        if (j.contains("message_type") && j["message_type"] == "score") {
            score_ = j.template get<robot_client::Score>();
            return;
        }

        // Convert JSON to robot_client::Sensors
        auto sensors = j.template get<robot_client::Sensors>();

        // Call the custom message callback
        if (message_cb_) {
            message_cb_(this, sensors);
        }
    } catch (const json::exception &e) {
        std::cout << "[ERROR] JSON exception: " << e.what() << std::endl;
    }
}

void RobotClient::sendInputMessage(Input input) {
    json response = input;
    client_.send(connection_hdl_, response.dump(),
                 websocketpp::frame::opcode::text);
}

std::optional<robot_client::Score> RobotClient::getScore() const {
    return score_;
}

void RobotClient::run() { client_.run(); }

}  // namespace robot_client
