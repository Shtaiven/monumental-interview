#include <iostream>
#include <nlohmann/json.hpp>
#include <websocketpp/client.hpp>
#include <websocketpp/config/asio_no_tls_client.hpp>

#include "robot.h"

typedef websocketpp::client<websocketpp::config::asio_client> client;

using websocketpp::lib::bind;
using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using json = nlohmann::json;

typedef websocketpp::config::asio_client::message_type::ptr message_ptr;

// Called on every message from the websocket server
void on_message(client *c, websocketpp::connection_hdl hdl, message_ptr msg) {
  json j = json::parse(msg->get_payload(), nullptr, false);

  // Check if the JSON is valid
  if (j.is_discarded()) {
    std::cout << "[ERROR] Received invalid JSON: " << msg->get_payload()
              << std::endl;
    return;
  }

  try {
    // Convert JSON to robot::Sensors
    auto sensors = j.template get<robot::Sensors>();

    // Print the received sensors data
    std::cout << "[INFO] Received sensors data: " << std::endl;
    for (const auto &sensor : sensors.sensors) {
      std::cout << "  Sensor Name: " << sensor.name << std::endl;
      std::cout << "  Data: ";
      for (const auto &value : sensor.data) {
        std::cout << value << " ";
      }
      std::cout << std::endl;
      std::cout << "  Unit: " << sensor.unit << std::endl;
      std::cout << "  Timestamp: " << sensor.timestamp << std::endl;
    }
  } catch (const json::exception &e) {
    std::cout << "[ERROR] JSON exception: " << e.what() << std::endl;
    return;
  }

  // Send the next input data to the robot
  robot::Input input;
  input.v_left = 0;   // Example value for left wheel speed
  input.v_right = 0;  // Example value for right wheel speed
  json response = input;
  c->send(hdl, response.dump(), websocketpp::frame::opcode::text);
  std::cout << "[INFO] Sent input data: " << input.v_left << " "
            << input.v_right << std::endl;
}

int main(int argc, char *argv[]) {
  client c;

  std::string uri = "ws://localhost:9002";

  if (argc == 2) {
    uri = argv[1];
  } else {
    std::cout << "Please provide a websocket address e.g. " << uri << std::endl;
    return 0;
  }

  try {
    // Websocket client setup
    c.set_access_channels(websocketpp::log::alevel::connect |
                          websocketpp::log::alevel::disconnect);
    c.clear_access_channels(websocketpp::log::alevel::frame_payload);

    c.init_asio();
    c.set_message_handler(bind(&on_message, &c, ::_1, ::_2));

    websocketpp::lib::error_code ec;
    client::connection_ptr con = c.get_connection(uri, ec);
    if (ec) {
      std::cout << "could not create connection because: " << ec.message()
                << std::endl;
      return 0;
    }

    c.connect(con);
    c.run();

  } catch (websocketpp::exception const &e) {
    std::cout << e.what() << std::endl;
  }
}