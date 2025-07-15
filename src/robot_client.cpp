#include "robot_client.h"

#include <iostream>
#include <nlohmann/json.hpp>
#include <websocketpp/client.hpp>
#include <websocketpp/config/asio_no_tls_client.hpp>

using json = nlohmann::json;

namespace robot_client {

RobotClient::RobotClient() : connected_(false) {
  try {
    // Websocket client setup
    client_.clear_access_channels(websocketpp::log::alevel::all);
    client_.set_access_channels(websocketpp::log::alevel::connect);
    client_.set_access_channels(websocketpp::log::alevel::disconnect);
    client_.set_access_channels(websocketpp::log::alevel::app);

    client_.init_asio();

    using websocketpp::lib::bind;
    using websocketpp::lib::placeholders::_1;
    using websocketpp::lib::placeholders::_2;
    client_.set_message_handler(bind(&RobotClient::on_message, this, _1, _2));

  } catch (websocketpp::exception const &e) {
    std::cout << e.what() << std::endl;
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

}  // namespace robot_client
