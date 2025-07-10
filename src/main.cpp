#include <iostream>
#include <nlohmann/json.hpp>
#include <websocketpp/client.hpp>
#include <websocketpp/config/asio_no_tls_client.hpp>

typedef websocketpp::client<websocketpp::config::asio_client> client;

using websocketpp::lib::bind;
using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using json = nlohmann::json;

typedef websocketpp::config::asio_client::message_type::ptr message_ptr;

// Called on every message from the websocket server
void on_message(client *c, websocketpp::connection_hdl hdl, message_ptr msg) {
  json j{json::parse(msg->get_payload(), nullptr, false)};
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