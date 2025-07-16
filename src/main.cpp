#include <iostream>

#include "robot_client.h"

using json = nlohmann::json;
using client = websocketpp::client<websocketpp::config::asio_client>;

void message_cb(robot_client::RobotClient *c, const robot_client::Sensors) {
    // TODO: Connect to visualizer socket

    // TODO: Implement controller
    // Send the next input data to the robot
    robot_client::Input input{
        .v_left = 0,  // Example value for left wheel speed
        .v_right = 0  // Example value for right wheel speed
    };

    c->send_input_message(input);

    std::cout << "[INFO] Sent input data: " << input.v_left << " "
              << input.v_right << std::endl;
}

int main(int argc, char *argv[]) {
    std::string uri = "ws://localhost:9002";

    if (argc == 2) {
        uri = argv[1];
    } else {
        std::cout << "Please provide a websocket address e.g. " << uri
                  << std::endl;
        return 0;
    }

    try {
        auto c = robot_client::RobotClient();
        c.set_message_cb(message_cb);
        c.connect(uri);
        c.run();

    } catch (websocketpp::exception const &e) {
        std::cout << e.what() << std::endl;
    }
}