#include <QApplication>
#include <cmath>
#include <iostream>

#include "controller.h"
#include "path_generator.h"
#include "robot_client.h"
#include "robot_model.h"
#include "visualizer.h"

using json = nlohmann::json;
using client = websocketpp::client<websocketpp::config::asio_client>;

Visualizer *visualizer = nullptr;

void message_cb(robot_client::RobotClient *c,
                const robot_client::Sensors sensors) {
    static robot_model::RobotModel robot_model;

    // Update the robot model with the time elapsed and new sensor data
    robot_model.update(0.1, sensors);

    // Compute the next robot input
    auto input = controller(robot_model, path_generator::eval(0.0));

    // Send the next input data to the robot
    c->sendInputMessage(input);

    std::cout << "[INFO] Sent input data: " << input.v_left << " "
              << input.v_right << std::endl;

    // Update the visualizer with the new position and orientation
    if (visualizer) {
        auto pos = robot_model.getPosition();
        visualizer->updatePosition(pos.x, pos.y);
    }
}

int main(int argc, char *argv[]) {
    // Handle args
    std::string uri = "ws://localhost:9002";
    if (argc == 2) {
        uri = argv[1];
    } else {
        std::cout << "Please provide a websocket address e.g. " << uri
                  << std::endl;
        return 0;
    }

    QApplication app(argc, argv);

    // Create the visualizer window
    Visualizer vis;
    visualizer = &vis;

    // Draw the path in the visualizer
    vis.setPathFunction(path_generator::eval, 0, 21, 0.1);

    vis.show();

    try {
        auto c = robot_client::RobotClient();
        c.set_message_cb(message_cb);
        c.connect(uri);

        // Run the WebSocket client in a separate thread
        std::thread client_thread([&c]() { c.run(); });

        // Run the Qt event loop
        int result = app.exec();

        // Wait for the WebSocket client thread to finish
        client_thread.join();

        return result;

    } catch (websocketpp::exception const &e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
}