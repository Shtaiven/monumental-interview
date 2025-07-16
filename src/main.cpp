#include <QApplication>
#include <cmath>
#include <iostream>

#include "controller.h"
#include "robot_client.h"
#include "visualizer.h"

using json = nlohmann::json;
using client = websocketpp::client<websocketpp::config::asio_client>;

Visualizer *visualizer = nullptr;

void message_cb(robot_client::RobotClient *c,
                const robot_client::Sensors sensors) {
    static double last_x = 0.0;
    static double last_y = 0.0;
    static double last_theta = 0.0;
    static bool has_last = false;

    for (const auto &sensor : sensors.sensors) {
        if (sensor.name == "gps" && sensor.data.size() >= 2) {
            double x = sensor.data[0];  // GPS x-coordinate
            double y = sensor.data[1];  // GPS y-coordinate

            double dx = 0.0;
            double dy = 0.0;
            if (has_last) {
                dx = x - last_x;
                dy = y - last_y;
            }
            last_x = x;
            last_y = y;
            has_last = true;

            // Update the visualizer with the new position and orientation
            if (visualizer) {
                visualizer->updatePosition(x, y);
            }
        }
    }

    // Send the next input data to the robot
    auto input = controller(sensors);
    c->sendInputMessage(input);

    std::cout << "[INFO] Sent input data: " << input.v_left << " "
              << input.v_right << std::endl;
}

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    // Create the visualizer window
    Visualizer vis;
    visualizer = &vis;
    vis.show();

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