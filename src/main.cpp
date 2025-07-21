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
    static double time_elapsed_s = 0.0;
    static double path_eval_time = 0.0;

    // Update the robot model with new sensor data
    robot_model.update(sensors);

    auto lifetime = robot_model.getLifetimeSeconds();
    auto pos = robot_model.getPosition();
    auto theta = robot_model.getOrientation();

    std::cout << "[INFO] Current state:" << std::endl;
    std::cout << "  lifetime: " << lifetime << "s" << std::endl;
    std::cout << "  position: " << pos.x << " " << pos.y << std::endl;
    std::cout << "  theta: " << theta << "rad" << std::endl;

    // Get the point in the path to target
    auto setpoint = path_generator::eval(path_eval_time);
    std::cout << "[INFO] Setpoint: " << setpoint.x << " " << setpoint.y
              << std::endl;

    // Compute the next robot input
    auto input = controller(robot_model, setpoint);

    auto current_pos = robot_model.getPosition();
    Vec2 distance_vec{setpoint.x - current_pos.x, setpoint.y - current_pos.y};
    double distance = std::sqrt(distance_vec.x * distance_vec.x +
                                distance_vec.y * distance_vec.y);
    std::cout << "[INFO] Distance to target: " << distance << std::endl;
    // If the setpoint is reached, then set the next setpoint
    if (distance <= 0.2 && path_eval_time <= 20) {
        path_eval_time += 0.25;
    }

    // Send the next input data to the robot
    c->sendInputMessage(input);

    std::cout << "[INFO] Sent input data: " << input.v_left << " "
              << input.v_right << std::endl;

    // Update the visualizer with the new state
    if (visualizer) {
        visualizer->setSetpoint(setpoint.x, setpoint.y);
        visualizer->updateRobotModel(robot_model);
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