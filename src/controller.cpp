#include "controller.h"

#include "robot_client.h"

robot_client::Input controller(robot_client::Sensors sensors) {
    // TODO: Implement controller
    return robot_client::Input{.v_left = 0, .v_right = 0};
}
