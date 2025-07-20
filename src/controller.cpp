#include "controller.h"

robot_client::Input controller(const robot_model::RobotModel &model,
                               const Vec2 &setpoint) {
    // TODO: Implement controller
    return robot_client::Input{.v_left = 0.0, .v_right = 1.0};
}
