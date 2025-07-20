#include "controller.h"

robot_client::Input controller(const robot_model::RobotModel &model,
                               const Point &setpoint) {
    // TODO: Implement controller
    return robot_client::Input{.v_left = 0.5, .v_right = 0.3};
}
