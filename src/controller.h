#pragma once

#include "robot_client.h"
#include "robot_model.h"
#include "types.h"

robot_client::Input controller(const robot_model::RobotModel &model,
                               const Vec2 &setpoint);
