#include "visualizer.h"

#include <QPainterPath>
#include <cmath>

Visualizer::Visualizer(QWidget *parent) : QWidget(parent) {
    setFixedSize(window_width, window_height);  // Set the window size
}

void Visualizer::updateRobotModel(const robot_model::RobotModel &model) {
    auto pos = model.getPfPosition();
    robot_x = width() / 2.0 + pos.x * size_multiplier;
    robot_y = height() / 2.0 - pos.y * size_multiplier;
    robot_theta = model.getPfOrientation();
    particles = model.getPfParticles();

    trail.push_back(QPointF(robot_x, robot_y));
    if (trail.size() > 1000) {
        trail.pop_front();
    }

    update();
}

QPainterPath Visualizer::drawPath(std::function<Vec2(double)> path_func,
                                  double start, double stop, double step) {
    QPainterPath path;
    if (!path_func) {
        return path;  // No path function set
    }

    for (double t = start; t < stop; t += step) {
        auto p = path_func(t);
        QPointF point(width() / 2.0 + p.x * size_multiplier,
                      height() / 2.0 - p.y * size_multiplier);
        if (t == start) {
            path.moveTo(point);
        } else {
            path.lineTo(point);
        }
    }
    return path;
}

void Visualizer::paintEvent(QPaintEvent *event) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // Clear the background
    painter.fillRect(rect(), Qt::black);

    // Draw path
    if (path_func) {
        painter.setPen(QPen(Qt::blue, 2));
        painter.setBrush(Qt::NoBrush);
        auto path = drawPath(path_func, path_start, path_stop, path_step);
        painter.drawPath(path);
    }

    // Draw the robot body
    painter.setPen(Qt::NoPen);
    painter.setBrush(Qt::red);
    painter.drawEllipse(QPointF(robot_x, robot_y), 0.25 * size_multiplier,
                        0.25 * size_multiplier);

    double wheel_offset =
        0.25 * size_multiplier;  // distance from center to wheel
    double wheel_radius = 0.1 * size_multiplier;  // radius of wheels

    // Draw heading indicator (yellow line in direction of orientation)
    double heading_length = 0.4 * size_multiplier;
    double hx = robot_x + std::cos(robot_theta) * heading_length;
    double hy =
        robot_y - std::sin(robot_theta) *
                      heading_length;  // minus because screen y is inverted

    // Perpendicular vector for wheels (90 degrees from heading)
    double perp_dx = -std::sin(robot_theta);
    double perp_dy = std::cos(robot_theta);

    // Wheel positions
    double wx1 = robot_x + perp_dx * wheel_offset;
    double wy1 =
        robot_y - perp_dy * wheel_offset;  // minus because screen y is inverted
    double wx2 = robot_x - perp_dx * wheel_offset;
    double wy2 = robot_y + perp_dy * wheel_offset;

    // Draw wheels
    painter.setBrush(Qt::gray);
    painter.drawEllipse(QPointF(wx1, wy1), wheel_radius, wheel_radius);
    painter.drawEllipse(QPointF(wx2, wy2), wheel_radius, wheel_radius);

    // Draw heading line
    painter.setPen(QPen(Qt::yellow, 2));
    painter.drawLine(QPointF(robot_x, robot_y), QPointF(hx, hy));

    // Draw the trail above everything else
    if (trail.size() > 1) {
        painter.setPen(QPen(Qt::green, 2));
        painter.setBrush(Qt::NoBrush);
        for (size_t i = 1; i < trail.size(); ++i) {
            painter.drawLine(trail[i - 1], trail[i]);
        }
    }

    // Draw the setpoint as a cyan "X"
    double sx = width() / 2.0 + setpoint_x * size_multiplier;
    double sy = height() / 2.0 - setpoint_y * size_multiplier;
    double x_size = 0.2 * size_multiplier;

    painter.setPen(QPen(Qt::cyan, 2));
    painter.drawLine(QPointF(sx - x_size, sy - x_size),
                     QPointF(sx + x_size, sy + x_size));
    painter.drawLine(QPointF(sx - x_size, sy + x_size),
                     QPointF(sx + x_size, sy - x_size));

    // Draw particle filter dots
    painter.setBrush(QColor(128, 128, 128, 80));
    painter.setPen(Qt::NoPen);
    double particle_radius = 0.07 * size_multiplier;
    for (const auto& p : particles) {
        double px = width() / 2.0 + p.x * size_multiplier;
        double py = height() / 2.0 - p.y * size_multiplier;
        painter.drawEllipse(QPointF(px, py), particle_radius, particle_radius);
    }
}