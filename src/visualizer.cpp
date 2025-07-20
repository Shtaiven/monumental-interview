#include "visualizer.h"

#include <QPainterPath>
#include <cmath>

Visualizer::Visualizer(QWidget *parent)
    : QWidget(parent), robot_x(0), robot_y(0), heading_dx(0), heading_dy(0) {
    setFixedSize(window_width, window_height);  // Set the window size
}

void Visualizer::updatePosition(double x, double y) {
    robot_x = width() / 2.0 + x * size_multiplier;
    robot_y = height() / 2.0 - y * size_multiplier;

    // Store previous position and compute movement vector
    static double last_x = x;
    static double last_y = y;
    static double last_dx = 1.0;  // Default to pointing right
    static double last_dy = 0.0;

    double dx = x - last_x;
    double dy = y - last_y;

    if (std::abs(dx) > 1e-6 || std::abs(dy) > 1e-6) {
        last_dx = dx;
        last_dy = dy;
        // Only add to trail if position changed
        trail.push_back(QPointF(robot_x, robot_y));
        if (trail.size() > 10) {
            trail.pop_front();
        }
    }
    heading_dx = last_dx;
    heading_dy = last_dy;

    last_x = x;
    last_y = y;

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

    // Draw heading indicator (yellow line in direction of movement)
    double heading_length = 0.4 * size_multiplier;
    double norm = std::sqrt(heading_dx * heading_dx + heading_dy * heading_dy);
    double hx = robot_x, hy = robot_y;
    double wx1 = robot_x, wy1 = robot_y, wx2 = robot_x, wy2 = robot_y;
    if (norm > 1e-6) {
        // Heading line
        hx += (heading_dx / norm) * heading_length;
        hy -= (heading_dy / norm) *
              heading_length;  // minus because screen y is inverted

        // Perpendicular vector for wheels
        double perp_dx = -heading_dy / norm;
        double perp_dy = heading_dx / norm;

        // Wheel positions
        wx1 += perp_dx * wheel_offset;
        wy1 -= perp_dy * wheel_offset;  // minus because screen y is inverted
        wx2 -= perp_dx * wheel_offset;
        wy2 += perp_dy * wheel_offset;
    } else {
        // Default wheel positions if no heading
        wx1 = robot_x;
        wy1 = robot_y - wheel_offset;
        wx2 = robot_x;
        wy2 = robot_y + wheel_offset;
    }

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
}