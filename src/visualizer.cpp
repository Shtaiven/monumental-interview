#include "visualizer.h"

#include <cmath>

Visualizer::Visualizer(QWidget *parent)
    : QWidget(parent), robotX(0), robotY(0), headingDX(0), headingDY(0) {
    setFixedSize(800, 600);  // Set the window size
}

void Visualizer::updatePosition(double x, double y) {
    robotX = 400 + x * 10.0;
    robotY = 300 - y * 10.0;

    // Store previous position and compute movement vector
    static double lastX = x;
    static double lastY = y;
    static double lastDX = 1.0;  // Default to pointing right
    static double lastDY = 0.0;

    double dx = x - lastX;
    double dy = y - lastY;

    if (std::abs(dx) > 1e-6 || std::abs(dy) > 1e-6) {
        lastDX = dx;
        lastDY = dy;
        // Only add to trail if position changed
        trail.push_back(QPointF(robotX, robotY));
        if (trail.size() > 10) {
            trail.pop_front();
        }
    }
    headingDX = lastDX;
    headingDY = lastDY;

    lastX = x;
    lastY = y;

    update();
}

void Visualizer::paintEvent(QPaintEvent *event) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // Clear the background
    painter.fillRect(rect(), Qt::black);

    // Draw the robot body
    painter.setPen(Qt::NoPen);
    painter.setBrush(Qt::red);
    painter.drawEllipse(QPointF(robotX, robotY), 10, 10);

    double wheelOffset = 12.0;  // distance from center to wheel
    double wheelRadius = 4.0;

    // Draw heading indicator (yellow line in direction of movement)
    double headingLength = 18.0;
    double norm = std::sqrt(headingDX * headingDX + headingDY * headingDY);
    double hx = robotX, hy = robotY;
    double wx1 = robotX, wy1 = robotY, wx2 = robotX, wy2 = robotY;
    if (norm > 1e-6) {
        // Heading line
        hx += (headingDX / norm) * headingLength;
        hy -= (headingDY / norm) *
              headingLength;  // minus because screen y is inverted

        // Perpendicular vector for wheels
        double perpDX = -headingDY / norm;
        double perpDY = headingDX / norm;

        // Wheel positions
        wx1 += perpDX * wheelOffset;
        wy1 -= perpDY * wheelOffset;  // minus because screen y is inverted
        wx2 -= perpDX * wheelOffset;
        wy2 += perpDY * wheelOffset;
    } else {
        // Default wheel positions if no heading
        wx1 = robotX;
        wy1 = robotY - wheelOffset;
        wx2 = robotX;
        wy2 = robotY + wheelOffset;
    }

    // Draw wheels
    painter.setBrush(Qt::gray);
    painter.drawEllipse(QPointF(wx1, wy1), wheelRadius, wheelRadius);
    painter.drawEllipse(QPointF(wx2, wy2), wheelRadius, wheelRadius);

    // Draw heading line
    painter.setPen(QPen(Qt::yellow, 2));
    painter.drawLine(QPointF(robotX, robotY), QPointF(hx, hy));

    // Draw the trail above everything else
    if (trail.size() > 1) {
        painter.setPen(QPen(Qt::green, 2));
        painter.setBrush(Qt::NoBrush);  // No fill for trail
        for (size_t i = 1; i < trail.size(); ++i) {
            painter.drawLine(trail[i - 1], trail[i]);
        }
    }
}