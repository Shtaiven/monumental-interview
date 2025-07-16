#include "visualizer.h"

Visualizer::Visualizer(QWidget *parent)
    : QWidget(parent), robotX(0), robotY(0) {
    setFixedSize(800, 600);  // Set the window size
}

void Visualizer::updatePosition(double x, double y) {
    // Scale GPS coordinates to fit the window (adjust scaling as needed)
    robotX = 400 + x * 10.0;  // Centered at (400, 300)
    robotY = 300 - y * 10.0;

    // Trigger a repaint
    update();
}

void Visualizer::paintEvent(QPaintEvent *event) {
    QPainter painter(this);

    // Clear the background
    painter.fillRect(rect(), Qt::black);

    // Draw the robot as a red circle
    painter.setBrush(Qt::red);
    painter.drawEllipse(QPointF(robotX, robotY), 10, 10);  // Radius = 10
}