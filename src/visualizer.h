#pragma once

#include <QPainter>
#include <QPainterPath>
#include <QPointF>
#include <QWidget>
#include <deque>

#include "types.h"
#include "robot_model.h"

class Visualizer : public QWidget {
    Q_OBJECT

   public:
    explicit Visualizer(QWidget *parent = nullptr);
    void updateRobotModel(const robot_model::RobotModel &model);
    void setPathFunction(std::function<Vec2(double)> func, double start,
                         double stop, double step) {
        path_func = func;
        path_start = start;
        path_stop = stop;
        path_step = step;
    }
    void setSetpoint(double x, double y) {
        setpoint_x = x;
        setpoint_y = y;
    }

   protected:
    void paintEvent(QPaintEvent *event) override;
    QPainterPath drawPath(std::function<Vec2(double)> func, double start,
                          double stop, double step);

   private:
    const double size_multiplier = 80;
    const double window_width = 800;
    const double window_height = 800;
    QPainter painter;
    double robot_x = 0;
    double robot_y = 0;
    double heading_dx = 0;
    double heading_dy = 0;
    double setpoint_x = 0;
    double setpoint_y = 0;
    std::function<Vec2(double)> path_func = nullptr;
    double path_start;
    double path_stop;
    double path_step;
    std::deque<QPointF> trail;
};
