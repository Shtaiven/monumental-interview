#pragma once

#include <QPainter>
#include <QPainterPath>
#include <QPointF>
#include <QWidget>
#include <deque>

#include "types.h"

class Visualizer : public QWidget {
    Q_OBJECT

   public:
    explicit Visualizer(QWidget *parent = nullptr);
    void updatePosition(double x, double y);
    void setPathFunction(std::function<Vec2(double)> func, double start,
                         double stop, double step) {
        path_func = func;
        path_start = start;
        path_stop = stop;
        path_step = step;
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
    double robot_x;
    double robot_y;
    double heading_dx;
    double heading_dy;
    std::function<Vec2(double)> path_func = nullptr;
    double path_start;
    double path_stop;
    double path_step;
    std::deque<QPointF> trail;
};
