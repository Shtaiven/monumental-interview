#pragma once

#include <QPainter>
#include <QPainterPath>
#include <QPointF>
#include <QWidget>
#include <deque>

#include "path_generator.h"

class Visualizer : public QWidget {
    Q_OBJECT

   public:
    explicit Visualizer(QWidget *parent = nullptr);
    void updatePosition(double x, double y);
    void setPathFunction(std::function<path_generator::Point(double)> func,
                         double start, double stop, double step) {
        path_func = func;
        path_start = start;
        path_stop = stop;
        path_step = step;
    }

   protected:
    void paintEvent(QPaintEvent *event) override;
    QPainterPath drawPath(std::function<path_generator::Point(double)> func,
                          double start, double stop, double step);

   private:
    QPainter painter;
    double robot_x;
    double robot_y;
    double heading_dx;
    double heading_dy;
    std::function<path_generator::Point(double)> path_func = NULL;
    double path_start;
    double path_stop;
    double path_step;
    std::deque<QPointF> trail;
};
