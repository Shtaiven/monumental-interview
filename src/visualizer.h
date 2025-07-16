#pragma once

#include <QPainter>
#include <QWidget>
#include <deque>
#include <QPointF>

class Visualizer : public QWidget {
    Q_OBJECT

   public:
    explicit Visualizer(QWidget *parent = nullptr);
    void updatePosition(double x, double y);

   protected:
    void paintEvent(QPaintEvent *event) override;

   private:
    double robotX;
    double robotY;
    double headingDX;
    double headingDY;
    std::deque<QPointF> trail; // Add this line
};
