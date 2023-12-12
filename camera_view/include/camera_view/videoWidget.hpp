#ifndef VIDEOWIDGET_HPP_
#define VIDEOWIDGET_HPP_

#include <QOpenGLWidget>
#include <QImage>
#include <QPainter>
#include <opencv2/core/mat.hpp>

class VideoWidget : public QOpenGLWidget
{
private:
    QPainter painter;
    QImage q_frame;
    void paintEvent(QPaintEvent *event) override
    {
        painter.begin(this);
        painter.drawImage(0, 0, q_frame);
        painter.end();
    }
public:
    VideoWidget(QWidget *parent=nullptr)
    : QOpenGLWidget(parent),
    painter(QPainter(this))
    {
    }

    void set_frame(const cv::Mat &frame)
    {
        q_frame = QImage(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888).rgbSwapped();
    }
};

#endif
