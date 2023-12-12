#ifndef VIDEOWINDOW_HPP_
#define VIDEOWINDOW_HPP_

#include <QWidget>
#include <QTimer>

class VideoWindow : public QWidget
{
private:
    QTimer *timer;
    std::vector<QWidget*> observers;
    int rate;
    virtual void set_next_frame() // this function is overriden in its child classes
    {
    }

    void notify_start()
    {
        for (QWidget* observer : observers)
        {
            observer->setEnabled(false);
        }
    }

    void notify_end()
    {
        for (QWidget* observer : observers)
        {
            observer->setEnabled(true);
        }
    }
public:
    VideoWindow(QWidget *parent=nullptr)
    : QWidget(parent),
    timer(new QTimer(this))
    {
        timer->setTimerType(Qt::PreciseTimer);
        connect(timer, &QTimer::timeout, this, &VideoWindow::set_next_frame);
    }

    void set_frame_rate(int frame_rate)
    {
        rate = 1000 / frame_rate;
    }

   void start_timer()
    {
        notify_start();
        timer->start(rate);
        this->activateWindow();
        this->show();
    }

    void stop_timer()
    {
        timer->stop();
        notify_end();
    }

    void attach(QWidget *observer)
    {
        observers.push_back(observer);
    }
};

#endif
