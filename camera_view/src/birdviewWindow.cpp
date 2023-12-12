#include <QVBoxLayout>

#include "camera_view/birdviewWindow.hpp"
#include "camera_view/settings.hpp"

BirdviewWindow::BirdviewWindow(QWidget *parent)
: VideoWindow(parent),
birdview_view(new VideoWidget(this)),
close_button(new QPushButton("Close", this)),
frame(projection::BIRDVIEW_DISPLAY_RESOLUTION, CV_8UC3)
{
    init_ui();
}

void BirdviewWindow::init_ui()
{
    this->setWindowTitle("Birdview");

    QVBoxLayout *VBL = new QVBoxLayout(this);
    VBL->addWidget(birdview_view);
    VBL->addWidget(close_button);
    VBL->setContentsMargins(0, 0, 0, 0);

    connect(close_button, &QPushButton::clicked, this, &BirdviewWindow::enable_close);

    birdview_buffer = std::make_shared<BirdviewBuffer>(5);
    left_proc_thrd.attach(birdview_buffer);
    right_proc_thrd.attach(birdview_buffer);
    back_proc_thrd.attach(birdview_buffer);

    left_proc_thrd.load_camera(left_camera);
    right_proc_thrd.load_camera(right_camera);
    back_proc_thrd.load_camera(back_camera);

    birdview_view->setMinimumSize(projection::BIRDVIEW_DISPLAY_RESOLUTION.width, projection::BIRDVIEW_DISPLAY_RESOLUTION.height);
}

void BirdviewWindow::set_next_frame()
{
    bool ret = birdview_buffer->read(frame);
    if (ret)
    {
        birdview_view->set_frame(frame);
        birdview_view->update();
    }

    if (close_flag)
    {
        close_flag = false;
        this->close();
    }
}

void BirdviewWindow::enable_close()
{
    close_flag = true;
}

void BirdviewWindow::closeEvent(QCloseEvent *event)
{
    left_proc_thrd.stop();
    right_proc_thrd.stop();
    back_proc_thrd.stop();

    VideoWindow::stop_timer();
}

void BirdviewWindow::start()
{
    left_proc_thrd.start();
    right_proc_thrd.start();
    back_proc_thrd.start();
    VideoWindow::start_timer();
}