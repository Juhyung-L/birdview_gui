#include <QVBoxLayout>
#include <opencv2/calib3d.hpp>

#include "camera_view/undistortedWindow.hpp"
#include "camera_view/settings.hpp"

UndistortedWindow::UndistortedWindow(QWidget *parent)
: VideoWindow(parent),
undistorted_view(new VideoWidget(this)),
close_button(new QPushButton("Close", this)),
frame(resolutions::CAPTURE_RESOLUTION, CV_8UC3),
frame_undistorted(resolutions::CAPTURE_RESOLUTION, CV_8UC3)
{
    init_ui();
}

void UndistortedWindow::init_ui()
{
    this->setWindowTitle("Undistorted");

    QVBoxLayout *VBL = new QVBoxLayout(this);
    VBL->addWidget(undistorted_view);
    VBL->addWidget(close_button);
    VBL->setContentsMargins(0, 0, 0, 0);

    connect(close_button, &QPushButton::clicked, this, &UndistortedWindow::enable_close);

    undistorted_view->setMinimumSize(resolutions::CAPTURE_RESOLUTION.width, resolutions::CAPTURE_RESOLUTION.height);
}

void UndistortedWindow::set_next_frame()
{
    bool ret = current_camera.img_buffer->read(frame);
    if (ret)
    {
        undistortion_camera.undistort(frame, frame_undistorted);
        undistorted_view->set_frame(frame_undistorted);
        undistorted_view->update();
    }

    if (close_flag)
    {
        close_flag = false;
        this->close();
    }
}

void UndistortedWindow::start(const Camera& selected_camera)
{
    current_camera = selected_camera;
    undistortion_camera.load_data(current_camera);
    VideoWindow::start_timer();
}

void UndistortedWindow::closeEvent(QCloseEvent *event)
{
    VideoWindow::stop_timer();
}

void UndistortedWindow::enable_close()
{
    close_flag = true;
}