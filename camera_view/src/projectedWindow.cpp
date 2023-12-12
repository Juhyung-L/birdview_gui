#include <QVBoxLayout>

#include "camera_view/projectedWindow.hpp"
#include "camera_view/settings.hpp"

ProjectedWindow::ProjectedWindow(QWidget *parent)
: VideoWindow(parent),
projected_view(new VideoWidget(this)),
close_button(new QPushButton("Close", this)),
frame(resolutions::CAPTURE_RESOLUTION, CV_8UC3),
frame_undistorted(resolutions::CAPTURE_RESOLUTION, CV_8UC3)
{
    init_ui();
}

void ProjectedWindow::init_ui()
{
    this->setWindowTitle("Projected");

    QVBoxLayout *VBL = new QVBoxLayout(this);
    VBL->addWidget(projected_view);
    VBL->addWidget(close_button);
    VBL->setContentsMargins(0, 0, 0, 0);

    connect(close_button, &QPushButton::clicked, this, &ProjectedWindow::enable_close);
}

void ProjectedWindow::set_next_frame()
{
    bool ret = current_camera.img_buffer->read(frame);
    if (ret)
    {
        undistortion_camera.undistort(frame, frame_undistorted);
        if (current_camera.name == "left")
        {
            projection_camera.project_left(frame_undistorted, frame_projected);
        }
        else if (current_camera.name == "right")
        {
            projection_camera.project_right(frame_undistorted, frame_projected);
        }
        else // probably faster than doing else if (current_camera.name == "back")
        {
            projection_camera.project_back(frame_undistorted, frame_projected);
        }
        projected_view->set_frame(frame_projected);
        projected_view->update();
    }

    if (close_flag)
    {
        close_flag = false;
        this->close();
    }
}

void ProjectedWindow::start(const Camera& selected_camera)
{
    current_camera = selected_camera;
    undistortion_camera.load_data(current_camera);
    projection_camera.load_data(current_camera);

    if (current_camera.name == "left")
    {
        projected_view->setMinimumSize(projection::LEFT_PROJ_RESOLUTION.width, projection::LEFT_PROJ_RESOLUTION.height);
        frame_projected = cv::Mat(projection::LEFT_PROJ_RESOLUTION, CV_8UC3);
    }
    else if (current_camera.name == "right")
    {
        projected_view->setMinimumSize(projection::RIGHT_PROJ_RESOLUTION.width, projection::RIGHT_PROJ_RESOLUTION.height);
        frame_projected = cv::Mat(projection::RIGHT_PROJ_RESOLUTION, CV_8UC3);
    }
    else if (current_camera.name == "back")
    {
        projected_view->setMinimumSize(projection::BACK_PROJ_RESOLUTION.width, projection::BACK_PROJ_RESOLUTION.height);
        frame_projected = cv::Mat(projection::BACK_PROJ_RESOLUTION, CV_8UC3);
    }

    VideoWindow::start_timer();
}

void ProjectedWindow::closeEvent(QCloseEvent *event)
{
    VideoWindow::stop_timer();
}

void ProjectedWindow::enable_close()
{
    close_flag = true;
}

