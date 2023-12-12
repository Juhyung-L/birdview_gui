#ifndef PROJECTEDWINDOW_HPP_
#define PROJECTEDWUNDOW_HPP_

#include <QCloseEvent>
#include <QPushButton>

#include "camera_view/videoWindow.hpp"
#include "camera_view/videoWidget.hpp"
#include "camera_view/undistortionCamera.hpp"
#include "camera_view/projectionCamera.hpp"
#include "camera_view/util.hpp"

class ProjectedWindow : public VideoWindow
{
private:
    void init_ui();
    void set_next_frame() override;
    void enable_close();
    void closeEvent(QCloseEvent *event) override;

    bool close_flag = false;
    Camera current_camera;
    VideoWidget *projected_view;
    QPushButton *close_button;
    UndistortionCamera undistortion_camera;
    ProjectionCamera projection_camera;
    cv::Mat frame;
    cv::Mat frame_undistorted;
    cv::Mat frame_projected;

public:
    ProjectedWindow(QWidget *parent=nullptr);
    void start(const Camera& selected_camera);
};

#endif