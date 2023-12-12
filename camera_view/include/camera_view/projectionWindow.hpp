#ifndef PROJECTIONWINDOW_HPP_
#define PROJECTIONWINDOW_HPP_

#include <vector>

#include <QPushButton>
#include <QCloseEvent>
#include <QMouseEvent>
#include <QOpenGLWidget>
#include <opencv2/core.hpp>

#include "camera_view/videoWindow.hpp"
#include "camera_view/videoWidget.hpp"
#include "camera_view/undistortionCamera.hpp"
#include "camera_view/settings.hpp"
#include "camera_view/util.hpp"

class ProjectionWindow : public VideoWindow
{
    Q_OBJECT
signals:
    void projection_status(bool status);
private:
    void init_ui();
    void enable_close();
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void set_next_frame() override;
    void closeEvent(QCloseEvent *event) override;
    int is_point(int x, int y);
    void sort_points();
    void get_projection();

    bool close_flag = false;
    QPushButton *close_button;
    QPushButton *project_button;
    VideoWidget *projection_view;
    cv::Mat frame;
    cv::Mat frame_undistorted;
    Camera current_camera;
    UndistortionCamera undistortion_camera;
    int max_num_points = 4;
    int dot_radius = 5;
    std::vector<cv::Point2i> points;
public:
    ProjectionWindow(QWidget *parent=nullptr);
    void start(const Camera& selected_camera);
};

#endif