#ifndef UNDISTORTEDWINDOW_HPP_
#define UNDISTORTEDWINDOW_HPP_

#include <memory>
#include <string>

#include <QCloseEvent>
#include <QPushButton>
#include <opencv2/core.hpp>

#include "camera_view/videoWindow.hpp"
#include "camera_view/buffer.hpp"
#include "camera_view/videoWidget.hpp"
#include "camera_view/util.hpp"
#include "camera_view/undistortionCamera.hpp"

class UndistortedWindow : public VideoWindow
{
private:
    void init_ui();
    void set_next_frame() override;
    void enable_close();
    void closeEvent(QCloseEvent *event) override;
    
    bool close_flag = false;
    Camera current_camera;
    VideoWidget *undistorted_view;
    QPushButton *close_button;
    UndistortionCamera undistortion_camera;
    cv::Mat frame;
    cv::Mat frame_undistorted;
public:
    UndistortedWindow(QWidget *parent=nullptr);
    void start(const Camera& selected_camera);
};

#endif

