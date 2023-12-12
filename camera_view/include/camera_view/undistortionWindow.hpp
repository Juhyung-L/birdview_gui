#ifndef UNDISTORTIONWINDOW_HPP_
#define UNDISTORTIONWINDOW_HPP_

#include <memory>
#include <vector>
#include <string>

#include <QPushButton>
#include <QCloseEvent>
#include <QLabel>
#include <opencv2/core.hpp>

#include "camera_view/videoWidget.hpp"
#include "camera_view/videoWindow.hpp"
#include "camera_view/util.hpp"

class UndistortionWindow : public VideoWindow
{
    Q_OBJECT
signals:
    void undistortion_status(bool status);
private:
    void init_ui();
    void reset_vars();
    void set_next_frame() override;
    void enable_undistortion();
    void enable_close();
    void closeEvent(QCloseEvent *evnet) override;
    
    bool close_flag = false;
    bool undistort_flag = false;
    Camera current_camera;
    QLabel *num_checkerboards_found;
    VideoWidget *undistortion_view;
    QPushButton *undistort_button;
    QPushButton *close_button;
    cv::Mat frame;
    cv::Mat gray;
    
    // variables for undistortion
    int num_frames;
    cv::Size CHESSBOARD_SIZE;
    std::vector<std::vector<cv::Point3f>> objpoints;
    std::vector<std::vector<cv::Point2f> > imgpoints;
    std::vector<cv::Point3f> objp;
    std::vector<cv::Point2f> corner_pts;
public:
    UndistortionWindow(QWidget *parent=nullptr);
    void start(const Camera& selected_camera);
};

#endif
