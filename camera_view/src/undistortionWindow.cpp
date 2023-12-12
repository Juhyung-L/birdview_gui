#include <QVBoxLayout>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "camera_view/undistortionWindow.hpp"
#include "camera_view/settings.hpp"

UndistortionWindow::UndistortionWindow(QWidget *parent)
    : VideoWindow(parent),
    num_checkerboards_found(new QLabel(this)),
    undistortion_view(new VideoWidget(this)),
    undistort_button(new QPushButton("Undistort", this)),
    close_button(new QPushButton("Close", this)),
    frame(resolutions::CAPTURE_RESOLUTION, CV_8UC3),
    gray(resolutions::CAPTURE_RESOLUTION, CV_8U),
    close_flag(false),
    CHESSBOARD_SIZE(6, 9),
    num_frames(100)
{
    for (int i{0}; i<CHESSBOARD_SIZE.height; ++i)
    {
        for (int j{0}; j<CHESSBOARD_SIZE.width; ++j)
        {
            objp.push_back(cv::Point3f(j,i,0));
        }
    }
    init_ui();
}

void UndistortionWindow::init_ui()
{
    this->setWindowTitle("Undistortion");

    QVBoxLayout *VBL = new QVBoxLayout(this);
    VBL->addWidget(num_checkerboards_found, 0, Qt::AlignCenter);
    VBL->addWidget(undistortion_view);
    VBL->addWidget(undistort_button);
    VBL->addWidget(close_button);
    VBL->setContentsMargins(0, 0, 0, 0);

    connect(undistort_button, &QPushButton::clicked, this, &UndistortionWindow::enable_undistortion);
    connect(close_button, &QPushButton::clicked, this, &UndistortionWindow::enable_close);

    undistortion_view->setMinimumSize(resolutions::CAPTURE_RESOLUTION.width, resolutions::CAPTURE_RESOLUTION.height);
}

void UndistortionWindow::reset_vars()
{
    objpoints.clear();
    imgpoints.clear();
    corner_pts.clear();

    num_checkerboards_found->setText("");
}

void UndistortionWindow::set_next_frame()
{
    if (!undistort_flag)
    {
        bool found = false;
        bool ret = current_camera.img_buffer->read(frame);
        if (ret)
        {
           cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
           found = cv::findChessboardCorners(gray,
                                             CHESSBOARD_SIZE,
                                             corner_pts,
                                             cv::CALIB_CB_ADAPTIVE_THRESH |
                                             cv::CALIB_CB_FAST_CHECK |
                                             cv::CALIB_CB_NORMALIZE_IMAGE);
            if (found)
            {
                cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.1);
                cv::cornerSubPix(gray,corner_pts, cv::Size(3,3), cv::Size(-1,-1), criteria);
                cv::drawChessboardCorners(frame, CHESSBOARD_SIZE, corner_pts, found);
                objpoints.push_back(objp);
                imgpoints.push_back(corner_pts);
            }

            num_checkerboards_found->setText(QString::number(objpoints.size()) + " images captured");

            undistortion_view->set_frame(frame);
            undistortion_view->update();
        }
    }
    else
    {
        undistort_flag = false;
        cv::Mat camera_matrix, dist_coeffs;
        std::vector<cv::Mat> rvecs, tvecs;
        int flag = cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC | cv::fisheye::CALIB_FIX_SKEW;
        cv::TermCriteria criteria((cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER), 30, 1e-6);
        try
        {
            cv::fisheye::calibrate(objpoints,
                                   imgpoints,
                                   resolutions::CAPTURE_RESOLUTION,
                                   camera_matrix,
                                   dist_coeffs,
                                   rvecs,
                                   tvecs,
                                   flag,
                                   criteria);
        }
        catch (cv::Exception e)
        {
            emit undistortion_status(false);
            this->close();
            return;
        }

        cv::FileStorage fs(current_camera.undistortion_file, cv::FileStorage::WRITE);
        fs << "camera_matrix" << camera_matrix;
        fs << "dist_coeffs" << dist_coeffs;
        fs << "resolution" << resolutions::CAPTURE_RESOLUTION;
        fs.release();
        emit undistortion_status(true);
        this->close();
    }

    if (close_flag)
    {
        close_flag = false;
        this->close();
    }
}

void UndistortionWindow::start(const Camera& selected_camera)
{
    current_camera = selected_camera;
    reset_vars();
    VideoWindow::start_timer();
}

void UndistortionWindow::closeEvent(QCloseEvent *event)
{
    VideoWindow::stop_timer();
}

void UndistortionWindow::enable_close()
{
    close_flag = true;
}

void UndistortionWindow::enable_undistortion()
{
    undistort_flag = true;
}
