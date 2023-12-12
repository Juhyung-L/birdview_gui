#ifndef UNDISTORTIONCAMERA_HPP_
#define UNDISTORTIONCAMERA_HPP_

#include <string>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/calib3d.hpp>

#include "camera_view/settings.hpp"
#include "camera_view/util.hpp"

class UndistortionCamera
{
private:
    cv::Mat undistort_mapx;
    cv::Mat undistort_mapy;
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    cv::Size resolution;
public:
    UndistortionCamera()
    {
    }

    void load_data(const Camera& camera)
    {
        cv::FileStorage fs(camera.undistortion_file, cv::FileStorage::READ);
        fs["camera_matrix"] >> camera_matrix;
        fs["dist_coeffs"] >> dist_coeffs;
        fs["resolution"] >> resolution;
        cv::Mat R = cv::Mat::eye(cv::Size(3, 3), CV_32FC1);

        cv::fisheye::initUndistortRectifyMap(camera_matrix,
                                             dist_coeffs,
                                             R,
                                             camera_matrix,
                                             resolution,
                                             CV_32FC1,
                                             undistort_mapx,
                                             undistort_mapy);
    }
    
    void undistort(const cv::Mat& frame, cv::Mat& frame_undistorted)
    {
        cv::remap(frame, frame_undistorted, undistort_mapx, undistort_mapy, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    }
};

#endif