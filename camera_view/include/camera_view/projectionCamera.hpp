#ifndef PROJECTIONCAMERA_HPP_
#define PROJECTIONCAMERA_HPP_

#include <opencv2/core/mat.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/imgproc.hpp>

#include "camera_view/settings.hpp"
#include "camera_view/util.hpp"

class ProjectionCamera
{
private:
    cv::Mat proj_mat;
public:
    ProjectionCamera()
    {
    }

    void load_data(const Camera& camera)
    {
        cv::FileStorage fs(camera.projection_file, cv::FileStorage::READ);
        fs["proj_mat"] >> proj_mat;
    }

    void project_left(const cv::Mat& frame, cv::Mat& frame_projected)
    {
        cv::warpPerspective(frame, frame_projected, proj_mat, projection::LEFT_PROJ_RESOLUTION);
        cv::fillPoly(frame_projected, projection::LEFT_CONTOUR, cv::Scalar(0, 0, 0));
    }

    void project_right(const cv::Mat& frame, cv::Mat& frame_projected)
    {
        cv::warpPerspective(frame, frame_projected, proj_mat, projection::RIGHT_PROJ_RESOLUTION);
        cv::fillPoly(frame_projected, projection::RIGHT_CONTOUR, cv::Scalar(0, 0, 0));
    }

    void project_back(const cv::Mat& frame, cv::Mat& frame_projected)
    {
        cv::warpPerspective(frame, frame_projected, proj_mat, projection::BACK_PROJ_RESOLUTION);
        cv::fillPoly(frame_projected, projection::BACK_CONTOUR_L, cv::Scalar(0, 0, 0));
        cv::fillPoly(frame_projected, projection::BACK_CONTOUR_R, cv::Scalar(0, 0, 0));
    }
};

#endif