#ifndef UTIL_HPP_
#define UTIL_HPP_

#include <string>
#include <memory>

#include <opencv2/core/mat.hpp>

#include "camera_view/buffer.hpp"

struct Camera
{
    std::string name;
    std::string undistortion_file;
    std::string projection_file;
    std::shared_ptr<Buffer> img_buffer;
};

#endif