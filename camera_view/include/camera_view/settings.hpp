#ifndef SETTINGS_HPP_
#define SETTINGS_HPP_

#include <filesystem>
#include <string>
#include <vector>
#include <unordered_map>

#include <opencv2/core/types.hpp>
#include <opencv2/core/persistence.hpp>

#include "camera_view/buffer.hpp"

namespace directories
{
    inline const std::filesystem::path YAML_DIR = "/home/dev_ws/src/camera_view/yaml";
    inline const std::string LEFT_DISTORTION_FILE = YAML_DIR.string() + "/distortion_left.yaml";
    inline const std::string RIGHT_DISTORTION_FILE = YAML_DIR.string() + "/distortion_right.yaml";
    inline const std::string BACK_DISTORTION_FILE = YAML_DIR.string() + "/distortion_back.yaml";
    
    inline const std::string LEFT_PROJECTION_FILE = YAML_DIR.string() + "/projection_left.yaml";
    inline const std::string RIGHT_PROJECTION_FILE = YAML_DIR.string() + "/projection_right.yaml";
    inline const std::string BACK_PROJECTION_FILE = YAML_DIR.string() + "/projection_back.yaml";

    inline void make_directory()
    {
        if (!std::filesystem::exists(YAML_DIR))
        {
            std::filesystem::create_directory(YAML_DIR);
        }
    }
    
    inline std::vector<std::string> undistortion_file_check()
    {
        std::vector<std::string> names;
        if (!std::filesystem::exists(std::filesystem::path(LEFT_DISTORTION_FILE)))
        {
            names.push_back("left");
        }
        if (!std::filesystem::exists(std::filesystem::path(RIGHT_DISTORTION_FILE)))
        {
            names.push_back("right");
        }
        if (!std::filesystem::exists(std::filesystem::path(BACK_DISTORTION_FILE)))
        {
            names.push_back("back");
        }
        return names;
    }

    inline std::vector<std::string> projection_file_check()
    {
        std::vector<std::string> names;
        if (!std::filesystem::exists(std::filesystem::path(LEFT_PROJECTION_FILE)))
        {
            names.push_back("left");
        }
        if (!std::filesystem::exists(std::filesystem::path(RIGHT_PROJECTION_FILE)))
        {
            names.push_back("right");
        }
        if (!std::filesystem::exists(std::filesystem::path(BACK_PROJECTION_FILE)))
        {
            names.push_back("back");
        }
        return names;
    }
}

namespace resolutions
{
    inline const std::string resolutions_file = "/home/dev_ws/src/camera_view/yaml/resolutions.yaml";

    inline cv::Size CAPTURE_RESOLUTION;
    
    inline void load_resolutions()
    {
        cv::FileStorage fs(resolutions_file, cv::FileStorage::READ);
        fs["capture_resolution"] >> CAPTURE_RESOLUTION;
    }
}

namespace projection
{
    inline const std::string projection_board_file = "/home/dev_ws/src/camera_view/yaml/project_board.yaml";
    inline std::unordered_map<std::string, std::unordered_map<std::string, cv::Point2i>> proj_pts;
    inline cv::Size LEFT_PROJ_RESOLUTION;
    inline cv::Size RIGHT_PROJ_RESOLUTION;
    inline cv::Size BACK_PROJ_RESOLUTION;
    inline cv::Point2i LEFT_ORIGIN, RIGHT_ORIGIN, BACK_ORIGIN;
    inline std::vector<cv::Point2i> LEFT_CONTOUR;
    inline std::vector<cv::Point2i> RIGHT_CONTOUR;
    inline std::vector<cv::Point2i> BACK_CONTOUR_L;
    inline std::vector<cv::Point2i> BACK_CONTOUR_R;
    inline cv::Size BIRDVIEW_DISPLAY_RESOLUTION;

    inline void load_projection_pts()
    {
        cv::FileStorage fs(projection_board_file, cv::FileStorage::READ);
        cv::Size display, board, outer_shift;
        fs["display"] >> display;
        fs["board"] >> board;

        outer_shift.width = (display.width - board.height - board.width - board.height) / 2;
        outer_shift.height = display.height - board.width - board.height;

        BIRDVIEW_DISPLAY_RESOLUTION = display;

        LEFT_PROJ_RESOLUTION = cv::Size(outer_shift.width + board.height, display.height);
        RIGHT_PROJ_RESOLUTION = LEFT_PROJ_RESOLUTION;
        BACK_PROJ_RESOLUTION = cv::Size(display.width, outer_shift.height + board.height);

        proj_pts["left"]["top_left"] = cv::Point2i(outer_shift.width, board.width);
        proj_pts["left"]["top_right"] = cv::Point2i(outer_shift.width, 0);
        proj_pts["left"]["bottom_right"] = cv::Point2i(outer_shift.width + board.height, 0);
        proj_pts["left"]["bottom_left"] = cv::Point2i(outer_shift.width + board.height, board.width);

        proj_pts["right"]["top_left"] = cv::Point2i(board.height, 0);
        proj_pts["right"]["top_right"] = cv::Point2i(board.height, board.width);
        proj_pts["right"]["bottom_right"] = cv::Point2i(0, board.width);
        proj_pts["right"]["bottom_left"] = cv::Point2i(0, 0);

        proj_pts["back"]["top_left"] = cv::Point2i(outer_shift.width + board.height + board.width, board.height);
        proj_pts["back"]["top_right"] = cv::Point2i(outer_shift.width + board.height, board.height);
        proj_pts["back"]["bottom_right"] = cv::Point2i(outer_shift.width + board.height, 0);
        proj_pts["back"]["bottom_left"] = cv::Point2i(outer_shift.width + board.height + board.width, 0);
    
        LEFT_ORIGIN = cv::Point2i(0, 0);
        RIGHT_ORIGIN = cv::Point2i(outer_shift.width + board.height + board.width, 0);
        BACK_ORIGIN = cv::Point2i(0, board.width);

        LEFT_CONTOUR.emplace_back(LEFT_PROJ_RESOLUTION.width, board.width);
        LEFT_CONTOUR.emplace_back(0, LEFT_PROJ_RESOLUTION.height);
        LEFT_CONTOUR.emplace_back(LEFT_PROJ_RESOLUTION.width, LEFT_PROJ_RESOLUTION.height);

        RIGHT_CONTOUR.emplace_back(0, board.width);
        RIGHT_CONTOUR.emplace_back(0, RIGHT_PROJ_RESOLUTION.height);
        RIGHT_CONTOUR.emplace_back(RIGHT_PROJ_RESOLUTION.width, RIGHT_PROJ_RESOLUTION.height);

        BACK_CONTOUR_L.emplace_back(outer_shift.width + board.height, 0);
        BACK_CONTOUR_L.emplace_back(0, 0);
        BACK_CONTOUR_L.emplace_back(0, BACK_PROJ_RESOLUTION.height);

        BACK_CONTOUR_R.emplace_back(outer_shift.width + board.height + board.width, 0);
        BACK_CONTOUR_R.emplace_back(BACK_PROJ_RESOLUTION.width, 0);
        BACK_CONTOUR_R.emplace_back(BACK_PROJ_RESOLUTION.width, BACK_PROJ_RESOLUTION.height);
    }
}

#endif