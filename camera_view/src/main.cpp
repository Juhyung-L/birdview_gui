#include <thread>
#include <chrono>
#include <iostream>

#include <QApplication>
#include <opencv2/calib3d.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

#include "camera_view/externVariables.hpp"
#include "camera_view/mainWindow.hpp"
#include "camera_view/util.hpp"
#include "camera_view/settings.hpp"

using namespace std::placeholders;

Camera right_camera;
Camera left_camera;
Camera back_camera;

void left_camera_callback(const sensor_msgs::msg::Image& img)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img);
    left_camera.img_buffer->write(cv_ptr->image);
}

void right_camera_callback(const sensor_msgs::msg::Image& img)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img);
    right_camera.img_buffer->write(cv_ptr->image);
}

void back_camera_callback(const sensor_msgs::msg::Image& img)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img);
    back_camera.img_buffer->write(cv_ptr->image);
}

int main(int argc, char *argv[])
{
    left_camera.img_buffer = std::make_shared<Buffer>(5);
    left_camera.name = "left";
    left_camera.undistortion_file = directories::LEFT_DISTORTION_FILE;
    left_camera.projection_file = directories::LEFT_PROJECTION_FILE;
    
    right_camera.img_buffer = std::make_shared<Buffer>(5);
    right_camera.name = "right";
    right_camera.undistortion_file = directories::RIGHT_DISTORTION_FILE;
    right_camera.projection_file = directories::RIGHT_PROJECTION_FILE;

    back_camera.img_buffer = std::make_shared<Buffer>(5);
    back_camera.name = "back";
    back_camera.undistortion_file = directories::BACK_DISTORTION_FILE;
    back_camera.projection_file = directories::BACK_PROJECTION_FILE;

    rclcpp::init(argc, argv);
    QApplication q_app(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("camera_display");

    // left camera
    auto left_cam_cb_grp = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions left_cam_options;
    left_cam_options.callback_group = left_cam_cb_grp;
    auto left_cam_sub = node->create_subscription<sensor_msgs::msg::Image>(
        "/camera_left/image_raw", rclcpp::SensorDataQoS(), std::bind(&left_camera_callback, _1), left_cam_options);

    // right camera
    auto right_cam_cb_grp = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions right_cam_options;
    right_cam_options.callback_group = right_cam_cb_grp;
    auto right_cam_sub = node->create_subscription<sensor_msgs::msg::Image>(
        "/camera_right/image_raw", rclcpp::SensorDataQoS(), std::bind(&right_camera_callback, _1), right_cam_options);

    // back camera
    auto back_cam_cb_grp = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions back_cam_options;
    back_cam_options.callback_group = back_cam_cb_grp;
    auto back_cam_sub = node->create_subscription<sensor_msgs::msg::Image>(
        "/camera_back/image_raw", rclcpp::SensorDataQoS(), std::bind(&back_camera_callback, _1), back_cam_options);
    
    std::thread node_spin_thrd([&]
        {
            rclcpp::executors::MultiThreadedExecutor executor;
            executor.add_node(node);
            executor.spin();
        }
    );

    MainWindow w;
    q_app.exec();

    rclcpp::shutdown();
    node_spin_thrd.join();

    return 0;
}