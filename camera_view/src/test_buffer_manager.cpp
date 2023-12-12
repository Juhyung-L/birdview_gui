#include <thread>
#include <chrono>
#include <iostream>

#include <opencv2/calib3d.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

#include "camera_view/externVariables.hpp"
#include "camera_view/captureThread.hpp"
#include "camera_view/util.hpp"
#include "camera_view/bufferManager.hpp"

using namespace std::placeholders;

Camera right_camera;
Camera left_camera;
Camera back_camera;

void right_camera_callback(const sensor_msgs::msg::Image& img)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img); // copy the image to cv_ptr (sharing prevents modification to image)
    right_camera.img_buffer->write(cv_ptr->image);
}

void left_camera_callback(const sensor_msgs::msg::Image& img)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img);
    left_camera.img_buffer->write(cv_ptr->image);
}

void back_camera_callback(const sensor_msgs::msg::Image& img)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img);
    back_camera.img_buffer->write(cv_ptr->image);
}

int main(int argc, char *argv[])
{
    right_camera.img_buffer = std::make_shared<Buffer>(5);
    right_camera.name = "right";

    left_camera.img_buffer = std::make_shared<Buffer>(5);
    left_camera.name = "left";

    back_camera.img_buffer = std::make_shared<Buffer>(5);
    back_camera.name = "back";


    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("camera_display");

    // right camera
    auto right_cam_cb_grp = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions right_cam_options;
    right_cam_options.callback_group = right_cam_cb_grp;
    auto right_cam_sub = node->create_subscription<sensor_msgs::msg::Image>(
        "/camera_right/image_raw", rclcpp::SensorDataQoS(), std::bind(&right_camera_callback, _1), right_cam_options);

    // left camera
    auto left_cam_cb_grp = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions left_cam_options;
    left_cam_options.callback_group = left_cam_cb_grp;
    auto left_cam_sub = node->create_subscription<sensor_msgs::msg::Image>(
        "/camera_left/image_raw", rclcpp::SensorDataQoS(), std::bind(&left_camera_callback, _1), left_cam_options);

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

    CaptureThread right_cam_cap_thrd = CaptureThread(right_camera);
    CaptureThread left_cam_cap_thrd = CaptureThread(left_camera);
    CaptureThread back_cam_cap_thrd = CaptureThread(back_camera);

    std::shared_ptr<BufferManager> buffer_manager = std::make_shared<BufferManager>(5);
    right_cam_cap_thrd.attachBufferManager(buffer_manager);
    left_cam_cap_thrd.attachBufferManager(buffer_manager);
    back_cam_cap_thrd.attachBufferManager(buffer_manager);

    std::thread right_cam_run([&]
        {
            right_cam_cap_thrd.start();
        }
    );
    std::thread left_cam_run([&]
        {
            left_cam_cap_thrd.start();
        }
    );
    std::thread back_cam_run([&]
        {
            back_cam_cap_thrd.start();
        }
    );

    cv::Mat all_frames;
    cv::Mat tmp;
    std::unordered_map<std::string, cv::Mat> all_imgs;
    while (rclcpp::ok())
    {
        if (buffer_manager->read(all_imgs))
        {
            cv::hconcat(all_imgs["right"], all_imgs["back"], tmp);
            cv::hconcat(tmp, all_imgs["left"], all_frames);
            cv::imshow("all_frames", all_frames);
            cv::waitKey(1);
        }

        usleep(1000000.0 / 30.0);
    }
    rclcpp::shutdown();
    
    node_spin_thrd.join();
    right_cam_cap_thrd.stop();
    left_cam_cap_thrd.stop();
    back_cam_cap_thrd.stop();
    
    right_cam_run.join();
    left_cam_run.join();
    back_cam_run.join();

    return 0;
}