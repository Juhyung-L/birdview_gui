#ifndef PROCESSTHREAD_HPP_
#define PROCESSTHREAD_HPP_

#include <memory>
#include <condition_variable>
#include <mutex>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include <opencv2/core/mat.hpp>

#include "camera_view/undistortionCamera.hpp"
#include "camera_view/projectionCamera.hpp"
#include "camera_view/util.hpp"
#include "camera_view/birdviewBuffer.hpp"

class ProcessThread
{
public:
    ProcessThread()
    {
    }

    void load_camera(const Camera& camera)
    {
        camera_ = camera;
        undistortion_camera_.load_data(camera_);
        projection_camera_.load_data(camera_);
    }

    void start()
    {
        continue_thread_ = true;
        if (camera_.name == "left")
        {
            run_thrd = std::thread(&ProcessThread::left_run, this);
        }
        else if (camera_.name == "right")
        {
            run_thrd = std::thread(&ProcessThread::right_run, this);
        }
        else if (camera_.name == "back")
        {
            run_thrd = std::thread(&ProcessThread::back_run, this);
        }
    }

    void stop()
    {
        continue_thread_ = false;
        camera_.img_buffer->buffer_full_cv_.notify_one();
        birdview_buffer_->all_arrived_cv_.notify_one();
        run_thrd.join();
    }

    void attach(const std::shared_ptr<BirdviewBuffer>& birdview_buffer)
    {
        birdview_buffer->num_attached_buffers_++;
        birdview_buffer_ = birdview_buffer;
    }

private:
    void left_run()
    {
        while (continue_thread_)
        {
            std::unique_lock<std::mutex> lock(mtx_);
            camera_.img_buffer->buffer_full_cv_.wait(lock); // blocking call
            if (!continue_thread_) 
            {
                break;
            }

            if (camera_.img_buffer->read(frame_))
            {
                // undistort
                undistortion_camera_.undistort(frame_, frame_undistorted_);

                // project and crop
                projection_camera_.project_left(frame_undistorted_, frame_projected_);

                // write to buffer
                birdview_buffer_->syncAndWrite(frame_projected_, projection::LEFT_ORIGIN); // blocking call
                if (!continue_thread_) 
                {
                    break;
                }
            }
        }
    }

    void right_run()
    {
        while (continue_thread_)
        {
            std::unique_lock<std::mutex> lock(mtx_);
            camera_.img_buffer->buffer_full_cv_.wait(lock);
            if (!continue_thread_) 
            {
                break;
            }

            if (camera_.img_buffer->read(frame_))
            {
                // undistort
                undistortion_camera_.undistort(frame_, frame_undistorted_);

                // project and crop
                projection_camera_.project_right(frame_undistorted_, frame_projected_);

                // write to buffer
                birdview_buffer_->syncAndWrite(frame_projected_, projection::RIGHT_ORIGIN);
                if (!continue_thread_) 
                {
                    break;
                }
            }
        }
    }

    void back_run()
    {
        while (continue_thread_)
        {
            std::unique_lock<std::mutex> lock(mtx_);
            camera_.img_buffer->buffer_full_cv_.wait(lock);
            if (!continue_thread_) 
            {
                break;
            }

            if (camera_.img_buffer->read(frame_))
            {
                // undistort
                undistortion_camera_.undistort(frame_, frame_undistorted_);

                // project and crop
                projection_camera_.project_back(frame_undistorted_, frame_projected_);

                // write to buffer
                birdview_buffer_->syncAndWrite(frame_projected_, projection::BACK_ORIGIN);
                if (!continue_thread_) 
                {
                    break;
                }
            }
        }
    }

    cv::Mat frame_;
    cv::Mat frame_undistorted_;
    cv::Mat frame_projected_;
    bool continue_thread_ = true;
    Camera camera_;
    std::shared_ptr<BirdviewBuffer> birdview_buffer_;
    std::mutex mtx_;
    UndistortionCamera undistortion_camera_;
    ProjectionCamera projection_camera_;
    std::thread run_thrd;
};

#endif