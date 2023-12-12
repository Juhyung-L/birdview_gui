#ifndef BIRDVIEWBUFFER_HPP_
#define BIRDVIEWBUFFER_HPP_

#include <mutex>
#include <queue>
#include <unordered_map>
#include <condition_variable>

#include <opencv2/core/mat.hpp>

#include "camera_view/settings.hpp"
#include "camera_view/util.hpp"

class BirdviewBuffer
{
public:
    BirdviewBuffer(int buffer_size) : counter_(0), buffer_size_(buffer_size),
    frame_original_(projection::BIRDVIEW_DISPLAY_RESOLUTION, CV_8UC3),
    frame_birdview_(projection::BIRDVIEW_DISPLAY_RESOLUTION, CV_8UC3)
    {
        // load frame_original_
        // make frame_original_ have a wheelchair picture at the center

        frame_original_.copyTo(frame_birdview_);
    }

    void syncAndWrite(const cv::Mat& frame, cv::Point2i origin)
    {
        std::unique_lock lock(mtx_);
        counter_++;

        // add the frame to frame_birdview_
        cv::Rect rect(origin.x, origin.y, frame.cols, frame.rows);
        cv::Mat roi = frame_birdview_(rect);
        cv::add(roi, frame, roi);

        if (counter_ >= num_attached_buffers_)
        {
            if (birdview_buffer_.size() >= buffer_size_)
            {
                birdview_buffer_.pop();
            }
            // push a copy of frame_birdview_
            cv::Mat frame_birdview_cpy;
            frame_birdview_.copyTo(frame_birdview_cpy);
            birdview_buffer_.push(frame_birdview_cpy);
            
            // reset frame_birdview_
            frame_original_.copyTo(frame_birdview_);
            all_arrived_cv_.notify_all();
            counter_ = 0;
        }
        else
        {
            all_arrived_cv_.wait(lock);
        }
    }

    bool read(cv::Mat& birdview_frame)
    {
        std::lock_guard lock(mtx_);
        if (birdview_buffer_.empty())
        {
            return false;
        }
        birdview_frame = birdview_buffer_.front();
        birdview_buffer_.pop();
        return true;
    }

    int num_attached_buffers_;
    std::condition_variable all_arrived_cv_;

private:
    std::queue<cv::Mat> birdview_buffer_;
    int counter_;
    std::mutex mtx_;
    int buffer_size_;
    cv::Mat frame_original_;
    cv::Mat frame_birdview_;
};

#endif