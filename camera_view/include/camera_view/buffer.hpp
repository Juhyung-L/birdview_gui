#ifndef BUFFER_HPP_
#define BUFFER_HPP_

#include <mutex>
#include <queue>
#include <condition_variable>

#include <opencv2/opencv.hpp>

class Buffer
{
public:
    Buffer(int max_items) : max_items_(max_items)
    {
    }

    void write(const cv::Mat& img)
    {
        std::lock_guard<std::mutex> lock(mtx_); // lock to prevent read() from accessing data_
        if (data_.size() >= max_items_) // if buffer is full, throw out the oldest image
        {
            data_.pop();
        }
        data_.push(img);

        if (data_.size() >= max_items_)
        {
            buffer_full_cv_.notify_one();
        }
    }
    
    bool read(cv::Mat& img)
    {
        std::lock_guard<std::mutex> lock(mtx_); // lock to prevent write() from accessing data_
        if (data_.empty())
        {
            return false;
        }
        img = data_.front();
        data_.pop();
        return true;
    }

    std::condition_variable buffer_full_cv_;
    std::mutex mtx_;

private:
    std::queue<cv::Mat> data_;
    int max_items_;
};

#endif 