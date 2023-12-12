#ifndef BIRDVIEWWINDOW_HPP_
#define BIRDVIEWWINDOW_HPP_

#include <memory>
#include <thread>

#include <QWidget>
#include <QCloseEvent>
#include <QPushButton>
#include <opencv2/core.hpp>

#include "camera_view/birdviewBuffer.hpp"
#include "camera_view/processThread.hpp"
#include "camera_view/videoWindow.hpp"
#include "camera_view/videoWidget.hpp"
#include "camera_view/util.hpp"
#include "camera_view/externVariables.hpp"

class BirdviewWindow : public VideoWindow
{
private:
    void init_ui();
    void enable_close();
    void set_next_frame() override;
    void closeEvent(QCloseEvent *event);

    bool close_flag = false;
    QPushButton *close_button;
    cv::Mat frame;
    ProcessThread left_proc_thrd;
    ProcessThread right_proc_thrd;
    ProcessThread back_proc_thrd;
    std::shared_ptr<BirdviewBuffer> birdview_buffer;
    VideoWidget *birdview_view;
public:
    BirdviewWindow(QWidget* parent=nullptr);
    void start();
};

#endif