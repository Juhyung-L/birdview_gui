#include <cmath>

#include <QVBoxLayout>
#include <opencv2/core/persistence.hpp>

#include "camera_view/projectionWindow.hpp"

ProjectionWindow::ProjectionWindow(QWidget *parent)
: VideoWindow(parent),
projection_view(new VideoWidget(this)),
close_button(new QPushButton("Close", this)),
project_button(new QPushButton("Project", this)),
frame(resolutions::CAPTURE_RESOLUTION, CV_8UC3),
frame_undistorted(resolutions::CAPTURE_RESOLUTION, CV_8UC3)
{
    init_ui();
}

void ProjectionWindow::init_ui()
{
    this->setWindowTitle("Projection");

    QVBoxLayout *VBL = new QVBoxLayout(this);
    VBL->addWidget(projection_view);
    VBL->addWidget(project_button);
    VBL->addWidget(close_button);
    VBL->setContentsMargins(0, 0, 0, 0);

    connect(close_button, &QPushButton::clicked, this, &ProjectionWindow::enable_close);
    connect(project_button, &QPushButton::clicked, this, &ProjectionWindow::get_projection);

    projection_view->setMinimumSize(resolutions::CAPTURE_RESOLUTION.width, resolutions::CAPTURE_RESOLUTION.height);
}

// determine if the input pixel position is inside any existing point
// and return the index of the point in the vector points
int ProjectionWindow::is_point(int x, int y)
{
    for (int i=0; i<points.size(); ++i)
    {
        if (abs(points[i].x - x) <= dot_radius && abs(points[i].y - y) <= dot_radius)
        {
            return i;
        }
    }
    return -1;
}

// sort in order:
// top_left -> top_right -> bottom_right -> bottom_left
void ProjectionWindow::sort_points()
{
    // sort into left and right groups
    std:sort(points.begin(), points.end(), [](const cv::Point2i& a, const cv::Point2i& b)
        {
            return a.y < b.y;
        }
    );
    // sort left group
    if (points[0].x > points[1].x)
    {
        auto tmp = points[1];
        points[1] = points[0];
        points[0] = tmp;
    }
    // sort right group
    if (points[2].x < points[3].x)
    {
        auto tmp = points[3];
        points[3] = points[2];
        points[2] = tmp;
    }
}

void ProjectionWindow::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {
        // check if cursor is on an existing point
        if (is_point(event->pos().x(), event->pos().y()) != -1)
        {
            return;
        }

        if (points.size() < max_num_points)
        {
            points.emplace_back(event->pos().x(), event->pos().y());
        }
    }
}

void ProjectionWindow::mouseMoveEvent(QMouseEvent *event)
{
    int i = is_point(event->pos().x(), event->pos().y());
    if (i != -1)
    {
        points[i].x = event->pos().x();
        points[i].y = event->pos().y();
    }
}

void ProjectionWindow::set_next_frame()
{
    bool ret = current_camera.img_buffer->read(frame);
    if (ret)
    {
        // don't resize because it messes up the projection
        undistortion_camera.undistort(frame, frame_undistorted);
        
        // draw points
        for (auto& point : points)
        {
            cv::circle(frame_undistorted, point, dot_radius, cv::Scalar(0, 0, 255), -1);
        }

        // draw quadrilateral
        if (points.size() == max_num_points)
        {
            sort_points();
            cv::polylines(frame_undistorted, points, true, cv::Scalar(0, 255, 0));
        }
        
        projection_view->set_frame(frame_undistorted);
        projection_view->update();
    }
    
    if (close_flag)
    {
        close_flag=false;
        this->close();
    }
}

void ProjectionWindow::get_projection()
{
    // the cv::getPerspectiveTransform() needs 32-bit floating point numbers for some reason
    std::vector<cv::Point2f> dest_pts;
    std::vector<cv::Point2f> src_pts;
    cv::Mat proj_mat;

    for (auto& point : points)
    {
        src_pts.push_back(cv::Point2f(point));
    }

    dest_pts.push_back(cv::Point2f(projection::proj_pts[current_camera.name]["top_left"]));
    dest_pts.push_back(cv::Point2f(projection::proj_pts[current_camera.name]["top_right"]));
    dest_pts.push_back(cv::Point2f(projection::proj_pts[current_camera.name]["bottom_right"]));
    dest_pts.push_back(cv::Point2f(projection::proj_pts[current_camera.name]["bottom_left"]));

    try
    {
        proj_mat = cv::getPerspectiveTransform(src_pts, dest_pts);
    }
    catch(const cv::Exception& e)
    {
        std::cout << e.msg << std::endl;
        emit projection_status(false);
        this->close();
        return;
    }

    cv::FileStorage fs(current_camera.projection_file, cv::FileStorage::WRITE);
    fs << "proj_mat" << proj_mat;
    fs.release();
    emit projection_status(true);
    this->close();
}

void ProjectionWindow::enable_close()
{
    close_flag = true;
}

void ProjectionWindow::closeEvent(QCloseEvent *event)
{
    VideoWindow::stop_timer();
}

void ProjectionWindow::start(const Camera& selected_camera)
{
    points.clear();
    current_camera = selected_camera;
    undistortion_camera.load_data(current_camera);
    VideoWindow::start_timer();
}