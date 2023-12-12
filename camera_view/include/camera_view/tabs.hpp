#ifndef TABS_HPP_
#define TABS_HPP_

#include <QWidget>
#include <QPushButton>
#include <QCheckBox>
#include <QComboBox>
#include <QLabel>

#include "camera_view/undistortionWindow.hpp"
#include "camera_view/undistortedWindow.hpp"
#include "camera_view/projectionWindow.hpp"
#include "camera_view/projectedWindow.hpp"
#include "camera_view/birdviewWindow.hpp"

class UndistortionTab : public QWidget
{
    Q_OBJECT
private slots:
    void set_undistortion_success_message(bool status);
private:
    QPushButton *undistort_button;
    QPushButton *undistorted_view_button;
    QComboBox *dropdown_menu;
    UndistortionWindow *undistortion_window;
    UndistortedWindow *undistorted_window;
    QLabel *undistortion_success_message;
    QLabel *undistortion_status_message;
    void init_ui();
    void start_undistortion();
    void start_undistorted();
public:
    UndistortionTab(QWidget *parent=nullptr);
    ~UndistortionTab();
    void set_undistortion_status_message();
    void clear_undistortion_success_message();
};

class ProjectionTab : public QWidget
{
    Q_OBJECT;
private slots:
    void set_projection_success_message(bool status);
private:
    QPushButton *project_button;
    QPushButton *projected_view_button;
    QComboBox *dropdown_menu;
    ProjectionWindow *projection_window;
    ProjectedWindow *projected_window;
    QLabel *undistortion_status_message;
    QLabel *projection_status_message;
    QLabel *projection_success_message;
    void init_ui();
    void start_projection();
    void start_projected();
public:
    ProjectionTab(QWidget *parent=nullptr);
    ~ProjectionTab();
    void set_undistortion_status_message();
    void set_projection_status_message();
    void clear_projection_success_message();
};

class BirdviewTab : public QWidget
{
private:
    QPushButton *start_button;
    QLabel *undistortion_status_message;
    QLabel *projection_status_message;
    BirdviewWindow *birdview_window;
    void init_ui();
    void start_birdview();
public:
    BirdviewTab(QWidget *parent=nullptr);
    ~BirdviewTab();
    void set_undistortion_status_message();
    void set_projection_status_message();
};

#endif
