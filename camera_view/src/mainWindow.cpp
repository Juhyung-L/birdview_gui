#include <QVBoxLayout>
#include <filesystem>

#include "camera_view/mainWindow.hpp"
#include "camera_view/settings.hpp"
#include "camera_view/projectionWindow.hpp"

MainWindow::MainWindow(QWidget *parent)
: QWidget(parent),
close_button(new QPushButton("Close", this))
{   
    init_ui();
}

void MainWindow::init_ui()
{
    this->setWindowTitle("Camera-view GUI");
    this->setMinimumSize(700, 700);

    directories::make_directory(); // makes the yaml folder if it doesn't already exist
    resolutions::load_resolutions();
    projection::load_projection_pts();

    // these are called after load_resolutions() on purpose (and not in the initializer list)
    // these objects need the resolutions to be set
    tab_widget = new QTabWidget(this);
    undistortion_tab = new UndistortionTab(this);
    projection_tab = new ProjectionTab(this);
    birdview_tab = new BirdviewTab(this);

    connect(close_button, &QPushButton::clicked, this, &MainWindow::close);

    tab_widget->setFont(QFont("Arial", 30));
    tab_widget->addTab(undistortion_tab, "Undistortion");
    tab_widget->addTab(projection_tab, "Projection");
    tab_widget->addTab(birdview_tab, "Birdview");

    connect(tab_widget, &QTabWidget::currentChanged, this, &MainWindow::tab_changed);

    close_button->setFont(QFont("Arial", 20));

    QVBoxLayout *VBL = new QVBoxLayout(this);
    VBL->addWidget(tab_widget);
    VBL->addWidget(close_button);

    this->show();
}

void MainWindow::tab_changed(int index)
{
    if (index == 0)
    {
        undistortion_tab->set_undistortion_status_message();
        undistortion_tab->clear_undistortion_success_message();
    }
    else if (index == 1)
    {
        projection_tab->set_undistortion_status_message();
        projection_tab->set_projection_status_message();
        projection_tab->clear_projection_success_message();
    }
    else if (index == 2)
    {
        birdview_tab->set_undistortion_status_message();
        birdview_tab->set_projection_status_message();
    }
}



