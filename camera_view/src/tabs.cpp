#include <filesystem>
#include <iostream>

#include <QVBoxLayout>
#include <QString>
#include <QFont>

#include "camera_view/tabs.hpp"
#include "camera_view/externVariables.hpp"

UndistortionTab::UndistortionTab(QWidget *parent)
    : QWidget(parent),
    undistortion_window(new UndistortionWindow()),
    undistorted_window(new UndistortedWindow()),
    undistort_button(new QPushButton("Undistort", this)),
    undistorted_view_button(new QPushButton("Undistorted View", this)),
    dropdown_menu(new QComboBox(this)),
    undistortion_success_message(new QLabel(this)),
    undistortion_status_message(new QLabel(this))
{
    init_ui();
    undistortion_window->attach(parent);
    undistorted_window->attach(parent);
}

void UndistortionTab::init_ui()
{
    undistortion_window->set_frame_rate(30);
    undistorted_window->set_frame_rate(30);
    
    undistortion_success_message->setFont(QFont("Arial", 30));
    dropdown_menu->setFont(QFont("Arial", 50));
    undistort_button->setFont(QFont("Arial", 40));
    undistortion_status_message->setFont(QFont("Arial", 30));
    undistortion_status_message->setStyleSheet("QLabel { color : red; }");

    QVBoxLayout *VBL = new QVBoxLayout(this);
    VBL->addWidget(undistortion_success_message, 0, Qt::AlignCenter);
    VBL->addWidget(undistortion_status_message, 0, Qt::AlignCenter);
    VBL->addWidget(dropdown_menu);
    VBL->addWidget(undistort_button);
    VBL->addWidget(undistorted_view_button);
    VBL->setContentsMargins(100, 100, 100, 100);
    dropdown_menu->addItem(QString::fromStdString(right_camera.name));
    dropdown_menu->addItem(QString::fromStdString(left_camera.name));
    dropdown_menu->addItem(QString::fromStdString(back_camera.name));

    connect(undistort_button, &QPushButton::clicked, this, &UndistortionTab::start_undistortion);
    connect(undistorted_view_button, &QPushButton::clicked, this, &UndistortionTab::start_undistorted);
    connect(undistortion_window, &UndistortionWindow::undistortion_status, this, &UndistortionTab::set_undistortion_success_message);

    set_undistortion_status_message();
}

void UndistortionTab::set_undistortion_success_message(bool status)
{
    if (status)
    {
        undistortion_success_message->setText("Undistortion Success");
        undistortion_success_message->setStyleSheet("QLabel { color : green; }");
    }
    else
    {
        undistortion_success_message->setText("Undistortion Failed");
        undistortion_success_message->setStyleSheet("QLabel { color : red; }");
    }
    set_undistortion_status_message();
}

void UndistortionTab::set_undistortion_status_message()
{
    // disable show undistorted button if no undistortion file
    std::vector<std::string> names = directories::undistortion_file_check();
    std::string warning_msg = "[ ";

    if (names.empty())
    {
        undistorted_view_button->setEnabled(true);
        undistortion_status_message->setText("");
    }
    else
    {
        undistorted_view_button->setEnabled(false);
        for (int i=0; i<names.size(); ++i)
        {
            if (i != names.size() - 1)
            {
                warning_msg += names[i] + ", ";
            }
            else
            {
                warning_msg += names[i];
            }
        }
        warning_msg += " ]\n";
        warning_msg += "camera(s) not undistorted";
        undistortion_status_message->setText(QString::fromStdString(warning_msg));
    }
}

void UndistortionTab::clear_undistortion_success_message()
{
    undistortion_success_message->clear();
}

void UndistortionTab::start_undistortion()
{
    Camera selected_camera;
    if (dropdown_menu->currentText().toStdString() == left_camera.name)
    {
        selected_camera = left_camera;
    }
    else if (dropdown_menu->currentText().toStdString() == right_camera.name)
    {
        selected_camera = right_camera;
    }
    else if (dropdown_menu->currentText().toStdString() == back_camera.name)
    {
        selected_camera = back_camera;
    }
    undistortion_window->start(selected_camera);
}

void UndistortionTab::start_undistorted()
{
    Camera selected_camera;
    if (dropdown_menu->currentText().toStdString() == left_camera.name)
    {
        selected_camera = left_camera;
    }
    else if (dropdown_menu->currentText().toStdString() == right_camera.name)
    {
        selected_camera = right_camera;
    }
    else if (dropdown_menu->currentText().toStdString() == back_camera.name)
    {
        selected_camera = back_camera;
    }
    undistorted_window->start(selected_camera);
}

// need to explicitly delete these widgets because
// they are not set as the child of UndistortionTab (to prevent the widgets from automatically being added to the layout of UndistortionTab)
UndistortionTab::~UndistortionTab()
{
    delete undistortion_window;
    delete undistorted_window;
}

ProjectionTab::ProjectionTab(QWidget *parent)
: QWidget(parent),
project_button(new QPushButton("Project", this)),
projected_view_button(new QPushButton("Projected View", this)),
projection_status_message(new QLabel(this)),
projection_success_message(new QLabel(this)),
dropdown_menu(new QComboBox(this)),
undistortion_status_message(new QLabel(this)),
projection_window(new ProjectionWindow()),
projected_window(new ProjectedWindow())
{
    projection_window->attach(parent);
    projected_window->attach(parent);
    init_ui();
}

void ProjectionTab::init_ui()
{
    projection_window->set_frame_rate(30);
    projected_window->set_frame_rate(30);

    dropdown_menu->setFont(QFont("Arial", 50));
    project_button->setFont(QFont("Arial", 40));
    undistortion_status_message->setFont(QFont("Arial", 20));
    undistortion_status_message->setStyleSheet("QLabel { color : red; }");
    projection_status_message->setFont(QFont("Arial", 20));
    projection_status_message->setStyleSheet("QLabel { color : red; }");

    QVBoxLayout *VBL = new QVBoxLayout(this);
    VBL->addWidget(undistortion_status_message, 0, Qt::AlignCenter);
    VBL->addWidget(projection_status_message, 0, Qt::AlignCenter);
    VBL->addWidget(projection_success_message, 0, Qt::AlignCenter);
    VBL->addWidget(dropdown_menu);
    VBL->addWidget(project_button);
    VBL->addWidget(projected_view_button);
    VBL->setContentsMargins(100, 100, 100, 100);
    dropdown_menu->addItem(QString::fromStdString(right_camera.name));
    dropdown_menu->addItem(QString::fromStdString(left_camera.name));
    dropdown_menu->addItem(QString::fromStdString(back_camera.name));

    connect(project_button, &QPushButton::clicked, this, &ProjectionTab::start_projection);
    connect(projected_view_button, &QPushButton::clicked, this, &ProjectionTab::start_projected);
    connect(projection_window, &ProjectionWindow::projection_status, this, &ProjectionTab::set_projection_success_message);

    set_undistortion_status_message();
    set_projection_status_message();
}

void ProjectionTab::set_projection_success_message(bool status)
{
    if (status)
    {
        projection_success_message->setText("Projection Success");
        projection_success_message->setStyleSheet("QLabel { color : green; }");
    }
    else
    {
        projection_success_message->setText("Projection Failed");
        projection_success_message->setStyleSheet("QLabel { color : red; }");
    }
    set_projection_status_message();
}

void ProjectionTab::clear_projection_success_message()
{
    projection_success_message->clear();
}

void ProjectionTab::set_undistortion_status_message()
{
    // disable project button if no undistortion file
    std::vector<std::string> names = directories::undistortion_file_check();
    std::string warning_msg = "[ ";

    if (names.empty())
    {
        project_button->setEnabled(true);
        undistortion_status_message->setText("");
    }
    else
    {
        project_button->setEnabled(false);
        for (int i=0; i<names.size(); ++i)
        {
            if (i != names.size() - 1)
            {
                warning_msg += names[i] + ", ";
            }
            else
            {
                warning_msg += names[i];
            }
        }
        warning_msg += " ]\n";
        warning_msg += "camera(s) not undistorted";
        undistortion_status_message->setText(QString::fromStdString(warning_msg));
    }
}

void ProjectionTab::set_projection_status_message()
{
    // disable project button if no projection file
    std::vector<std::string> names = directories::projection_file_check();
    std::string warning_msg = "[ ";

    if (names.empty())
    {
        projected_view_button->setEnabled(true);
        projection_status_message->setText("");
    }
    else
    {
        projected_view_button->setEnabled(false);
        for (int i=0; i<names.size(); ++i)
        {
            if (i != names.size() - 1)
            {
                warning_msg += names[i] + ", ";
            }
            else
            {
                warning_msg += names[i];
            }
        }
        warning_msg += " ]\n";
        warning_msg += "camera(s) not projected";
        projection_status_message->setText(QString::fromStdString(warning_msg));
    }
}

void ProjectionTab::start_projection()
{
    Camera selected_camera;
    if (dropdown_menu->currentText().toStdString() == left_camera.name)
    {
        selected_camera = left_camera;
    }
    else if (dropdown_menu->currentText().toStdString() == right_camera.name)
    {
        selected_camera = right_camera;
    }
    else if (dropdown_menu->currentText().toStdString() == back_camera.name)
    {
        selected_camera = back_camera;
    }
    projection_window->start(selected_camera);
}

void ProjectionTab::start_projected()
{
    Camera selected_camera;
    if (dropdown_menu->currentText().toStdString() == left_camera.name)
    {
        selected_camera = left_camera;
    }
    else if (dropdown_menu->currentText().toStdString() == right_camera.name)
    {
        selected_camera = right_camera;
    }
    else if (dropdown_menu->currentText().toStdString() == back_camera.name)
    {
        selected_camera = back_camera;
    }
    projected_window->start(selected_camera);
}

// need to explicitly delete these widgets because
// they are not set as the child of ProjectionTab (to prevent the widgets from automatically being added to the layout of ProjectionTab)
ProjectionTab::~ProjectionTab()
{
    delete projection_window;
    delete projected_window;
}

BirdviewTab::BirdviewTab(QWidget *parent)
: QWidget(parent),
birdview_window(new BirdviewWindow()),
start_button(new QPushButton("Start", this)),
undistortion_status_message(new QLabel(this)),
projection_status_message(new QLabel(this))
{
    birdview_window->attach(parent);
    init_ui();
}

void BirdviewTab::init_ui()
{
    birdview_window->set_frame_rate(30);
    undistortion_status_message->setFont(QFont("Arial", 20));
    undistortion_status_message->setStyleSheet("QLabel { color : red; }");
    projection_status_message->setFont(QFont("Arial", 20));
    projection_status_message->setStyleSheet("QLabel { color : red; }");

    QVBoxLayout *VBL = new QVBoxLayout(this);
    VBL->addWidget(undistortion_status_message, 0, Qt::AlignCenter);
    VBL->addWidget(projection_status_message, 0, Qt::AlignCenter);
    VBL->addWidget(start_button);

    connect(start_button, &QPushButton::clicked, this, &BirdviewTab::start_birdview);
}

void BirdviewTab::start_birdview()
{
    birdview_window->start();
}

BirdviewTab::~BirdviewTab()
{
    delete birdview_window;
}

void BirdviewTab::set_undistortion_status_message()
{
    // disable project button if no undistortion file
    std::vector<std::string> names = directories::undistortion_file_check();
    std::string warning_msg = "[ ";

    if (names.empty())
    {
        start_button->setEnabled(true);
        undistortion_status_message->setText("");
    }
    else
    {
        start_button->setEnabled(false);
        for (int i=0; i<names.size(); ++i)
        {
            if (i != names.size() - 1)
            {
                warning_msg += names[i] + ", ";
            }
            else
            {
                warning_msg += names[i];
            }
        }
        warning_msg += " ]\n";
        warning_msg += "camera(s) not undistorted";
        undistortion_status_message->setText(QString::fromStdString(warning_msg));
    }
}

void BirdviewTab::set_projection_status_message()
{
    // disable project button if no projection file
    std::vector<std::string> names = directories::projection_file_check();
    std::string warning_msg = "[ ";

    if (names.empty())
    {
        start_button->setEnabled(true);
        projection_status_message->setText("");
    }
    else
    {
        start_button->setEnabled(false);
        for (int i=0; i<names.size(); ++i)
        {
            if (i != names.size() - 1)
            {
                warning_msg += names[i] + ", ";
            }
            else
            {
                warning_msg += names[i];
            }
        }
        warning_msg += " ]\n";
        warning_msg += "camera(s) not projected";
        projection_status_message->setText(QString::fromStdString(warning_msg));
    }
}
