#ifndef MAINWINDOW_HPP_
#define MAINWINDOW_HPP_

#include <QMainWindow>
#include <QTabWidget>
#include <QPushButton>
#include <QWidget>
#include <QLabel>
#include <QVBoxLayout>
#include <QStackedWidget>
#include "camera_view/tabs.hpp"

class MainWindow : public QWidget
{
private:
    QTabWidget* tab_widget;
    UndistortionTab *undistortion_tab;
    ProjectionTab *projection_tab;
    BirdviewTab * birdview_tab;
    QPushButton *close_button;
    void init_ui();
    void init_vars();
    void tab_changed(int index);
public:
    MainWindow(QWidget *parent = nullptr);
};
#endif
