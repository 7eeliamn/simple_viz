#include <rclcpp/rclcpp.hpp>
#include <QApplication>
// #include "visualizer.h"
#include <QMainWindow>
#include "pointcloudviz.h"
#include "imageviz.h"
#include <QGridLayout>
#include <QSplitter>
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);
    
    QWidget *window = new QWidget();
    window->setWindowTitle("Visualizer");
    ImageVisualizer *image_visualizer = new ImageVisualizer(640,480,"/image_raw/video7");
    PointCloudVisualizer *pointcloud_visualizer = new PointCloudVisualizer(640,480,"/rslidar_points_main");
    // image_visualizer->show();
    image_visualizer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    pointcloud_visualizer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    QGridLayout * grid = new QGridLayout();
    QSplitter *splitter = new QSplitter(Qt::Horizontal); // 设置水平排列
    splitter->addWidget(image_visualizer);
    splitter->addWidget(pointcloud_visualizer);
    grid->addWidget(splitter);
    // grid->setRowStretch(0, 1);//row strech
    // grid->setColumnStretch(0, 1);
    window->setLayout(grid);

    window->show();


    return app.exec();
}



// #include "glviz.hpp"
// int main(int argc, char *argv[]) {
//     QApplication app(argc, argv);

//     MyPangolinWidget widget;
//     widget.resize(800, 600);
//     widget.show();

//     return app.exec();
// }

