#ifndef IMAGE_VISUALIZER_H
#define IMAGE_VISUALIZER_H

#include <QWidget>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <QImage>
#include <QPainter>
#include <QResizeEvent>
#include <mutex>
#include <semaphore>
#include <memory>
#include <QLabel>
#include <QRect>
#include <string>
#include <cv_bridge/cv_bridge.h>
class ImageVisualizer: public QWidget {
    Q_OBJECT
public:
    ImageVisualizer(int width, int height,
                    std::string img_topic = "/image_raw", 
                    std::string node_name = "viz_image", 
                    QWidget *parent =nullptr);
    ~ImageVisualizer();

protected:
    void paintEvent(QPaintEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;
    void updateCounter();
private:
    int image_width_;
    int image_height_;
    std::string node_name_ ;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    void updateImages(const sensor_msgs::msg::Image::SharedPtr msg);
    
    QPainter painter;
    QRect numberRect;
    QImage image_;
    QPixmap pixmap_;
    QLabel *imglabel_;
    std::mutex mutex_img_;
    cv_bridge::CvImagePtr cv_image_ = nullptr;

    int img_Count = 0 , show_Count =0;
};



#endif