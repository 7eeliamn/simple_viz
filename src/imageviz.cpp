#include "imageviz.h"

#include <chrono>
#include <iostream>

#include <QGridLayout>
#include <QTimer>

#define MAX_IMAGE_SIZE_H 1080
#define MAX_IMAGE_SIZE_W 1920
#define MIN_IMAGE_SIZE_H 180
#define MIN_IMAGE_SIZE_W 320
ImageVisualizer::ImageVisualizer(int width, int height, std::string img_topic, std::string node_name, QWidget *parent)
    : QWidget(parent), image_width_(width), image_height_(height), node_name_(node_name)
{   
    QSize initSize(image_width_,image_height_);
    node_ = rclcpp::Node::make_shared(node_name_);
    
    subscription_ = node_->create_subscription<sensor_msgs::msg::Image>(
        img_topic, 10, 
        std::bind(&ImageVisualizer::updateImages, this, std::placeholders::_1));
    std::thread([this](){ rclcpp::spin(node_); }).detach();
    
    //添加label 用于显示图像
    setLayout(new QGridLayout); // 居中分布
    imglabel_ = new QLabel(this);// 指定父类 即可在本widget创建该label
    this->layout()->addWidget(imglabel_);
    imglabel_->setAlignment(Qt::AlignCenter);
    imglabel_->setMinimumSize(QSize(MIN_IMAGE_SIZE_W,MIN_IMAGE_SIZE_H));
    imglabel_->setMaximumSize(QSize(MAX_IMAGE_SIZE_W,MAX_IMAGE_SIZE_H));
    
    
    // 无图像显示
    QPixmap pixmap(initSize);
    pixmap.fill(Qt::white);
    painter.setPen(Qt::black); // 设置字体颜色为黑色
    QFont font = painter.font();
    font.setPointSize(24);     // 设置字体大小
    painter.setFont(font);
    painter.begin(&pixmap);
    painter.drawText(pixmap.rect(), Qt::AlignCenter, "no image");
    // 绘制帧率
    numberRect = painter.boundingRect(pixmap.rect(), Qt::AlignRight | Qt::AlignTop, QString::number(show_Count));
    numberRect.moveTopRight(pixmap.rect().topRight()-QPoint(20,-20));
    painter.drawText(numberRect, Qt::AlignRight | Qt::AlignTop, QString::number(show_Count));
    painter.end();
    pixmap_ = pixmap;
    imglabel_->setPixmap(pixmap_);
    this->resize(initSize);//默认大小

    // 定时器 统计每秒接受到的msg
    QTimer *timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &ImageVisualizer::updateCounter);
    timer->start(1000);  // 1000 ms
    
}

ImageVisualizer::~ImageVisualizer()
{
    
    rclcpp::shutdown();

}
void ImageVisualizer::updateCounter(){
    show_Count=img_Count;
    img_Count=0;
}

void ImageVisualizer::updateImages(const sensor_msgs::msg::Image::SharedPtr msg){
    try {
        // 使用 cv_bridge 将 ROS 图像消息转换为 OpenCV 图像
        std::lock_guard<std::mutex> lock(mutex_img_);
        // auto start = std::chrono::high_resolution_clock::now();
        cv_image_ = cv_bridge::toCvCopy(msg, "bgr8");
        // auto end = std::chrono::high_resolution_clock::now();
        // RCLCPP_INFO(node_->get_logger(), "Time taken: %ld ms", std::chrono::duration_cast<std::chrono::microseconds>(end - start).count());
        if(!cv_image_){
            return ;
        }
        if(cv_image_->image.channels() == 3){
            image_ = QImage(cv_image_->image.data, cv_image_->image.cols, cv_image_->image.rows, QImage::Format_RGB888).rgbSwapped();
        }
        else if(cv_image_->image.channels() == 1){
            image_ = QImage(cv_image_->image.data, cv_image_->image.cols, cv_image_->image.rows, QImage::Format_Grayscale8);
        }
        img_Count++;
        update();
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(node_->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void ImageVisualizer::paintEvent(QPaintEvent *event){
    std::lock_guard<std::mutex> lock(mutex_img_);
    if(image_.byteCount() == 0){
        return ;
    } 
    pixmap_ = QPixmap::fromImage(image_);
    pixmap_ = pixmap_.scaled(this->imglabel_->size(), Qt::KeepAspectRatio);
    // 绘制帧率
    painter.begin(&pixmap_);
    numberRect = painter.boundingRect(pixmap_.rect(), Qt::AlignRight | Qt::AlignTop, QString::number(show_Count));
    numberRect.moveTopRight(pixmap_.rect().topRight()-QPoint(20,-20));
    painter.drawText(numberRect, Qt::AlignRight | Qt::AlignTop, QString::number(show_Count));
    painter.end();
    imglabel_->setPixmap(pixmap_);
}

void ImageVisualizer::resizeEvent(QResizeEvent *event) {
    QSize size = event->size();
    if(size.height() < MIN_IMAGE_SIZE_H || size.width() < MIN_IMAGE_SIZE_W){
        size = QSize(MIN_IMAGE_SIZE_W,MIN_IMAGE_SIZE_H);
    }
    if(size.height() > MAX_IMAGE_SIZE_H || size.width() > MAX_IMAGE_SIZE_W){
        size = QSize(MAX_IMAGE_SIZE_W,MAX_IMAGE_SIZE_H);
    }
    std::cout << "resizeEvent : " << size.height()<< " " << size.width() << std::endl;
    QPixmap px = pixmap_.scaled(size, Qt::KeepAspectRatio);
    imglabel_->setPixmap(px);
    imglabel_->resize(size);
    QWidget::resizeEvent(event);
}

