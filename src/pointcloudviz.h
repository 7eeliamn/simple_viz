#ifndef POINT_CLOUD_VISUALIZER_H
#define POINT_CLOUD_VISUALIZER_H
//QOpenGL
#include <QOpenGLWidget>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>
//ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
//QEvent
#include <QMouseEvent>
#include <QWheelEvent>
//utils
#include <mutex>
#include <limits>
#include <string>

class PointCloudVisualizer : public QOpenGLWidget, protected QOpenGLFunctions_3_3_Core
{
    Q_OBJECT

public:
    explicit PointCloudVisualizer(int width, int height, 
                                    std::string pcd_topic = "/points_raw", 
                                    std::string node_name = "viz_pointcloud", 
                                    QWidget *parent = nullptr);
    ~PointCloudVisualizer();


protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
    QSize sizeHint() const override;

    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;

private:
    int width_;
    int height_;
    std::string pcd_topic_;
    std::string node_name_;
    void updatePoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg); 
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    std::vector<float> points_; // 点云数据

    float maxDistance_ = std::numeric_limits<float>::min(), 
          minDistance_ = std::numeric_limits<float>::max(),
          maxIntensity_ = std::numeric_limits<float>::min(),
          minIntensity_ = std::numeric_limits<float>::max();
    float maxPercent_ = 0.5f;

    rclcpp::Node::SharedPtr node_;
    std::mutex mutex_pcd_;
    bool gotpcd_ = false;


protected:
    QPoint lastPos;

    virtual unsigned int drawMeshline(float size, int count);
    virtual void drawCooraxis(float length);
    virtual unsigned int drawPointdata(std::vector<float> &pointVertexs);
    virtual void initPointData();  

    QOpenGLShaderProgram m_shaderProgramMesh;
    QOpenGLShaderProgram m_shaderProgramAxis;
    QOpenGLShaderProgram m_shaderProgramPoint;
 
    unsigned int m_VBO_MeshLine;
    unsigned int m_VAO_MeshLine;
 
    unsigned int m_VBO_Axis;
    unsigned int m_VAO_Axis;
 
    unsigned int m_VBO_Point;
    unsigned int m_VAO_Point;
 
    unsigned int m_pointCount = 0;  // 点云 顶点数量
    unsigned int m_vertexCount;     // 地面方格 顶点数量
 
    float m_xRotate;
    float m_zRotate;
    float m_xTrans;
    float m_yTrans;
    float m_zoom;
    
    bool isIntensity = true; // 默认按照强度 显示

};

#endif // POINT_CLOUD_VISUALIZER_H
