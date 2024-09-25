#include "pointcloudviz.h"
#include <QMouseEvent>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <thread>
#include <filesystem>
#include <algorithm>
#include <chrono>
#include <iostream>
PointCloudVisualizer::PointCloudVisualizer(int width, int height, std::string pcd_topic, std::string node_name,QWidget *parent)
    : QOpenGLWidget(parent),
      m_xRotate(-30.0f),
      m_zRotate(100.0f),
      m_xTrans(0.0f),
      m_yTrans(0.0f),
      m_zoom(45.0f),
      width_(width),
      height_(height),
      pcd_topic_(pcd_topic),
      node_name_(node_name)
{
    // 初始化 ROS 2 节点
    node_ = rclcpp::Node::make_shared(node_name_);
    
    subscription_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        pcd_topic_, 10,
        std::bind(&PointCloudVisualizer::updatePoints, this, std::placeholders::_1)
    );

    // 创建一个 ROS 2 执行器并在单独的线程中旋转
    std::thread([this]() { rclcpp::spin(this->node_); }).detach();

}

PointCloudVisualizer::~PointCloudVisualizer()
{
    rclcpp::shutdown();

    makeCurrent();
    glDeleteBuffers(1, &m_VBO_MeshLine);
    glDeleteVertexArrays(1, &m_VAO_MeshLine);
 
    glDeleteBuffers(1, &m_VBO_Axis);
    glDeleteVertexArrays(1, &m_VAO_Axis);
 
    glDeleteBuffers(1, &m_VBO_Point);
    glDeleteVertexArrays(1, &m_VAO_Point);
 
    m_shaderProgramMesh.release();
    m_shaderProgramAxis.release();
    m_shaderProgramPoint.release();
 
    doneCurrent();

}

// 提供默认尺寸
QSize PointCloudVisualizer::sizeHint() const
{
    return QSize(width_, height_);  // 返回首选的窗口尺寸
}
void PointCloudVisualizer::initializeGL()
{
    initializeOpenGLFunctions();
    glEnable(GL_DEPTH_TEST);
    // 链接 mesh shader
    if(!m_shaderProgramMesh.addShaderFromSourceFile(QOpenGLShader::Vertex,
            "../src/shader/shader_mesh.vs")){
        std::cerr << "shader_mesh.vs load failed" << std::endl;
    }
    if(!m_shaderProgramMesh.addShaderFromSourceFile(QOpenGLShader::Fragment,
            "../src/shader/shader_mesh.fs")){
        std::cerr << "shader_mesh.fs load failed" << std::endl;
    }
    m_shaderProgramMesh.link();
 
    // 链接 coordinate axis shaders
    if(!m_shaderProgramAxis.addShaderFromSourceFile(QOpenGLShader::Vertex,
            "../src/shader/shader_axis.vs")){
        std::cerr << "shader_axis.vs load failed" << std::endl;
    }
    if(!m_shaderProgramAxis.addShaderFromSourceFile(QOpenGLShader::Fragment,
            "../src/shader/shader_axis.fs")){
        std::cerr << "shader_axis.fs load failed" << std::endl;
    }
    m_shaderProgramAxis.link();
 
    // 链接 pointcloud shaders
    if(!m_shaderProgramPoint.addShaderFromSourceFile(QOpenGLShader::Vertex,
            "../src/shader/shader_points.vs")){
        std::cerr << "shader_points.vs load failed" << std::endl;
    }
    if(!m_shaderProgramPoint.addShaderFromSourceFile(QOpenGLShader::Fragment,
            "../src/shader/shader_points.fs")){
        std::cerr << "shader_points.fs load failed" << std::endl;
    }
    m_shaderProgramPoint.link();
 

    //绘制基本图形 
    m_vertexCount = drawMeshline(2.0, 16); //底面方格
    drawCooraxis(2.0); //坐标轴
    initPointData();

}

void PointCloudVisualizer::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);
}

void PointCloudVisualizer::paintGL()
{
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f); // 默认背景色白色
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
 
    /*
       为了将坐标从一个坐标系转换到另一个坐标系，需要用到几个转换矩阵，
       分别是模型(Model)、视图(View)、投影(Projection)三个矩阵。
    */
    QMatrix4x4 projection, view, model;
    //透视矩阵变换
    projection.perspective(m_zoom, (float)width() / (float)height(), 1.0f, 100.0f);
 
    // eye：摄像机位置  center：摄像机看的点位 up：摄像机上方的朝向
    view.lookAt(QVector3D(0.0, 0.0, 50.0), QVector3D(0.0, 0.0, 1.0), QVector3D(0.0, 1.0, 0.0));
 
    model.translate(m_xTrans, m_yTrans, 0.0);
    model.rotate(m_xRotate, 1.0, 0.0, 0.0);
    model.rotate(m_zRotate, 0.0, 0.0, 1.0);
 
    m_shaderProgramMesh.bind();
    m_shaderProgramMesh.setUniformValue("projection", projection);
    m_shaderProgramMesh.setUniformValue("view", view);
    m_shaderProgramMesh.setUniformValue("model", model);
 
    m_shaderProgramAxis.bind();
    m_shaderProgramAxis.setUniformValue("projection", projection);
    m_shaderProgramAxis.setUniformValue("view", view);
    m_shaderProgramAxis.setUniformValue("model", model);
 
    m_shaderProgramPoint.bind();
    m_shaderProgramPoint.setUniformValue("projection", projection);
    m_shaderProgramPoint.setUniformValue("view", view);
    m_shaderProgramPoint.setUniformValue("model", model);
    if(isIntensity)
        m_shaderProgramPoint.setUniformValue("isIntensity", true);
    else
        m_shaderProgramPoint.setUniformValue("isIntensity", false);
    
 
    //画网格
    m_shaderProgramMesh.bind();
    glBindVertexArray(m_VAO_MeshLine);
    glLineWidth(1.0f);
    glDrawArrays(GL_LINES, 0, m_vertexCount);
 
    //画坐标轴
    m_shaderProgramAxis.bind();
    glBindVertexArray(m_VAO_Axis);
    glLineWidth(3.0f);
    glDrawArrays(GL_LINES, 0, 6);
 
    //画点云
    {
        std::lock_guard<std::mutex> lock(mutex_pcd_);
        // 读取点云数据并传递到GPU
        m_pointCount = drawPointdata(points_);
        // 绘制 vao 中的点云
        m_shaderProgramPoint.bind();
        m_shaderProgramPoint.setUniformValue("minIntensity",minIntensity_);
        m_shaderProgramPoint.setUniformValue("maxIntensity",maxIntensity_);
       
        m_shaderProgramPoint.setUniformValue("minDistance",minDistance_);
        m_shaderProgramPoint.setUniformValue("maxDistance",maxDistance_/10);
        glBindVertexArray(m_VAO_Point);
        glPointSize(2.0f);
        glDrawArrays(GL_POINTS, 0, m_pointCount);
        // gotpcd_ = false;
    }
}

// 生产者
// 接收点云topic并更新点云数据
void PointCloudVisualizer::updatePoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_pcd_);
    points_.clear();
    #ifdef LOG
        RCLCPP_INFO(node_->get_logger(), "Received point cloud with %d points", msg->width * msg->height);
    #endif
    // auto start = std::chrono::high_resolution_clock::now();
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*msg, "intensity");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
        points_.emplace_back(*iter_x);
        points_.emplace_back(*iter_y);
        points_.emplace_back(*iter_z);
        points_.emplace_back(*iter_intensity);
        float distance = std::pow(*iter_x, 2) + std::pow(*iter_y, 2) + std::pow(*iter_z, 2);
        maxIntensity_ = std::max(std::move(maxIntensity_),std::move(*iter_intensity));
        minIntensity_ = std::min(std::move(minIntensity_),std::move(*iter_intensity));
        maxDistance_ = std::max(std::move(maxDistance_),std::move(distance));
        minDistance_ = std::min(std::move(minDistance_),std::move(distance));

    }
    // gotpcd_ = true;
    // auto end = std::chrono::high_resolution_clock::now();
    // std::cout << "update points time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
    update();  // 请求重绘
}

void PointCloudVisualizer::mousePressEvent(QMouseEvent *event)
{
    lastPos = event->pos();
}

void PointCloudVisualizer::mouseMoveEvent(QMouseEvent *event)
{
    int dx = event->pos().x() - lastPos.x();
    int dy = event->pos().y() - lastPos.y();
    if (event->buttons() & Qt::LeftButton)
    {
        m_xRotate = m_xRotate + 0.3 * dy;
        m_zRotate = m_zRotate + 0.3 * dx;
 
        if (m_xRotate > 30.0f)
        {
            m_xRotate = 30.0f;
        }
        if (m_xRotate < -120.0f)
        {
            m_xRotate = -120.0f;
        }
        update();
    }
    else if (event->buttons() & Qt::MidButton)
    {
        m_xTrans = m_xTrans + 0.1 * dx;
        m_yTrans = m_yTrans - 0.1 * dy;
        update();
    }
    lastPos = event->pos();
}



void PointCloudVisualizer::wheelEvent(QWheelEvent *event)
{
    auto scroll_offest = event->angleDelta().y() / 120;
    m_zoom = m_zoom - (float)scroll_offest;
 
    if (m_zoom < 1.0f)    /* 放大限制 */
    {
        m_zoom = 1.0f;
    }
 
    if (m_zoom > 80.0f)
    {
        m_zoom = 80.0f;
    }
 
    update();
}

// 绘制地面网格
// 传递网格的顶点数据到GPU
unsigned int PointCloudVisualizer::drawMeshline(float size, int count)
{
    std::vector<float> mesh_vertexs;
    unsigned int vertex_count = 0;
 
    float start = count * (size / 2);
    float posX = start, posZ = start;
 
    for (int i = 0; i <= count; ++i)
    {
        mesh_vertexs.push_back(posX);
        mesh_vertexs.push_back(start);
        mesh_vertexs.push_back(0);
 
        mesh_vertexs.push_back(posX);
        mesh_vertexs.push_back(-start);
        mesh_vertexs.push_back(0);
 
        mesh_vertexs.push_back(start);
        mesh_vertexs.push_back(posZ);
        mesh_vertexs.push_back(0);
 
        mesh_vertexs.push_back(-start);
        mesh_vertexs.push_back(posZ);
        mesh_vertexs.push_back(0);
 
        posX = posX - size;
        posZ = posZ - size;
    }
 
    glGenVertexArrays(1, &m_VAO_MeshLine); // 生成vao
    glGenBuffers(1, &m_VBO_MeshLine); // 生成vbo
 
    glBindVertexArray(m_VAO_MeshLine); // 绑定vao
 
    glBindBuffer(GL_ARRAY_BUFFER, m_VBO_MeshLine); // 绑定vbo
 
    glBufferData(GL_ARRAY_BUFFER, mesh_vertexs.size() * sizeof(float), &mesh_vertexs[0], GL_STATIC_DRAW); // 将顶点数据传入vbo
 
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0); // 位置属性
    glEnableVertexAttribArray(0);// 启动 vao 定点属性  索引为0 
 
    glBindBuffer(GL_ARRAY_BUFFER, 0); // 解绑vbo
 
    glBindVertexArray(0); // 解绑vao
 
    vertex_count = (int)mesh_vertexs.size() / 3;
 
    return vertex_count;
}

// 绘制坐标轴
// 传递坐标轴的顶点数据到GPU
void PointCloudVisualizer::drawCooraxis(float length)
{
    std::vector<float> axis_vertexs =
    {
        //x,y ,z ,r, g, b
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
        length, 0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
        0.0, length, 0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        0.0, 0.0, length, 0.0, 0.0, 1.0,
    };
 
    glGenVertexArrays(1, &m_VAO_Axis); // 生成vao
    glGenBuffers(1, &m_VBO_Axis);       // 生成vbo
 
    glBindVertexArray(m_VAO_Axis); // 绑定vao
 
    glBindBuffer(GL_ARRAY_BUFFER, m_VBO_Axis); // 绑定vbo 
    glBufferData(GL_ARRAY_BUFFER, axis_vertexs.size() * sizeof(float), &axis_vertexs[0], GL_STATIC_DRAW); // 将顶点数据传入vbo
 
    //  坐标属性 索引 0  大小为 3 , float 类型, 不需要归一化, 步长为 6 * sizeof(float), 偏移量为 0
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)0); 
    glEnableVertexAttribArray(0); //启动 vao 定点属性  索引为0
 
    // 颜色属性 索引 1  大小为 3 , float 类型, 不需要归一化, 步长为 6 * sizeof(float), 偏移量为 3 * sizeof(float)
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);   //启动 vao 定点属性  索引为1
 
    glBindBuffer(GL_ARRAY_BUFFER, 0); // 接棒vbo
 
    glBindVertexArray(0); //解绑 vao
}

// 初始化点云VAO VBO 
void PointCloudVisualizer::initPointData(){
    glGenVertexArrays(1, &m_VAO_Point); // 生成vao
    glGenBuffers(1, &m_VBO_Point); // 生成vbo
}

// 消费者 
// 将点云数据(x, y, z, i)传递到GPU (VBO)
unsigned int PointCloudVisualizer::drawPointdata(std::vector<float> &pointVertexs)
{
    if(pointVertexs.empty())
    {
        #ifdef LOG
            RCLCPP_INFO(node_->get_logger(), "No point cloud data to render");
        #endif
        return 0;
    }
    unsigned int point_count = 0;

    glBindVertexArray(m_VAO_Point); // 绑定vao
    glBindBuffer(GL_ARRAY_BUFFER, m_VBO_Point); // 绑定vbo
    glBufferData(GL_ARRAY_BUFFER, pointVertexs.size() * sizeof(float), &pointVertexs[0], GL_DYNAMIC_DRAW); // 将顶点数据传入vbo
 
    // situation属性
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);
 
    // intensity属性
    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
 
    glBindBuffer(GL_ARRAY_BUFFER, 0);
 
    glBindVertexArray(0);
 
    point_count = (unsigned int)pointVertexs.size() / 4;

    return point_count;
}