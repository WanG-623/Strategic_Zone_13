#ifndef _CONTROL_H_
#define _CONTROL_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
//------------------------------------------------ 定义一个三维空间点坐标的运算类 --------------------------------------------------
class Point
{
    public:
    double x, y, z; //定义成员变量
    // 默认构造函数，将x\y\z 初始化为0
    Point():x(0.), y(0.), z(0.){
    }
    // 构造函数：将x、y坐标初始化为x_in y_in 并将z坐标初始化为0
    Point(double x_in, double y_in): x(x_in), y(y_in), z(0.){
    }

    template<typename T>
    Point(T point): x(point.x), y(point.y), z(point.z){
    }
    // 设置点的坐标-二维 x y 
    void setValue(double x, double y){
        this->x = x;
        this->y = y;
        this->z = 0.;
    }
    // 设置点的坐标-三维 x y z 
    void setValue(double x, double y, double z){
        this->x = x;
        this->y = y;
        this->z = z;
    }
    // 判断该店是否为原点
    bool isOrigin(){
        return this->x == 0 && this->y == 0;
    }
// 书写成员函数：定义两点之间的运算式
    // 加法
    Point operator+(const Point p){
        Point sum;
        sum.x = x + p.x;
        sum.y = y + p.y;
        sum.z = z + p.z;
        return sum;
    }
    // 减法
    Point operator-(const Point p){
        Point sum;
        sum.x = x - p.x;
        sum.y = y - p.y;
        sum.z = z - p.z;
        return sum;
    }
    // 两点之间标量的乘法
    Point operator*(const double a){
        Point product;
        product.x = x * a;
        product.y = y * a;
        product.z = z * a;
        return product;
    }
    // 重载运算符 实现点与点之间累加
    void operator+= (Point p){
        x += p.x;
        y += p.y;
        z += p.z;
    }
    // 计算两点之间的距离
    double distance(Point P){
        return sqrt((x - P.x) * (x - P.x) + (y - P.y) * (y - P.y));
    }
    // 计算两点之间的极角
    double polar(Point end){
        return atan2(end.y - y, end.x - x);
    }
    // 计算两点之间的叉积
    double cross(Point P){
        return x * P.y  - y * P.x;
    }
    // 计算两点之间的点积
    double dot(Point P){
        return x * P.x  + y * P.y;
    }
    // UnitVec返回该点为起点，沿着当前yaw角度方向的单位向量
    static Point UnitVec(double yaw){
        Point p;
        p.x = cos(yaw);
        p.y = sin(yaw);
        return p;
    }
    // 计算该点的模长
    double norm(){
        return sqrt(this->x * this->x + this->y * this->y);
    }
    // 该点绕原点旋转yaw角度后返回旋转后的点
    Point rotate(double yaw){
        Point p;
        p.x = this->x * cos(yaw) - this->y * sin(yaw);
        p.y = this->x * sin(yaw) + this->y * cos(yaw);
        return p;
    }
};
// ---------------------------------------------------定义控制类---------------------------------------------------------------------
class control  
{
private:
    ros::NodeHandle nx; // 定义ROS句柄
    ros::Subscriber subPath;// 订阅路径
    ros::Subscriber subOdom;// 订阅里程计
    ros::Publisher pubTwist;// 发布控制速度(线速度/角速度)
    std::vector<Point> path;// path储存路径点的序列，类型为Point
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud_;// 用pcl_cloud_储存点云数据
public:
    // 构造函数control()初始化了ROS节点句柄、订阅路径消息、订阅小车里程计消息、发布小车速度控制消息三个ROS话题
    control(/* args */):nx("~")
{
    subPath=nx.subscribe("/lane_show", 1, &control::pathHandler, this);
    subOdom=nx.subscribe("/odom_chassis", 2, &control::twistHandler, this);
    pubTwist = nx.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
};
    ~control()
    {};
public:
// 定义变量
double target_speed=0.;
double target_turn=0.;
sensor_msgs::PointCloud2 path_;
nav_msgs::Odometry  twist_; 
// 成员回调函数path
void pathHandler(const sensor_msgs::PointCloud2::ConstPtr &msg); // 处理ROS话题中的路径消息
// 成员回调函数twist
void twistHandler(const nav_msgs::Odometry::ConstPtr &msg);// 处理ROS话题中的小车里程计消息
void loadPath(sensor_msgs::PointCloud2 &cloud);// 用于将ROS话题中的路径消息转换为 pcl::PointCloud<pcl::PointXYZI> 类型的点云数据
void loadTwist(nav_msgs::Odometry &twist);// 处理ROS话题中的里程计消息转换为 nav_msgs::Odometry 类型的数据
void run(); // 主要控制函数，用于控制小车的运动
void curve();// 用于控制小车路径中曲线的部分运动
void turn();  // 用于控制小车路径中转弯的部分运动
// 小车转弯的坐标
double turn_point_x=-7.2;
double turn_point_y=-7.6;
// TF位姿转换定义
tf::TransformListener listener_;
tf::StampedTransform transForm_;
nav_msgs::Odometry cur_weizhi;// 当前小车的位置信息
};
// ---------------------------------------------------------------------------------------------------------------------------------



#endif