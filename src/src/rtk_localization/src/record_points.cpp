/*节点说明：
 * @Description:订阅gnss话题，并每隔0.1m记录一个点;
 * @Author: wanglili
 * @Date: 2023-05-04 16:30:27
 */
#include "ros/ros.h"
// #include "raw_sensor_msgs/GnssImuInfo.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
 #include <LocalCartesian.hpp>
#include <Geocentric.hpp>
#include "raw_sensor_msgs/ChanavGnss.h"

struct Point2d                   //定义接收所有坐标点的结构体变量
{
    double x;
    double y;
    double z;
    double yaw;
    double roll;
    double pitch;
    
}point2d;

struct after_Ponit2d
{
    double temp_x;
    double temp_y;
    double temp_z;
    double temp_yaw;
    double temp_roll;
    double temp_pitch;
}after_point2d;

std::vector<Point2d> points;     //定义存储所有点坐标的vector<Point2d>类型的点
std::vector<after_Ponit2d>  after_points;

std::string _gps_topic;
//std::string _filename = "/home/lyl/MapMake_ws/src/DoubleLane/data/lane_1.txt";

//gps
struct gnssinfo
{
    double latitude;
    double longitude;
    double altitude;

    double roll;
    double pitch;
    double yaw;
};
static gnssinfo init_gnssinfo;
std::string _param_path;
std::string _param_path_data;
std::ifstream read_param;
static GeographicLib::LocalCartesian local_cartesian;

static void load_config_param(const std::string& param_file, gnssinfo& init_gnssinfo_)
{
    read_param.open(param_file);
    int flag = 0;
    //eof()函数 用于判断是否为空或者到文件结尾
    while(!read_param.eof())
    {
        std::string s;
        std::getline(read_param,s);

        if(!s.empty() && flag == 1)
        {
            std::stringstream ss;
            ss << s;
            ss >> init_gnssinfo_.latitude;
            ss >> init_gnssinfo_.longitude;
            ss >> init_gnssinfo_.altitude; 
        }
        if(!s.empty() && flag == 2)
        {
            std::stringstream ss;
            ss << s;
            ss >> init_gnssinfo_.roll;
            ss >> init_gnssinfo_.pitch;
            ss >> init_gnssinfo_.yaw;
        }
        flag++;
    }
    read_param.close();
}



void subCallback(const raw_sensor_msgs::ChanavGnss::ConstPtr &msg)
{
    // ROS_INFO("I recevied the topic: ");
   double latitude,longitude,altitude;
   double real_x,real_y,real_z;
    latitude = msg->latitude;
    longitude= msg->longitude;
    altitude = msg->altitude;
 
    local_cartesian.Forward(latitude,longitude,altitude,real_x,real_y,real_z);
    // std::cout<<"x=  "<<real_x <<",y=  "<<real_y<<",z=  "<<real_z<<std::endl;
    point2d.x = real_x;    //将ROS消息变量赋值给结构体变量
    point2d.y = real_y;
    point2d.z = real_z;
    point2d.yaw = msg->yaw;
    point2d.roll = msg->roll;
    point2d.pitch = msg->pitch;
    
    points.push_back(point2d);
}

void WriteFileTxt();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "record_points_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    private_nh.getParam("gps_topic", _gps_topic);
    //private_nh.getParam("filename", _filename);
    //std::cout<<"filename： "<<_filename<<std::endl;
   //设置起点
    private_nh.getParam("param_path",_param_path);
    private_nh.getParam("param_path_data",_param_path_data);
    load_config_param(_param_path.c_str(),init_gnssinfo);
    std::cout<<"beforepoints.size:"<< points.size()<<std::endl;
    local_cartesian.Reset(init_gnssinfo.latitude, init_gnssinfo.longitude, init_gnssinfo.altitude);
    ros::Subscriber gnss_ins_sub = nh.subscribe(_gps_topic, 1000, subCallback);
     std::cout<<"afterpoints.size: "<< points.size()<<std::endl;
     ros::spin();
    WriteFileTxt();
    return 0;
}

//筛选函数的实现

void WriteFileTxt()
{   
    double temp_x = 0;
    double temp_y = 0;
    double temp_yaw = 0;
    int k = 0;
    double index_x,index_y;
    double delta_x,delta_y,delta;

    for(int i=0;i<points.size();i++)
    {
        if(after_point2d.temp_x == 0 && after_point2d.temp_y ==0)
        {
            after_point2d.temp_x = points[k].x;
            after_point2d.temp_y = points[k].y; 
            after_point2d.temp_z = points[k].z;
            after_point2d.temp_yaw = points[k].yaw;
            k++;
            after_points.push_back(after_point2d);//将第一个点装进vector<after_Point2d>
        }
        else
        {
            index_x = points[i].x;
            index_y = points[i].y;
            delta_x = index_x - after_point2d.temp_x;
            delta_y = index_y - after_point2d.temp_y;
            delta = sqrt(delta_x*delta_x + delta_y*delta_y);
            std::cout<<delta<<std::endl;
            if(delta >=0.2 && delta < 0.5)
            {
                after_point2d.temp_x = points[i].x;
                after_point2d.temp_y = points[i].y;
                after_point2d.temp_yaw = points[i].yaw;
                after_points.push_back(after_point2d);
            }
        }
    }
    std::ofstream outfile;
    outfile.open(_param_path_data.c_str(),std::ios::app);
    if(!outfile)
    {
        std::cerr<<"open error!"<<std::endl;
        exit(1);
    }
    
    for(int i=0;i<after_points.size();i++)
    {
        std::stringstream ss1,ss2,ss3,ss4;
        ss1<<std::setprecision(15)<<after_points[i].temp_x;
        ss2<<std::setprecision(15)<<after_points[i].temp_y;
        ss3<<std::setprecision(15)<<after_points[i].temp_z;
        ss4<<std::setprecision(15)<<after_points[i].temp_yaw;
        std::cout<<"x=  "<< points[i].x <<",y=  "<<  points[i].y<<",z=  "<<  points[i].z<<std::endl;

        // ss5<<std::setprecision(15)<<after_points[i].temp_pitch;  
        // ss6<<std::setprecision(15)<<after_points[i].temp_roll; 
        outfile<<ss1.str()<<"   "<<ss2.str()<<"   "<<ss3.str()<<"   "<<ss4.str()<<std::endl;
    }
    outfile.close();
}
