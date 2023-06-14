/*节点说明：
   1.订阅gnss话题，并每隔0.5m记录一个点;
*/
#include "ros/ros.h"
#include "raw_sensor_msgs/GnssImuInfo.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>


struct Point2d                   //定义接收所有坐标点的结构体变量
{
    double x;
    double y;
    double yaw;
}point2d;

struct after_Ponit2d
{
    double temp_x;
    double temp_y;
    double temp_yaw;
}after_point2d;

std::vector<Point2d> points;     //定义存储所有点坐标的vector<Point2d>类型的点
std::vector<after_Ponit2d>  after_points;

std::string _gps_topic;
//std::string _filename = "/home/lyl/MapMake_ws/src/DoubleLane/data/lane_1.txt";

void subCallback(const raw_sensor_msgs::GnssImuInfo &msg)
{
    ROS_INFO("I recevied the topic: ");

    point2d.x = msg.utm_east;    //将ROS消息变量赋值给结构体变量
    point2d.y = msg.utm_north;
    point2d.yaw = msg.yaw;
    
    points.push_back(point2d);
}

void WriteFileTxt();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Cycle_lane");
    
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.getParam("gps_topic", _gps_topic);
    //private_nh.getParam("filename", _filename);
    //std::cout<<"filename： "<<_filename<<std::endl;

    ros::Subscriber gnss_ins_sub = nh.subscribe(_gps_topic, 1000, subCallback);
    
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

    double index_x,index_y;
    double delta_x,delta_y,delta;

    for(int i=0;i<points.size();i++)
    {
        if(after_point2d.temp_x == 0 && after_point2d.temp_y ==0)
        {
            after_point2d.temp_x = points[0].x;
            after_point2d.temp_y = points[0].y; 
            after_point2d.temp_yaw = points[0].yaw;
            after_points.push_back(after_point2d);//将第一个点装进vector<after_Point2d>
        }
        else
        {
            index_x = points[i].x;
            index_y = points[i].y;
            delta_x = index_x - after_point2d.temp_x;
            delta_y = index_y - after_point2d.temp_y;
            delta = sqrt(delta_x*delta_x + delta_y*delta_y);

            if(delta >=0.5)
            {
                after_point2d.temp_x = points[i].x;
                after_point2d.temp_y = points[i].y;
                after_point2d.temp_yaw = points[i].yaw;
                after_points.push_back(after_point2d);
            }
        }
    }

    std::ofstream outfile;
    
    outfile.open("/home/lyl/MapMake_ws/src/DoubleLane/data/lane.txt",std::ios::out);
    
    if(!outfile)
    {
        std::cerr<<"open error!"<<std::endl;
        exit(1);
    }
    
    for(int i=0;i<after_points.size();i++)
    {
        std::stringstream ss1,ss2,ss3;
        ss1<<std::setprecision(15)<<after_points[i].temp_x;
        ss2<<std::setprecision(15)<<after_points[i].temp_y;
        ss3<<std::setprecision(15)<<after_points[i].temp_yaw;

        outfile<<ss1.str()<<"   "<<ss2.str()<<"   "<<ss3.str()<<std::endl;
    }
    
    outfile.close();
}
