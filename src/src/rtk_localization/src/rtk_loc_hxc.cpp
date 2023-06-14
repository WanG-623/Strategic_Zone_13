/*
 * @Description: １. 加载建图时车辆坐标系LLA、RPY(与此时东北天坐标系夹角)；
                 ２. 计算实时车辆坐标系base_link与建图起点base_link坐标系的变换矩阵T；
 * @Author: wanglili
 * @Date: 2023-05-04 15:30:27
 */

#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <iomanip>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <LocalCartesian.hpp>
#include <Geocentric.hpp>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include "raw_sensor_msgs/ChanavGnss.h"

struct pose
{
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};
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

std::ifstream read_param;
static std::string filename;

std::string _gnss_ins_src;
std::string _param_path;

static GeographicLib::LocalCartesian local_cartesian;

Eigen::Matrix4d T_enu2baseReal = Eigen::Matrix4d::Identity();

geometry_msgs::PoseStamped  pose_stamp;
nav_msgs::Path              rtk_path;
static ros::Publisher  rtk_path_pub;

bool is_first = true;

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

void BuildMatrix(pose& pose_in, Eigen::Matrix4d& transform_matrix){
    Eigen::Translation3d tl(pose_in.x,pose_in.y,pose_in.z);
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(pose_in.roll,Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(pose_in.pitch,Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(pose_in.yaw,Eigen::Vector3d::UnitZ()));
    transform_matrix = (tl*yawAngle*pitchAngle*rollAngle).matrix();
}

void fix_subCallback(const raw_sensor_msgs::ChanavGnss::ConstPtr& msg){
    
    pose pose_enu2basereal;
    double latitude,longitude,altitude;
    double real_x,real_y,real_z;
    double real_roll,real_pitch,real_yaw;
     latitude = msg->latitude;
     longitude= msg->longitude;
     altitude = msg->altitude;
     local_cartesian.Forward(latitude,longitude,altitude,real_x,real_y,real_z);
    // std::cout<<"x= %d, y= %d, z=%d"<<real_x<<real_y<<real_z<<std::endl;
    // std::cout<<"x=  "<<real_x <<",y=  "<<real_y<<",z=  "<<real_z<<std::endl;
    real_roll = ((msg->roll)/180)*M_PI;
    real_pitch = ((msg->pitch)/180)*M_PI;
    real_yaw =- ((msg->yaw)/180)*M_PI;
    // std::cout<<"origin_roll=  "<<real_roll <<",origin_pitch=  "<<real_pitch<<",origin_yaw=  "<<real_yaw<<",origin_x=  "<<real_x <<",origin_y=  "<<real_y<<",origin_z=  "<<real_z<<std::endl;
    pose_enu2basereal.x = real_x;
    pose_enu2basereal.y = real_y;
    pose_enu2basereal.z = real_z;
    pose_enu2basereal.roll = real_roll;
    pose_enu2basereal.pitch = real_pitch;
    pose_enu2basereal.yaw = real_yaw;
  BuildMatrix(pose_enu2basereal, T_enu2baseReal);
    
    //实时的base_link到建图起点的变换
    // T_baseReal2baseOrigin= T_enu2baseReal;
    Eigen::Quaterniond q(T_enu2baseReal.block<3, 3>(0, 0));
    Eigen::Vector3d trans = T_enu2baseReal.block<3, 1>(0, 3);

    pose_stamp.header.frame_id = "map";
    pose_stamp.header.stamp = ros::Time::now();
    pose_stamp.pose.position.x = trans[0];
    pose_stamp.pose.position.y = trans[1];
    pose_stamp.pose.position.z = trans[2];
    pose_stamp.pose.orientation.x = q.x();
    pose_stamp.pose.orientation.y = q.y();
    pose_stamp.pose.orientation.z = q.z();
    pose_stamp.pose.orientation.w = q.w();

    //std::cout<<"path_x: "<<trans[0]<<"  path_y: "<<trans[1]<<" path_z: "<<trans[2]<<std::endl;
    
    rtk_path.header.frame_id = "map";
    rtk_path.header.stamp = ros::Time::now();
    rtk_path.poses.push_back(pose_stamp);
    rtk_path_pub.publish(rtk_path);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q_;
    
    transform.setOrigin(tf::Vector3(real_x,real_y, real_z));
    q_.setRPY(real_roll,real_pitch, real_yaw);
    transform.setRotation(q_);

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rtk_loc_hxc_node");
    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;
    private_nh.getParam("gnss_ins_src", _gnss_ins_src);
    private_nh.getParam("param_path",_param_path);
    load_config_param(_param_path.c_str(),init_gnssinfo);
    local_cartesian.Reset(init_gnssinfo.latitude, init_gnssinfo.longitude, init_gnssinfo.altitude);
    // std::cout << "latitude=  "<< init_gnssinfo.latitude <<",longitude=   "<< init_gnssinfo.longitude<<",altitude=  "<<init_gnssinfo.altitude<< std::endl;
    ros::Subscriber gnss_ins_sub = nh.subscribe(_gnss_ins_src, 100, fix_subCallback);
    rtk_path_pub = nh.advertise<nav_msgs::Path>("rtk_path",100);
    ros::spin();
    
    return 0;
}
