/*
 * @Description: 1.车型　画线小车
                 2.本节点记录车辆起始点的位置LLA，并将此lla设为ENU坐标原点，接收实时数据进行lla转平面坐标;
 * @Author: wanglili
 * @Date: 2023-05-04 14:57:27
 */
#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <iomanip>
// #include "raw_sensor_msgs/ASENSING.h"
//gps驱动头文件
// #include"chcnav/hcinspvatzcb.h"
#include "raw_sensor_msgs/ChanavGnss.h"
std::ofstream save_param;
static std::string _param_path; //定义初始参数路径

static std::string _gnss_ins_src;

static bool has_set_init_lla = false;

void fix_subCallback(const raw_sensor_msgs::ChanavGnss::ConstPtr& msg){
    double latitude,longitude,altitude;
    double init_roll,init_pitch,init_yaw;

    latitude = msg->latitude;
    longitude= msg->longitude;
    altitude = msg->altitude;
    //std::cout<<"latitude: "<<latitude<<std::endl;
    if(!has_set_init_lla){
        has_set_init_lla=true;

        init_roll = msg->roll;
        init_pitch = msg->pitch;
        init_yaw = -msg->yaw;

        save_param<<"# 初始GNSS信息：latitude longitude altitude && 车体与ENU初始方位角度信息: init_roll init_pitch init_yaw(与正东夹角)"<<std::endl
                  <<" "<<std::fixed <<std::setprecision(15)<<latitude<<"  "<<longitude<<"  "<<altitude<<"  "<<std::endl
                  <<" "<<std::fixed <<std::setprecision(15)<<init_roll<<"  "<<init_pitch<<"  "<<init_yaw<<"  "<<std::endl;
        save_param.close();
        ROS_INFO("LLA && init_roll init_pitch init_yaw has be record!");
    }
    
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rtk_record_origin_hxc_node");

    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;

    private_nh.getParam("gnss_ins_src", _gnss_ins_src);
    //std::cout<<"gnss_topic: "<<_gnss_ins_src<<std::endl;
    private_nh.getParam("param_path",_param_path);
    std::cout<<"_param_path: "<<_param_path<<std::endl;
    save_param.open(_param_path.c_str(),std::ios::out | std::ios::trunc);

    ros::Subscriber gnss_ins_sub = nh.subscribe(_gnss_ins_src, 100, fix_subCallback);
    
    ros::spin();

    return 0;
}
