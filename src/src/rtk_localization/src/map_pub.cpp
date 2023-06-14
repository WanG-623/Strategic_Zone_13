 /*
  1.将前后100个点放入消息中发布
*/
#include "ros/ros.h"
#include <iostream>
// #include "PlatformMap/GnssImuInfo.h"
// #include "PlatformMap/LocalMapPoint.h"
// #include "PlatformMap/LocalMapLane.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <vector>
#include <sstream>
#include <fstream>
#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
// #include"chcnav/hcinspvatzcb.h"
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/Geocentric.hpp>
#include<sensor_msgs/PointCloud2.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include "raw_sensor_msgs/ChanavGnss.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

struct Piont_element
{
    double center_x;
    double center_y;
     double center_z;
    double center_yaw;
    double center_pitch;
    double center_roll;
};
std::string _gps_topic;
std::vector<Piont_element> global_map;
Piont_element  gnssinfo;
std::vector<Piont_element> gnssinfos;
//gps
struct Gnssinfo
{
    double latitude;
    double longitude;
    double altitude;
    double roll;
    double pitch;
    double yaw;
};
static Gnssinfo init_gnssinfo;
std::string _param_path;
std::ifstream read_param;
static GeographicLib::LocalCartesian local_cartesian;
//
struct pose
{
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

bool is_first = true;
double last_x,last_y,last_z;
Eigen::Matrix4d T_map2base = Eigen::Matrix4d::Identity();
Eigen::Matrix4d T_base2lidar=Eigen::Matrix4d::Identity();
Eigen::Matrix4d T_map2lidar=Eigen::Matrix4d::Identity();
Eigen::Matrix4d T_lidar2map= Eigen::Matrix4d::Identity();

static void load_config_param(const std::string& param_file, Gnssinfo& init_gnssinfo_)
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
void chatterCallback(const raw_sensor_msgs::ChanavGnss::ConstPtr& msg)
{
    // std::cout<<"回调函数成功"<<std::endl;
    ROS_INFO("I have received the IMU message!");
    double latitude,longitude,altitude;
    double real_x,real_y,real_z;
    pose pose_base2maporigin;
    double real_roll, real_pitch, real_yaw;
    latitude = msg->latitude;
    longitude= msg->longitude;
    altitude = msg->altitude;
     local_cartesian.Forward(latitude,longitude,altitude,real_x,real_y,real_z);
    gnssinfo.center_x = real_x;
    gnssinfo.center_y = real_y;
    gnssinfo.center_yaw = msg->yaw;
    gnssinfos.push_back(gnssinfo);
    std::cout<<gnssinfos.size()<<std::endl;
    if(gnssinfos.size()>50){
        gnssinfos.erase(gnssinfos.begin());
    }
    real_roll = ((msg->roll) / 180) * M_PI;
    real_pitch = ((msg->pitch) / 180) * M_PI;
    real_yaw = -((msg->yaw) / 180) * M_PI;
     if(is_first)
   {
    is_first = false;
    pose_base2maporigin.x = real_x;
    pose_base2maporigin.y = real_y;
    pose_base2maporigin.z = real_z;
    pose_base2maporigin.roll = real_roll;
    pose_base2maporigin.pitch = real_pitch;
    pose_base2maporigin.yaw = real_yaw;
    BuildMatrix(pose_base2maporigin, T_map2base);
    last_x = real_x;
    last_y = real_y;
    last_z = real_z;
   }
   else{
   double delta_x = real_x-last_x;
   double delta_y = real_y-last_y;
   double delta_z = real_z-last_z;
   double deta = sqrt(delta_x*delta_x + delta_y*delta_y+delta_z*delta_z);
   if(deta<0.1)
     {
        pose_base2maporigin.x = real_x;
        pose_base2maporigin.y = real_y;
        pose_base2maporigin.z = real_z;
        pose_base2maporigin.roll = real_roll;
        pose_base2maporigin.pitch = real_pitch;
        pose_base2maporigin.yaw = real_yaw;
        BuildMatrix(pose_base2maporigin, T_map2base);
        last_x = real_x;
        last_y = real_y;
         last_z = real_z;
      }
    else
    {
    last_x = real_x;
    last_y = real_y;
     last_z = real_z;
    }
   }
}
void readtxtmap()
{
    Piont_element point_element;
    double element1,element2,element3,element4,element5,element6;
    std::ifstream infile("/home/promote/catkin_ws/src/rtk_localization/data/huaxian_0602.txt");
    if(!infile.is_open())
    {
        std::cout<<"File is not existing!"<<std::endl;
        exit(1);
    }
    for(int i=0;!infile.eof();i++)
    {
        infile>>element1>>element2>>element3>>element4;
        point_element.center_x = element1;
        point_element.center_y = element2;
        point_element.center_z = element3;
        point_element.center_yaw = element4;
        // point_element.center_pitch= element5;
        // point_element.center_roll = element6;
        // std::cout<<"x:"<<  point_element.center_x<<std::endl;
        global_map.push_back(point_element);
    }
    std::cout<<"global_map.size:"<<global_map.size()<<std::endl;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "map_pub_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    readtxtmap();
  //设置起点
    private_nh.getParam("param_path",_param_path);
    load_config_param(_param_path.c_str(),init_gnssinfo);
    local_cartesian.Reset(init_gnssinfo.latitude, init_gnssinfo.longitude, init_gnssinfo.altitude);
    private_nh.getParam("gps_topic", _gps_topic);
    ros::Subscriber gnss_sub = nh.subscribe<raw_sensor_msgs::ChanavGnss>(_gps_topic,1000,chatterCallback);
    ros::Publisher  map_pub = nh.advertise<sensor_msgs::PointCloud2>("/lane_show", 1000);

        pose pose_base2lidar;
        pose_base2lidar.x = 0;
        pose_base2lidar.y = 0;
        pose_base2lidar.z = 0;
        pose_base2lidar.roll = 0;
        pose_base2lidar.pitch = 0;
        pose_base2lidar.yaw =M_PI/2;
        BuildMatrix(pose_base2lidar, T_base2lidar);

    ros::Rate loop_rate(1);
    while(ros::ok())
    {
        std::cout << "gnssinfos: " << gnssinfos.size() << std::endl;
        // pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        cloud_in_ptr->points.resize(39);
        cloud_out_ptr->points.resize(39);
        // cloud.points.resize(200);
        if(gnssinfos.size() > 0)
        
        {
            int idx = gnssinfos.size()-1;
            double realtime_x = gnssinfos[idx].center_x;
            double realtime_y = gnssinfos[idx].center_y;
            int id;
            int mapLens = global_map.size();
            int p_begin,p_end;
            double delta_x,delta_y,delta;
            double near_distance = 10e10;
            for(int i = 0;i < mapLens;i++)
            {
                delta_x = realtime_x - global_map[i].center_x;
                delta_y = realtime_y - global_map[i].center_y;
                delta = sqrt(delta_x*delta_x + delta_y*delta_y);
                if(delta < near_distance)
                {
                    near_distance = delta;
                    id = i;
                }
            }
            std::cout<<"id"<<id<<std::endl;
            p_begin = id+1;
            //if(id<100){p_begin = 0;} else{p_begin = id - 100;}
            //if(id > mapLens-100){p_end = mapLens;} else{p_end = id+100;}
            // if(id<20){p_begin = 0;} else{p_begin = id ;}
            if(id > mapLens-40){p_end = mapLens;} else{p_end = id+40;}
            sensor_msgs::PointCloud2 output;
            std::cout<<"p_begin"<<p_begin<<std::endl;
            std::cout<<"p_end"<<p_end<<std::endl;
            int j = 0;
            // std::ofstream pose2("/home/lsh/Teaching_platform/wire-barrow/data/pose2.txt", std::ios::app);
            for(int i = p_begin+1;i<p_end;i++)
            {
                // pose pose_map2maporigin;
                // Eigen::Matrix4d T_map2maporigin = Eigen::Matrix4d::Identity();
                // Eigen::Matrix4d T_map2baselink = Eigen::Matrix4d::Identity();
                // pose_map2maporigin.x= global_map[i].center_x;
                // pose_map2maporigin.y= global_map[i].center_y;
                // pose_map2maporigin.z= global_map[i].center_z;
                // pose_map2maporigin.roll = 0;
                // pose_map2maporigin.pitch = 0;
                // pose_map2maporigin.yaw = 0;
                // BuildMatrix(pose_map2maporigin, T_map2maporigin);
                // T_map2baselink=T_map2maporigin*T_base2maporigin.inverse();
                // cloud_ptr->points[j].x = T_map2baselink(0,3);
                // cloud_ptr->points[j].y = T_map2baselink(1,3);
                // cloud_ptr->points[j].z= T_map2baselink(2,3);
                
                // pose2.setf(std::ios::scientific, std::ios::floatfield);
                // // pose1.precision(9);
                // pose2<<"x: "<<cloud_ptr->points[j].x<<"y: "<<cloud_ptr->points[j].y<<"z: "<<cloud_ptr->points[j].z<< std::endl;
                
                cloud_in_ptr->points[j].x=global_map[i].center_x;
                cloud_in_ptr->points[j].y=global_map[i].center_y;
                cloud_in_ptr->points[j].z=global_map[i].center_z;
                // std::cout<<"map_x"<<global_map[i].center_x<<",map_y: "<<global_map[i].center_y<<",map_z: "<<global_map[i].center_y<<std::endl;
                j++;
          

            }
            //  std::cout<<"jjjjjjjjjjjjjjjjjjjjjj:"<<j<<std::endl;

            T_map2lidar=T_map2base*T_base2lidar;
            T_lidar2map=T_map2lidar.inverse();
            
            pcl::transformPointCloud(*cloud_in_ptr, *cloud_out_ptr, T_lidar2map);
            // pose2<<"===================================="<<std::endl;
            // pose2.close();
            // pcl::toROSMsg(cloud,output);
            std::cout<<"cloud_ptr.size: "<<cloud_out_ptr->points.size()<<std::endl;
            pcl::toROSMsg(*cloud_out_ptr,output);
            std::cout<<"base_link_x: "<<cloud_out_ptr->points[0].x<<std::endl;
            output.header.frame_id = "base_link";
            map_pub.publish(output);

        }
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
    
}