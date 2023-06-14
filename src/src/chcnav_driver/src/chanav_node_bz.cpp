#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <raw_sensor_msgs/ChanavGnss.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>

#include <string>
#include <iostream>
#include <fstream>
#include <cmath>
#include <stdlib.h>
#include<malloc.h>
#include <algorithm>

#define GPSWeek   2
#define GPSTime   3
#define Heading   4
#define Pitch     5
#define Roll      6
#define Gyro_x    7
#define Gyro_y    8
#define Gyro_z    9
#define Acc_x     10
#define Acc_y     11
#define Acc_z     12
#define Latitude  13
#define Longitude 14
#define Altitude  15
#define VE        16
#define VN        17
#define VU        18
#define Velocity  19
#define NSV1      20
#define NSV2      21
#define Status    22
#define Age       23
#define Space     24

//std::string _port;
int _baud_rate;

serial::Serial ser;

sensor_msgs::Imu imu_msg;
sensor_msgs::NavSatFix gps_msg;
raw_sensor_msgs::ChanavGnss  gnssimuinfo;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "chanav_node");

    ros::NodeHandle nh("chcnav_ros");
    ros::NodeHandle private_nh("~");

    //private_nh.param<std::string>("port", _port, "/dev/ttyUSB0");
    private_nh.param<int>("baud_rate", _baud_rate, 115200);

    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 200);
    ros::Publisher gps_pub = nh.advertise<sensor_msgs::NavSatFix>("fix", 200);
    ros::Publisher gnss_ins_pub = nh.advertise<raw_sensor_msgs::ChanavGnss>("gnss_ins", 1000);

    ros::Rate r(100); 

    std::string read;
    std::string str;
    std::string foundCHC = "$GPCHC";

    while(ros::ok())
    {
        try
        {
            if (ser.isOpen())
            {
                if(ser.available())
                {   //ser.available() 读取到缓存区数据的字节数
                    std::cout<<"******** ok **********"<<std::endl;

                    read = ser.read(ser.available());    //读出缓存区缓存的数据
                    str.append(read);                    //append() 向string后面追加字符或字符串
                    if(str.size()>350)                   //size() 返回字符串长度
                    {
                        unsigned int loc1 = str.find(foundCHC, 0);    //string中find()返回值是字母在母串中的位置（下标记录）
                        if(loc1 > 350)
                        {
                            ROS_INFO("read GPCHC fail!!!!");
                            str.erase();
                            continue;
                        }
                        unsigned int loc3 = str.find(foundCHC, loc1 + 1);
                        if(loc3 > 350)
                        {
                            str.erase();
                            continue;
                        }

                        double gpsweek,gpstime,yaw,pitch,roll,gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z,
                               latitude,longitude,altitude,ve,vn,vu,velocity,nsv1,nsv2,status,age,space;
                        int i = 0;
                        std::string gpchc(str, loc1, loc3 - loc1);
                        char *serial_Ctype;
                        int len = gpchc.length();   //length() 返回字符长度
                        serial_Ctype = (char *)malloc((len + 1)*sizeof(char));  //malloc() 分配长度为len+1所占的字节数
                        gpchc.copy(serial_Ctype,len,0);
                        serial_Ctype[len] = '\0';
                        char *ptr = strtok(serial_Ctype, ",");  //strtok() 分解字符串为一组字符串
                        //第一个参数为要分解的字符串 第二个参数为分隔符

                        while (ptr != NULL && i < Space + 1)
                        {
                            i++;
                            switch(i)
                            {  // atof() 函数将字符串的内容解释为浮点数并将其值作为双精度值返回
                                case GPSWeek:
                                     gpsweek = atoi(ptr);
                                     break;
                                case GPSTime:
                                     gpstime = atof(ptr);
                                     break;
                                case Heading:
                                     yaw = atof(ptr);
                                     break;
                                case Pitch:
                                     pitch = atof(ptr);
                                     break;
                                case Roll:
                                     roll = atof(ptr);
                                     break;
                                case Gyro_x:
                                     gyro_x = atof(ptr);
                                     break;
                                case Gyro_y:
                                     gyro_y = atof(ptr);
                                     break;
                                case Gyro_z:
                                     gyro_z = atof(ptr);
                                     break;
                                case Acc_x:
                                     acc_x = atof(ptr);
                                     break;
                                case Acc_y:
                                     acc_y = atof(ptr);
                                     break;
                                case Acc_z:
                                     acc_z = atof(ptr);
                                     break;
                                case Latitude:
                                     latitude = atof(ptr);
                                     break;
                                case Longitude:
                                     longitude = atof(ptr);
                                     break;
                                case Altitude:
                                     altitude = atof(ptr);
                                     break;
                                case VE:
                                     ve = atof(ptr);
                                     break;
                                case VN:
                                     vn = atof(ptr);
                                     break;
                                case VU:
                                     vu = atof(ptr);
                                     break;
                                case Velocity:
                                     velocity = atof(ptr);
                                     break;
                                case NSV1:
                                     nsv1 = atof(ptr);
                                     break;
                                case NSV2:
                                     nsv2 = atof(ptr);
                                     break;
                                case Status:
                                     status = atoi(ptr);
                                     break;
                                case Age:
                                     age = atof(ptr);
                                     break;
                                case Space:
                                     space = atof(ptr);
                                     break;
                            }
                            ptr = strtok(NULL, ",");
                        }

                        imu_msg.header.stamp = ros::Time::now();
                        imu_msg.header.frame_id = "imu_link";
                        tf::Quaternion imu_orientation;
                        imu_orientation=tf::createQuaternionFromRPY(roll,pitch,yaw);
                        quaternionTFToMsg(imu_orientation, imu_msg.orientation);
                        imu_msg.angular_velocity.x = gyro_x; 
                        imu_msg.angular_velocity.y = gyro_y;
                        imu_msg.angular_velocity.z = gyro_z;
                        imu_msg.linear_acceleration.x = acc_x;
                        imu_msg.linear_acceleration.y = acc_y;
                        imu_msg.linear_acceleration.z = acc_z;
                        imu_pub.publish(imu_msg);

                        gps_msg.header.stamp = ros::Time::now();
                        gps_msg.header.frame_id = "base_link";
                        gps_msg.latitude = latitude;
                        gps_msg.longitude = longitude;
                        gps_msg.altitude = altitude;
                        gps_pub.publish(gps_msg);
                

                        gnssimuinfo.none_gps_data = 0;
                        gnssimuinfo.none_vehicle_data = 0;
                        gnssimuinfo.error_gyroscope = 0;
                        gnssimuinfo.error_accelerometer = 0;

                        gnssimuinfo.header.stamp = ros::Time::now();
                        gnssimuinfo.header.frame_id = "base_link";
                        gnssimuinfo.latitude = latitude;
                        gnssimuinfo.longitude = longitude;
                        gnssimuinfo.altitude = altitude;
                        gnssimuinfo.east_velocity =ve;
                        gnssimuinfo.north_velocity=vn;
                        gnssimuinfo.up_velocity = vu;
                        gnssimuinfo.velocity = velocity;
                        gnssimuinfo.ax = acc_x;
                        gnssimuinfo.ay = acc_y;
                        gnssimuinfo.az = acc_z;
                        gnssimuinfo.roll = roll;
                        gnssimuinfo.pitch = pitch;
                        gnssimuinfo.yaw = yaw;
                        gnssimuinfo.x = imu_msg.orientation.x;
                        gnssimuinfo.y = imu_msg.orientation.y;
                        gnssimuinfo.z = imu_msg.orientation.z;
                        gnssimuinfo.w = imu_msg.orientation.w;
                        gnssimuinfo.nsv1_num = nsv1;
                        gnssimuinfo.nsv2_num = nsv2;
                        gnss_ins_pub.publish(gnssimuinfo);

                        str.erase(0, loc3);
                    }
                }
                else{
                    //ROS_ERROR_STREAM("serial port none data,eg. please check Baudrate " << ser.getPort() << ". Trying again in 5 seconds.");
                    //ros::Duration(5).sleep();
                }
            }
            else
            {
                try
                {
                    ser.setPort("/dev/ttyUSB0");
                    ser.setBaudrate(115200);
                    //ser.setPort(_port);
                    ser.setBaudrate(_baud_rate);
                    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
                    ser.setTimeout(to);
                    ser.open();
                }
                catch (serial::IOException& e)
                {
                    ROS_ERROR_STREAM("Unable to open serial port " << ser.getPort() << ". Trying again in 5 seconds.");
                    ros::Duration(5).sleep();
                }
                if(ser.isOpen())
                {
                    ROS_DEBUG_STREAM("Serial port " << ser.getPort() << " initialized and opened.");
                }
            }
        }
        catch (serial::IOException& e)
        {
            ROS_ERROR_STREAM("Error reading from the serial port " << ser.getPort() << ". Closing connection.");
            ser.close();
        }
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
