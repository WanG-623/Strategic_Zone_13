#include <tf/transform_listener.h>
tf::TransformListener listener_;
tf::StampedTransform transForm_;
nav::msgs::Odometry cur_weizhi;

try{
    listener_.lookupTransform("map","rslidar",ros::Time(0),transForm_);
}
catch(tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
}
tf::poseTFToMsg(transForm_,cur_weizhi_pose.pose);