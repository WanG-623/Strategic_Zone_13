#include "control.h"

int main(int argc, char **argv)
{
     ros::init(argc, argv, "control");
     control test; // 创建类的对象
     ros::Rate loop_rate(10);
     while(ros::ok()){
        ros::spinOnce();
        test.run();
        loop_rate.sleep();
    }
}