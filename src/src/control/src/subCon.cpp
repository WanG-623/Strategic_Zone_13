#include "control.h"

void control::loadPath(sensor_msgs::PointCloud2 &cloud)
{
    pcl::fromROSMsg(cloud, pcl_cloud_); //将cloud 类型的消息转换为 pcl_cloud_ 定义的类型
    path.clear();
    for(auto point : pcl_cloud_.points){   //遍历 pcl_cloud_ 中的所有点
        if (point.x<0) //过滤掉X小于0的点
        continue;
        path.push_back(Point(point));  // 将Point类型的点储存到path变量中
        std::cout<<"path.x: "<<Point(point).x<<"path.y: "<<Point(point).y<<std::endl;
    }
}
void control::loadTwist(nav_msgs::Odometry &msg)
{
    std::cout<<"***"<<std::endl;
}
  void control::pathHandler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    path_=*msg;
    std::cout<<"1"<<std::endl;
}
void control::twistHandler(const nav_msgs::Odometry::ConstPtr &msg)
{
   twist_=*msg;
   std::cout<<"2"<<std::endl;
}
// 定义主控制函数 run()
void control::run()
{
    //loadTwist(twist_);
    try{
            // 尝试获取车辆当前的地图位置和位姿信息 
            listener_.lookupTransform("map","base_link",ros::Time(0),transForm_);
        }
    catch(tf::TransformException ex){
            //ROS_ERROR("%s",ex.what());
        }
    int stop_point=0;
    // 调用loadPath()---将ROS话题中的sensor_msgs::PointCloud2类型的数据-
    // -转化为pcl::PointCloud<pcl::PointXYZI>类型，并存储到path变量中    
    loadPath(path_);   
    double point_a=1;
    double point_b=1;
    if(path.size()>8)  // 判断path中的点是否大于8
    {
        point_a=path.at(3).y;  // 将path中的第4个点的y坐标赋值
        point_b=path.at(6).y;  // 将path中的第7个点的y坐标赋值
    }
    // 将车辆当前的位置姿态信息转化为geometry_msgs::Pose类型的消息，并赋值给cur_weizhi变量
    tf::poseTFToMsg(transForm_,cur_weizhi.pose.pose);  
    if(abs(cur_weizhi.pose.pose.position.x-turn_point_x)<0.5&&abs(cur_weizhi.pose.pose.position.y-turn_point_y)<0.5&&abs(point_a)>0.5&&abs(point_b)>0.5)
    {
        stop_point=1;
        std::cout<<"****/n";
    }
    if(stop_point==1)
    {
        turn(); // 转弯
    }
    else
    {
        curve(); // 直行
    }
}
double target_turn_last,target_speed_last=0.0;
// 定义转弯函数 turn()
void control::turn()
{
    double x_idel_pose,y_idel_pose;
    double target_turn,target_speed;
    geometry_msgs::Twist twist;
    if(path.size()>5) // 判断path中的点是否大于5
    {
        x_idel_pose=path.at(4).x;
        y_idel_pose=path.at(4).y;
        target_speed=10;
        target_turn=y_idel_pose;
        // 控制命令--角速度赋值
        twist.angular.z =target_turn;
        // 控制命令--线速度赋值
        twist.linear.x =0;
        twist.linear.z =1; // ？
    }
    else
    {    
        y_idel_pose=path.back().y; // 将path中的末位点的y坐标赋值
        target_turn=y_idel_pose;
        while(abs(target_turn)>0.5)
        {
            target_turn=target_turn/2;
        }
        twist.angular.z =target_turn;
        twist.linear.x=0;
        twist.linear.z =1;
    }
     if(twist.angular.z!=twist.angular.z||twist.linear.x!=twist.linear.x)
    {
        twist.linear.x=0;
        twist.angular.z=0;
    }
    if(1){  //发布车辆速度
        pubTwist.publish(twist); // 发布
    }
}
// 定义直行函数 curve()
void control::curve()
{   std::cout<<"path.size"<<path.size()<<std::endl;
    if(path.size()>11) // 如果路径点大于11，则进行直线控制
    {
        int index=path.size();
        // 筛选第二个点为小车执行规划起点
        double x_init_pose=path.at(1).x;
        double y_init_pose=path.at(1).y;
        std::cout<<"x de weizhi"<<x_init_pose<<std::endl;
        std::cout<<"y de weizhi"<<y_init_pose<<std::endl;
        // 选取路径中的10个点，并将其x和y坐标储存在向量x_1,y_1中
        Eigen::VectorXd x_1(10), y_1(10); // 定义列向量
    // 利用最小二乘法拟合出一条二次曲线
        // 创建一个10行4列的矩阵A，用于存储最小二乘法中的系数
        Eigen::MatrixXd A(10,4), C, T; // 定义矩阵
        // 向量赋值
        for(int i=0; i<10; i++){
            x_1(i) = path.at((i+1)).x;
            y_1(i) = path.at((i+1)).y;
        }
	
        // 矩阵赋值 将10个路径点的x的不同次幂作为系数存储到矩阵A中
        for(int i=0; i<10; i++){
            for(int j=0; j<4; j++){
                A(i,j) = pow(x_1(i), j);
            }
        }
        C = (A.transpose()*A).inverse();
        T = C*A.transpose()*y_1;
    // 计算曲线上的一个点的理想位置，并计算出在该点上的横向误差和纵向误差
        // 获取路径末点的索引，如果超过30则将索引设为30
        int idx_=path.size()-1;
        if(idx_>30)
        idx_=30;
        // 计算曲线上一个点的理想位置
        double x_ideal_pose=1;
        double y_ideal_pose=T(0,0)+T(1,0)*x_ideal_pose+T(2,0)*x_ideal_pose*x_ideal_pose+T(3,0)*x_ideal_pose*x_ideal_pose*x_ideal_pose;    // jinghua_shihao
        //   std::cout<<T<<std::endl;
        // 计算在该点上的横向误差和纵向误差
        double LatDis_Err=y_ideal_pose;
        double LonDis_Err=x_ideal_pose;
        double Dis_Err=sqrt(LatDis_Err*LatDis_Err+LonDis_Err*LonDis_Err)/3.>1?1:sqrt(LatDis_Err*LatDis_Err+LonDis_Err*LonDis_Err)/3;
        // 将计算得到的误差作为目标速度和目标转向角度
        target_speed=Dis_Err;
        target_turn=LatDis_Err*1.5/(target_speed)/10.;
    }
    // 打印目标速度、目标转向角度
    std::cout<<"target_speed"<<std::endl;
    std::cout<<target_speed<<std::endl;
    std::cout<<target_turn<<std::endl;
    // 发布控制指令  
     geometry_msgs::Twist twist;
    //twist.angular.z =0;
    twist.angular.z =0.6*target_turn+0.4*target_turn_last;
    target_turn_last=twist.angular.z;
    twist.linear.x=0.2*target_speed+0.8*target_speed_last;
    target_speed_last=twist.linear.x;
    twist.linear.z =1; // z方向线速度赋值
    if(twist.angular.z!=twist.angular.z||twist.linear.x!=twist.linear.x)
    {
        twist.linear.x=0;
        twist.angular.z=0;
    }
   if(1){ // 发布车辆控制速度
        pubTwist.publish(twist);
    }
}  
