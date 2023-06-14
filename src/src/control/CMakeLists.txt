cmake_minimum_required(VERSION 2.8.3)
project(control)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy

  message_filters

  pcl_ros
  pcl_msgs
  pcl_conversions
  raw_sensor_msgs

  tf

  cv_bridge
  image_transport

  #planning_msgs
  # raw_sensor_msgs

)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

catkin_package(
  INCLUDE_DIRS 
  include
  CATKIN_DEPENDS
  raw_sensor_msgs 
  DEPENDS PCL
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
	include
  ${OpenCV_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
)

add_executable(control src/control.cpp src/subCon.cpp)
target_link_libraries(control ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARY_DIRS})
