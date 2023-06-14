cmake_minimum_required(VERSION 3.0.2)
project(rtk_localization)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  raw_sensor_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2
  tf2_ros
)

find_package(Eigen QUIET)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS 
   include
   CATKIN_DEPENDS geometry_msgs nav_msgs raw_sensor_msgs roscpp rospy sensor_msgs std_msgs tf2 tf2_ros
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
add_subdirectory(/home/promote/catkin_ws/src/rtk_localization/third_party/GeographicLib)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/include/eigen3
  /home/promote/catkin_ws/src/rtk_localization/third_party/GeographicLib/include
  /usr/include/pcl-1.8
)

add_executable(rtk_record_origin_hxc  src/rtk_record_origin_hxc.cpp)
add_dependencies(rtk_record_origin_hxc ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rtk_record_origin_hxc  libGeographiccc ${catkin_LIBRARIES})

add_executable(rtk_loc_hxc  src/rtk_loc_hxc.cpp)
add_dependencies(rtk_loc_hxc ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rtk_loc_hxc libGeographiccc ${catkin_LIBRARIES})

add_executable(record_points  src/record_points.cpp)
add_dependencies(record_points ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(record_points libGeographiccc ${catkin_LIBRARIES})

add_executable(map_pub  src/map_pub.cpp)
add_dependencies(map_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_pub  libGeographiccc ${catkin_LIBRARIES})
