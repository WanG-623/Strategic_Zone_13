project(chassis_ros)
cmake_minimum_required(VERSION 3.10.0)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-Wall -g -std=c++11 -O0 ")

#set(serial_driver "chassis_driver")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  roslib
  nav_msgs
  tf
)
find_package(socket_can REQUIRED)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/include/
  ${CMAKE_CURRENT_SOURCE_DIR}/../socket_can
  ${CMAKE_CURRENT_SOURCE_DIR}/../socket_can/src
  ${CMAKE_CURRENT_SOURCE_DIR}/../socket_can/include/
  ${socket_can_INCLUDE_DIRS}
)
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src DIR_SRCS)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

add_executable(${PROJECT_NAME}_node ${DIR_SRCS})
target_link_libraries(${PROJECT_NAME}_node 
#m ${serial_driver} 
${catkin_LIBRARIES}
${socket_can_INCLUDE_DIRS}
)

#add_custom_command(
#	OUTPUT copy_lib
#	COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/lib/* ${CATKIN_DEVEL_PREFIX}/lib
#)
#add_custom_target(COPY_CHASSIS_LIB ALL DEPENDS copy_lib ${PROJECT_NAME}_node)

#install(TARGETS ${PROJECT_NAME}_node
 # RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(FILES lib/lib${serial_driver}.so
#  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#)

#install(DIRECTORY launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
