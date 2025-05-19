cmake_minimum_required(VERSION 3.0.2)
project(pcd_to_rviz)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()

find_package(PCL 1.7 REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(pcd_to_rviz src/pcd_to_rviz.cpp)
target_link_libraries(pcd_to_rviz 
	${PCL_LIBRARIES}
	${catkin_LIBRARIES})
