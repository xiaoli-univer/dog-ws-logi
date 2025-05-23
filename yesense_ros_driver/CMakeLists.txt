cmake_minimum_required(VERSION 2.8.3)
project(yesense_imu)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
#  serial
  std_msgs
  std_srvs
  tf
)

find_package(serial REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mpu6050_serial_to_imu
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs serial std_msgs std_srvs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${serial_INCLUDE_DIRS}
  ./src
)

## Declare a C++ executable
add_executable(yesense_imu_node src/yesense_imu_node.cpp 
src/analysis_data.c)

## Specify libraries to link a library or executable target against
target_link_libraries(yesense_imu_node
  ${catkin_LIBRARIES}
  ${serial_LIBRARIES}
)

add_executable(example src/example.c 
src/analysis_data.c)

#############
## Install ##
#############


## Mark executables and/or libraries for installation
# install(TARGETS yesense_imu_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install (DIRECTORY launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# install (DIRECTORY rviz
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
