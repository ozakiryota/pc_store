cmake_minimum_required(VERSION 2.8.3)
project(pc_store)

add_compile_options(-std=c++14 -O2 -g -Wall)

find_package(catkin REQUIRED
  roscpp
  tf
)
find_package(PCL)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(pc_store_with_odometry src/pc_store_with_odometry.cpp)
target_link_libraries(pc_store_with_odometry
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(pc_leave_with_odometry src/pc_leave_with_odometry.cpp)
target_link_libraries(pc_leave_with_odometry
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(lower_pc_hz src/lower_pc_hz.cpp)
target_link_libraries(lower_pc_hz
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

#############
## Install ##
#############

#############
## Testing ##
#############