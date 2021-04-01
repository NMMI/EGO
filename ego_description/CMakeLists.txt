cmake_minimum_required(VERSION 2.8.3)
project(ego_description)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  urdf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ego_description
#  CATKIN_DEPENDS roscpp rospy std_msgs urdf
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

