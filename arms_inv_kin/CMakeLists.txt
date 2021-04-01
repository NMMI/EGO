cmake_minimum_required(VERSION 2.8.3)
project(arms_inv_kin)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  kdl_parser
  kdl_conversions
  roscpp
  tf
  rospy
  std_msgs
  ros_myo
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
 CATKIN_DEPENDS 
 	kdl_parser 
 	roscpp 
 	rospy 
 	std_msgs
 	ros_myo
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  include/arms_inv_kin
  include/utils
)

add_executable(inv_kin_node src/main_inv_kin.cpp src/inv_kin.cpp )
add_dependencies(inv_kin_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(inv_kin_node ${catkin_LIBRARIES})

add_executable(inv_kin_gravity_comp_node src/main_inv_kin_gravity_comp.cpp src/inv_kin_gravity_comp.cpp )
add_dependencies(inv_kin_gravity_comp_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(inv_kin_gravity_comp_node ${catkin_LIBRARIES})





