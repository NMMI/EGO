cmake_minimum_required(VERSION 2.8.3)
project(segway_base_controller)

set(CMAKE_BUILD_TYPE Release)


find_package(catkin REQUIRED COMPONENTS
  qb_interface
  roscpp
  rospy
  std_msgs
  kdl_parser
  kdl_conversions
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES segway
#  CATKIN_DEPENDS qb_interface roscpp rospy std_msgs
  DEPENDS qb_interface
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include/segway
  include/utils
)


add_executable(Sensor_node src/main_sensor.cpp src/Sensor.cpp )
add_dependencies(Sensor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Sensor_node ${catkin_LIBRARIES})

add_executable(lqr_node src/main_lqr.cpp src/lqr.cpp )
add_dependencies(lqr_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lqr_node ${catkin_LIBRARIES})

add_executable(pitch_loop_node src/main_pitch_loop.cpp )
add_dependencies(pitch_loop_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pitch_loop_node ${catkin_LIBRARIES})

add_executable(odometry_publisher_node src/main_odometry_publisher.cpp )
add_dependencies(odometry_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(odometry_publisher_node ${catkin_LIBRARIES})
