cmake_minimum_required(VERSION 2.8.3)
project(qb_frank_controller)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp roslib)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(manager_node src/qb_manager_node.cpp src/qbmove_communications.cpp )
add_dependencies(manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(manager_node ${catkin_LIBRARIES})