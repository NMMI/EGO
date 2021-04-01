cmake_minimum_required(VERSION 2.8.3)
project(zed_oculus_qb_bridge)

set(CMAKE_BUILD_TYPE Release)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp roslib qb_interface tf image_transport cv_bridge)
find_package(orocos_kdl REQUIRED)
#find_package(OpenCV COMPONENTS core highgui imgproc videoio imgcodecs REQUIRED)
find_package( OpenCV REQUIRED )

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
	include/zed_oculus_qb_bridge
	)


add_executable(Joy_oculus_bridge_node src/main_Joy_bridge.cpp src/Joy_oculus_bridge.cpp )
add_dependencies(Joy_oculus_bridge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Joy_oculus_bridge_node ${catkin_LIBRARIES})

add_executable(head_bridge_node src/main_head_bridge.cpp)
add_dependencies(head_bridge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(head_bridge_node ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${OpenCV_LIBRARIES})

# add_executable(rviz_zed_oculus_interface src/rviz_zed_oculus_interface.cpp src/udp_interface/udp_client.cpp)
# add_dependencies(rviz_zed_oculus_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(rviz_zed_oculus_interface ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

