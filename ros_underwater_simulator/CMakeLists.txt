cmake_minimum_required(VERSION 2.8.3)
project(ros_underwater_simulator)

find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
message_generation
gazebo_msgs
)

add_message_files(
	FILES
	cmd_pos.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
	gazebo_msgs
)

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build our plugin
add_library(velodyne_plugin SHARED velodyne_plugin.cc)
target_link_libraries(velodyne_plugin ${GAZEBO_LIBRARIES})

catkin_package(CATKIN_DEPENDS message_runtime)

if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()

install(DIRECTORY images meshes launch rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS nodes/subscriber.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)