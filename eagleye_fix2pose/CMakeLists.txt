cmake_minimum_required(VERSION 2.8.3)
project(eagleye_fix2pose)

find_package(catkin REQUIRED COMPONENTS
  eagleye_msgs
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package(
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(eagleye_fix2pose src/eagleye_fix2pose.cpp)
add_dependencies(eagleye_fix2pose ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(eagleye_fix2pose ${catkin_LIBRARIES})
