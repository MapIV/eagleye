cmake_minimum_required(VERSION 2.8.3)
project(fix2pose)

find_package(catkin REQUIRED COMPONENTS
  eagleye_msgs
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package(
)

include_directories(
  ${eagleye_core_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(fix2pose src/fix2pose.cpp)
add_dependencies(fix2pose ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(fix2pose ${catkin_LIBRARIES})
