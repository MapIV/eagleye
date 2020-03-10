cmake_minimum_required(VERSION 2.8.3)
project(fix2pose)

find_package(eagleye_core)

find_package(catkin REQUIRED COMPONENTS
  eagleye_msgs
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS
  eagleye_msgs
  eagleye_core
  roscpp
  rospy
  std_msgs
)

include_directories(
  ${eagleye_core_SOURCE_PREFIX}/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(fix2pose src/fix2pose.cpp)
target_link_libraries(fix2pose ${catkin_LIBRARIES} )

