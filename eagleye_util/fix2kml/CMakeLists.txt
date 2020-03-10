cmake_minimum_required(VERSION 2.8.3)
project(fix2kml)

find_package(catkin REQUIRED COMPONENTS
  eagleye_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package(
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(fix2kml src/fix2kml.cpp)
target_link_libraries(fix2kml ${catkin_LIBRARIES})
