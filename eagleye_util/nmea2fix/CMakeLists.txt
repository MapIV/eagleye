cmake_minimum_required(VERSION 2.8.3)
project(nmea2fix)

set(CMAKE_CXX_FLAGS "-O2 -std=c++11 -Wall")

find_package(catkin REQUIRED COMPONENTS
  eagleye_msgs
  rtklib_msgs
  roscpp
  rospy
  std_msgs
  tf
  coordinate
)

catkin_package(
  CATKIN_DEPENDS
  eagleye_msgs
  rtklib_msgs
  roscpp
  rospy
  std_msgs
  coordinate
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(nmea2fix src/nmea2fix.cpp)
target_link_libraries(nmea2fix ${catkin_LIBRARIES} )
add_dependencies(nmea2fix ${catkin_EXPORTED_TARGETS})
