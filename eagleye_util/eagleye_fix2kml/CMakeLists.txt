cmake_minimum_required(VERSION 2.8.3)
project(eagleye_fix2kml)

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

add_executable(eagleye_fix2kml src/eagleye_fix2kml.cpp)
add_dependencies(eagleye_fix2kml ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(eagleye_fix2kml ${catkin_LIBRARIES})
