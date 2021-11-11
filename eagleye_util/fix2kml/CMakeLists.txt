cmake_minimum_required(VERSION 2.8.3)
project(eagleye_fix2kml)

set(CMAKE_CXX_FLAGS "-O2 -std=c++11 -Wall")

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
add_dependencies(fix2kml ${catkin_EXPORTED_TARGETS})

install(TARGETS
  fix2kml
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)