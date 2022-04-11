cmake_minimum_required(VERSION 2.8.3)
project(multi_rosbag_controller)

# Enable support for C++11
if (${CMAKE_VERSION} VERSION_LESS "3.1.0")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else ()
  set(CMAKE_CXX_STANDARD 11)
endif ()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
)

catkin_package(
  CATKIN_DEPENDS roscpp rosbag
  INCLUDE_DIRS include
  LIBRARIES multi_rosbag_controller
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(multi_rosbag_controller src/multi_rosbag_controller/multi_rosbag_controller.cpp)
target_link_libraries(multi_rosbag_controller ${catkin_LIBRARIES})
add_dependencies(multi_rosbag_controller ${catkin_EXPORTED_TARGETS})
