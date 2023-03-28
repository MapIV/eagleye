cmake_minimum_required(VERSION 2.8.3)
project(eagleye_pp)

SET(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  std_msgs
  geometry_msgs
  sensor_msgs
  rtklib_msgs
  eagleye_msgs
  eagleye_navigation
  eagleye_coordinate
  eagleye_gnss_converter
  multi_rosbag_controller
  kml_generator
  tf2
  tf2_geometry_msgs
)

catkin_package(
   CATKIN_DEPENDS
   roscpp
   rosbag
   std_msgs
   geometry_msgs
   sensor_msgs
   rtklib_msgs
   eagleye_msgs
   eagleye_navigation
   eagleye_coordinate
   eagleye_gnss_converter
   multi_rosbag_controller
   kml_generator
   tf2
   tf2_geometry_msgs
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_executable(eagleye_pp
  src/eagleye_pp.cpp
  src/eagleye_pp_core.cpp
  src/eagleye_pp_output.cpp
  )
target_link_libraries(eagleye_pp ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(eagleye_pp ${catkin_EXPORTED_TARGETS})
