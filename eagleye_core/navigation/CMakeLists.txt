cmake_minimum_required(VERSION 2.8.3)
project(eagleye_navigation)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

find_package(Eigen3 QUIET)
set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  rtklib_msgs
  eagleye_msgs
  eagleye_coordinate
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES navigation
)

include_directories(
  include
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_library(navigation
  src/velocity_scale_factor.cpp
  src/distance.cpp
  src/yaw_rate_offset_stop.cpp
  src/yaw_rate_offset.cpp
  src/heading.cpp
  src/position.cpp
  src/slip_angle.cpp
  src/slip_coefficient.cpp
  src/smoothing.cpp
  src/trajectory.cpp
  src/heading_interpolate.cpp
  src/position_interpolate.cpp
  src/height.cpp
  src/angular_velocity_offset_stop.cpp
  src/rtk_dead_reckoning.cpp
  src/rtk_heading.cpp
  src/enable_additional_rolling.cpp
  src/rolling.cpp
  src/velocity_estimator.cpp
)

target_link_libraries(navigation
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)
add_dependencies(navigation ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/navigation/
  DESTINATION include/navigation/
  FILES_MATCHING PATTERN "*.hpp"
)

install(TARGETS navigation
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
