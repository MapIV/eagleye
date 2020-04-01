cmake_minimum_required(VERSION 2.8.3)
project(coordinate)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES coordinate
)

include_directories(
  include
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

add_library(coordinate
  src/ecef2llh.cpp
  src/enu2llh.cpp
  src/hgeoid.cpp
  src/ll2xy.cpp
  src/xyz2enu.cpp
  src/xyz2enu_vel.cpp
)

target_link_libraries(coordinate
  ${catkin_LIBRARIES}
)
