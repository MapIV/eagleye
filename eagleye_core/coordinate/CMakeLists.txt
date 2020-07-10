cmake_minimum_required(VERSION 2.8.3)
project(coordinate)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geodesy
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
  src/llh2xyz.cpp
  src/xyz2enu.cpp
  src/xyz2enu_vel.cpp
  src/ll2xy_mgrs.cpp

)

target_link_libraries(coordinate
  ${catkin_LIBRARIES}
)
