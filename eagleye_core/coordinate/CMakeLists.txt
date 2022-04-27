cmake_minimum_required(VERSION 2.8.3)
project(eagleye_coordinate)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  geodesy
)

find_package(PkgConfig)
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PAHT_SUFFIXES GeographicLib
)
set(GeographicLib_LIBRARIES
  NAMES Geographic
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES coordinate
)

include_directories(
  include
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)
link_directories(
  /usr/local/lib
)

add_library(coordinate
  src/ecef2llh.cpp
  src/enu2llh.cpp
  src/enu2xyz_vel.cpp
  src/ll2xy.cpp
  src/llh2xyz.cpp
  src/xyz2enu.cpp
  src/xyz2enu_vel.cpp
  src/ll2xy_mgrs.cpp
  src/geoid_per_degree.cpp
  src/geoid_per_minute.cpp
  src/convertheight.cpp

)

target_link_libraries(coordinate
  ${catkin_LIBRARIES}
  Geographic
)
add_dependencies(coordinate ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/coordinate/
  DESTINATION include/coordinate
  FILES_MATCHING PATTERN "*.hpp"
)

install(
  DIRECTORY
    data
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS coordinate
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
