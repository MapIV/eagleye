cmake_minimum_required(VERSION 3.5)
project(eagleye_coordinate)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

find_package(PkgConfig)
find_package(geodesy)
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PAHT_SUFFIXES GeographicLib
)
set(GeographicLib_LIBRARIES
  NAMES Geographic
)

include_directories(
  include
  ${PROJECT_SOURCE_DIR}/include
  ${GeographicLib_INCLUDE_DIRS}
)
link_directories(
  /usr/local/lib
)


ament_auto_add_library(eagleye_coordinate SHARED
  src/ecef2llh.cpp
  src/enu2llh.cpp
  src/enu2xyz_vel.cpp
  src/ll2xy.cpp
  src/llh2xyz.cpp
  src/xyz2enu_vel.cpp
  src/xyz2enu.cpp
  src/geoid_per_degree.cpp
  src/geoid_per_minute.cpp
  src/convertheight.cpp
  include/eagleye_coordinate/eagleye_coordinate.hpp
)

ament_export_include_directories(include)
ament_target_dependencies(eagleye_coordinate)

target_link_libraries(eagleye_coordinate
  Geographic
)

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)


target_include_directories(eagleye_coordinate
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_export_libraries(eagleye_coordinate)

install(
  TARGETS eagleye_coordinate
  EXPORT export_coordinate
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

install(TARGETS
  eagleye_coordinate
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()