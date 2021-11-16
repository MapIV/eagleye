cmake_minimum_required(VERSION 3.5)
project(coordinate)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

include_directories(include)

ament_auto_add_library(coordinate SHARED
  src/ecef2llh.cpp
  src/enu2llh.cpp
  src/hgeoid.cpp
  src/ll2xy_mgrs.cpp
  src/ll2xy.cpp
  src/llh2xyz.cpp
  src/xyz2enu_vel.cpp
  src/xyz2enu.cpp
  include/coordinate/coordinate.hpp
)

ament_export_include_directories(include)
ament_target_dependencies(coordinate)

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)


target_include_directories(coordinate
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_export_libraries(coordinate)

install(
  TARGETS coordinate
  EXPORT export_coordinate
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

install(TARGETS
  coordinate
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
