cmake_minimum_required(VERSION 3.5)
project(eagleye_navigation)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(eagleye_coordinate REQUIRED)
find_package(PkgConfig REQUIRED)

ament_auto_find_build_dependencies()

include_directories(include)

ament_auto_add_library(eagleye_navigation SHARED
  src/angular_velocity_offset_stop.cpp
  src/distance.cpp
  src/enable_additional_rolling.cpp
  src/rolling.cpp
  src/heading_interpolate.cpp
  src/heading.cpp
  src/height.cpp
  src/position_interpolate.cpp
  src/position.cpp
  src/slip_angle.cpp
  src/slip_coefficient.cpp
  src/smoothing.cpp
  src/trajectory.cpp
  src/velocity_scale_factor.cpp
  src/yaw_rate_offset_stop.cpp
  src/yaw_rate_offset.cpp
  src/rtk_dead_reckoning.cpp
  src/rtk_heading.cpp
  src/velocity_estimator.cpp
  include/eagleye_navigation/eagleye_navigation.hpp
)

ament_auto_find_build_dependencies()

install(TARGETS eagleye_navigation
  INCLUDES DESTINATION include
)

ament_target_dependencies(eagleye_navigation)

install(TARGETS eagleye_navigation
  DESTINATION lib/${PROJECT_NAME}
)

target_include_directories(eagleye_navigation
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_export_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

ament_export_libraries(eagleye_navigation)

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

install(TARGETS
  eagleye_navigation
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
