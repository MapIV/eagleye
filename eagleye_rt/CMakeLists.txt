cmake_minimum_required(VERSION 3.5)
project(eagleye_rt)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
# find_package(navigation REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
ament_auto_find_build_dependencies()

include_directories(include)

ament_auto_add_library(eagleye_rt
  src/angular_velocity_offset_stop_node.cpp
  src/correction_imu.cpp
  src/distance_node.cpp
  src/heading_interpolate_node.cpp
  src/heading_node.cpp
  src/height_node.cpp
  src/monitor_node.cpp
  src/position_interpolate_node.cpp
  src/position_node.cpp
  src/slip_angle_node.cpp
  src/slip_coefficient_node.cpp
  src/smoothing_node.cpp
  src/trajectory_node.cpp
  src/velocity_scale_factor_node.cpp
  src/yawrate_offset_node.cpp
  src/yawrate_offset_stop_node.cpp
)

install(TARGETS 
  eagleye_rt
  DESTINATION lib/$(PROJECT_NAME)
)

install(DIRECTORY src config launch
  DESTINATION share/${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
