cmake_minimum_required(VERSION 3.5)
project(navigation)

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
find_package(coordinate REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
ament_auto_find_build_dependencies()

include_directories(include)

ament_auto_add_library(navigation
  src/angular_velocity_offset_stop.cpp
  src/distance.cpp
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
  src/yawrate_offset_stop.cpp
  src/yawrate_offset.cpp
)

install(TARGETS navigation
  INCLUDES DESTINATION include
)

ament_target_dependencies(navigation)

install(TARGETS navigation
  DESTINATION lib/${PROJECT_NAME}
)

target_include_directories(navigation
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_export_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

install(TARGETS
  navigation
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

# install(DIRECTORY include/coordinate
#   TARGETS coordinate
#   INCLUDES DESTINATION include
# )

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
