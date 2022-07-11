cmake_minimum_required(VERSION 3.5)
project(eagleye_msgs)

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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AccXOffset.msg"
  "msg/AccXScaleFactor.msg"
  "msg/AccYOffset.msg"
  "msg/AngularVelocityOffset.msg"
  "msg/Distance.msg"
  "msg/Heading.msg"
  "msg/Height.msg"
  "msg/Pitching.msg"
  "msg/Position.msg"
  "msg/Rolling.msg"
  "msg/SlipAngle.msg"
  "msg/Status.msg"
  "msg/StatusStamped.msg"
  "msg/VelocityScaleFactor.msg"
  "msg/YawrateOffset.msg"
  DEPENDENCIES std_msgs
  DEPENDENCIES sensor_msgs
  DEPENDENCIES geometry_msgs
  DEPENDENCIES rtklib_msgs
  DEPENDENCIES builtin_interfaces
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
