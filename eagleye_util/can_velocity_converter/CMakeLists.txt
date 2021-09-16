cmake_minimum_required(VERSION 2.8.3)
project(eagleye_can_velocity_converter)

find_package(catkin REQUIRED COMPONENTS
  can_msgs
  geometry_msgs
  roscpp
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(can_velocity_converter src/can_velocity_converter.cpp)
target_link_libraries(can_velocity_converter ${catkin_LIBRARIES})

install(TARGETS
  can_velocity_converter
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    config
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)