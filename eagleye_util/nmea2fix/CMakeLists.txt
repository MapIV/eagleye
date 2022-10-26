cmake_minimum_required(VERSION 2.8.3)
project(eagleye_nmea2fix)

set(CMAKE_CXX_FLAGS "-O2 -std=c++11 -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nmea_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES nmea2fix
)

include_directories(
  include
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

add_library(nmea2fix
  src/nmea2fix_core.cpp
)

install(DIRECTORY include/nmea2fix/
  DESTINATION include/nmea2fix/
  FILES_MATCHING PATTERN "*.hpp"
)

target_link_libraries(nmea2fix
  ${catkin_LIBRARIES}
)

install(TARGETS nmea2fix
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

find_package(catkin REQUIRED COMPONENTS
  eagleye_nmea2fix
)

add_executable(nmea2fix_node src/nmea2fix_node.cpp)
target_link_libraries(nmea2fix_node ${catkin_LIBRARIES})
add_dependencies(nmea2fix_node ${catkin_EXPORTED_TARGETS})

install(TARGETS
  nmea2fix_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)