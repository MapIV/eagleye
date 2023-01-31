cmake_minimum_required(VERSION 2.8.3)
project(eagleye_gnss_converter)

set(CMAKE_CXX_FLAGS "-O2 -std=c++11 -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nmea_msgs
  rtklib_msgs
  geometry_msgs
  ublox_msgs
  eagleye_coordinate
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gnss_converter
)

include_directories(
  include
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

add_library(gnss_converter
  src/nmea2fix_core.cpp
)

install(DIRECTORY include/gnss_converter/
  DESTINATION include/gnss_converter/
  FILES_MATCHING PATTERN "*.hpp"
)

target_link_libraries(gnss_converter
  ${catkin_LIBRARIES}
)

install(TARGETS gnss_converter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

find_package(catkin REQUIRED COMPONENTS
  eagleye_gnss_converter
)

add_executable(gnss_converter_node src/gnss_converter_node.cpp)
target_link_libraries(gnss_converter_node ${catkin_LIBRARIES})
add_dependencies(gnss_converter_node ${catkin_EXPORTED_TARGETS})

install(TARGETS
  gnss_converter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)