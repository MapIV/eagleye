cmake_minimum_required(VERSION 2.8.3)
project(eagleye_rt)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  rtklib_msgs
  eagleye_msgs
  navigation
  coordinate
)

catkin_package(
   CATKIN_DEPENDS
   roscpp
   std_msgs
   geometry_msgs
   sensor_msgs
   rtklib_msgs
   eagleye_msgs
   navigation
   coordinate
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(velocity_scale_factor src/velocity_scale_factor_node.cpp)
target_link_libraries(velocity_scale_factor ${catkin_LIBRARIES})
add_dependencies(velocity_scale_factor ${catkin_EXPORTED_TARGETS})

add_executable(yawrate_offset_stop src/yawrate_offset_stop_node.cpp)
target_link_libraries(yawrate_offset_stop ${catkin_LIBRARIES})
add_dependencies(yawrate_offset_stop ${catkin_EXPORTED_TARGETS})

add_executable(yawrate_offset src/yawrate_offset_node.cpp)
target_link_libraries(yawrate_offset ${catkin_LIBRARIES})
add_dependencies(yawrate_offset ${catkin_EXPORTED_TARGETS})

add_executable(heading src/heading_node.cpp)
target_link_libraries(heading ${catkin_LIBRARIES})
add_dependencies(heading ${catkin_EXPORTED_TARGETS})

add_executable(position src/position_node.cpp)
target_link_libraries(position ${catkin_LIBRARIES})
add_dependencies(position ${catkin_EXPORTED_TARGETS})

add_executable(slip_angle src/slip_angle_node.cpp)
target_link_libraries(slip_angle ${catkin_LIBRARIES})
add_dependencies(slip_angle ${catkin_EXPORTED_TARGETS})

add_executable(smoothing src/smoothing_node.cpp)
target_link_libraries(smoothing ${catkin_LIBRARIES})
add_dependencies(smoothing ${catkin_EXPORTED_TARGETS})

add_executable(trajectory src/trajectory_node.cpp)
target_link_libraries(trajectory ${catkin_LIBRARIES})
add_dependencies(trajectory ${catkin_EXPORTED_TARGETS})

add_executable(heading_interpolate src/heading_interpolate_node.cpp)
target_link_libraries(heading_interpolate ${catkin_LIBRARIES})
add_dependencies(heading_interpolate ${catkin_EXPORTED_TARGETS})

add_executable(position_interpolate src/position_interpolate_node.cpp)
target_link_libraries(position_interpolate ${catkin_LIBRARIES})
add_dependencies(position_interpolate ${catkin_EXPORTED_TARGETS})

add_executable(distance src/distance_node.cpp)
target_link_libraries(distance ${catkin_LIBRARIES})
add_dependencies(distance ${catkin_EXPORTED_TARGETS})

add_executable(debug src/debug_node.cpp)
target_link_libraries(debug ${catkin_LIBRARIES})
add_dependencies(debug ${catkin_EXPORTED_TARGETS})

add_executable(slip_coefficient src/slip_coefficient_node.cpp)
target_link_libraries(slip_coefficient ${catkin_LIBRARIES})
add_dependencies(slip_coefficient ${catkin_EXPORTED_TARGETS})
