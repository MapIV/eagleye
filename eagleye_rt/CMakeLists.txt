cmake_minimum_required(VERSION 2.8.3)
project(eagleye_rt)

set(CMAKE_CXX_FLAGS "-O2 -std=c++11 -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  rtklib_msgs
  eagleye_msgs
  eagleye_navigation
  eagleye_coordinate
  geodesy
  geographic_msgs
  diagnostic_updater
  tf2
  tf2_ros
  tf2_geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  rtklib_msgs
  eagleye_msgs
  eagleye_navigation
  eagleye_coordinate
  geodesy
  geographic_msgs
  diagnostic_updater
  tf2
  tf2_ros
  tf2_geometry_msgs
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

add_executable(monitor src/monitor_node.cpp)
target_link_libraries(monitor ${catkin_LIBRARIES})
add_dependencies(monitor ${catkin_EXPORTED_TARGETS})

add_executable(height src/height_node.cpp)
target_link_libraries(height ${catkin_LIBRARIES})
add_dependencies(height ${catkin_EXPORTED_TARGETS})

add_executable(angular_velocity_offset_stop src/angular_velocity_offset_stop_node.cpp)
target_link_libraries(angular_velocity_offset_stop ${catkin_LIBRARIES})
add_dependencies(angular_velocity_offset_stop ${catkin_EXPORTED_TARGETS})

add_executable(correction_imu src/correction_imu.cpp)
target_link_libraries(correction_imu ${catkin_LIBRARIES})
add_dependencies(correction_imu ${catkin_EXPORTED_TARGETS})

add_executable(rtk_deadreckoning src/rtk_deadreckoning_node.cpp)
target_link_libraries(rtk_deadreckoning ${catkin_LIBRARIES})
add_dependencies(rtk_deadreckoning ${catkin_EXPORTED_TARGETS})

add_executable(rtk_heading src/rtk_heading_node.cpp)
target_link_libraries(rtk_heading ${catkin_LIBRARIES})
add_dependencies(rtk_heading ${catkin_EXPORTED_TARGETS})

add_executable(slip_coefficient src/slip_coefficient_node.cpp)
target_link_libraries(slip_coefficient ${catkin_LIBRARIES})
add_dependencies(slip_coefficient ${catkin_EXPORTED_TARGETS})

add_executable(roll_angle src/roll_angle_node.cpp)
target_link_libraries(roll_angle ${catkin_LIBRARIES} )
add_dependencies(roll_angle ${catkin_EXPORTED_TARGETS})

install(TARGETS
  velocity_scale_factor
  yawrate_offset_stop
  yawrate_offset
  heading
  position
  slip_angle
  slip_coefficient
  smoothing
  trajectory
  heading_interpolate
  position_interpolate
  distance
  monitor
  height
  angular_velocity_offset_stop
  correction_imu
  rtk_deadreckoning
  rtk_heading
  roll_angle
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    config
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)