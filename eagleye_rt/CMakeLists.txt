cmake_minimum_required(VERSION 2.8.3)
project(eagleye_rt)

set(CMAKE_CXX_FLAGS "-O2 -std=c++11 -Wall")
set(CMAKE_CXX_STANDARD 14)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
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
  tf2_sensor_msgs
  tf2_eigen
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
  tf2_sensor_msgs
  tf2_eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_executable(twist_relay src/twist_relay_node.cpp)
target_link_libraries(twist_relay ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(twist_relay ${catkin_EXPORTED_TARGETS})

add_executable(velocity_scale_factor src/velocity_scale_factor_node.cpp)
target_link_libraries(velocity_scale_factor ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(velocity_scale_factor ${catkin_EXPORTED_TARGETS})

add_executable(yaw_rate_offset_stop src/yaw_rate_offset_stop_node.cpp)
target_link_libraries(yaw_rate_offset_stop ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(yaw_rate_offset_stop ${catkin_EXPORTED_TARGETS})

add_executable(yaw_rate_offset src/yaw_rate_offset_node.cpp)
target_link_libraries(yaw_rate_offset ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(yaw_rate_offset ${catkin_EXPORTED_TARGETS})

add_executable(heading src/heading_node.cpp)
target_link_libraries(heading ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(heading ${catkin_EXPORTED_TARGETS})

add_executable(position src/position_node.cpp)
target_link_libraries(position ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(position ${catkin_EXPORTED_TARGETS})

add_executable(slip_angle src/slip_angle_node.cpp)
target_link_libraries(slip_angle ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(slip_angle ${catkin_EXPORTED_TARGETS})

add_executable(smoothing src/smoothing_node.cpp)
target_link_libraries(smoothing ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(smoothing ${catkin_EXPORTED_TARGETS})

add_executable(trajectory src/trajectory_node.cpp)
target_link_libraries(trajectory ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(trajectory ${catkin_EXPORTED_TARGETS})

add_executable(heading_interpolate src/heading_interpolate_node.cpp)
target_link_libraries(heading_interpolate ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(heading_interpolate ${catkin_EXPORTED_TARGETS})

add_executable(position_interpolate src/position_interpolate_node.cpp)
target_link_libraries(position_interpolate ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(position_interpolate ${catkin_EXPORTED_TARGETS})

add_executable(distance src/distance_node.cpp)
target_link_libraries(distance ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(distance ${catkin_EXPORTED_TARGETS})

add_executable(monitor src/monitor_node.cpp)
target_link_libraries(monitor ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(monitor ${catkin_EXPORTED_TARGETS})

add_executable(height src/height_node.cpp)
target_link_libraries(height ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(height ${catkin_EXPORTED_TARGETS})

add_executable(angular_velocity_offset_stop src/angular_velocity_offset_stop_node.cpp)
target_link_libraries(angular_velocity_offset_stop ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(angular_velocity_offset_stop ${catkin_EXPORTED_TARGETS})

add_executable(correction_imu src/correction_imu.cpp)
target_link_libraries(correction_imu ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(correction_imu ${catkin_EXPORTED_TARGETS})

add_executable(rtk_dead_reckoning src/rtk_dead_reckoning_node.cpp)
target_link_libraries(rtk_dead_reckoning ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(rtk_dead_reckoning ${catkin_EXPORTED_TARGETS})

add_executable(rtk_heading src/rtk_heading_node.cpp)
target_link_libraries(rtk_heading ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(rtk_heading ${catkin_EXPORTED_TARGETS})

add_executable(slip_coefficient src/slip_coefficient_node.cpp)
target_link_libraries(slip_coefficient ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(slip_coefficient ${catkin_EXPORTED_TARGETS})

add_executable(enable_additional_rolling src/enable_additional_rolling_node.cpp)
target_link_libraries(enable_additional_rolling ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(enable_additional_rolling ${catkin_EXPORTED_TARGETS})

add_executable(rolling src/rolling_node.cpp)
target_link_libraries(rolling ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(rolling ${catkin_EXPORTED_TARGETS})

add_executable(navpvt2rtk src/navpvt2rtk_node.cpp)
target_link_libraries(navpvt2rtk ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(navpvt2rtk ${catkin_EXPORTED_TARGETS})

add_executable(tf_converted_imu src/tf_converted_imu.cpp)
target_link_libraries(tf_converted_imu ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(tf_converted_imu ${catkin_EXPORTED_TARGETS})

add_executable(velocity_estimator src/velocity_estimator_node.cpp)
target_link_libraries(velocity_estimator ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(velocity_estimator ${catkin_EXPORTED_TARGETS})

install(TARGETS
  velocity_scale_factor
  yaw_rate_offset_stop
  yaw_rate_offset
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
  rtk_dead_reckoning
  rtk_heading
  enable_additional_rolling
  rolling
  navpvt2rtk
  tf_converted_imu
  velocity_estimator
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    config
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)