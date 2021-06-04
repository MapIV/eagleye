cmake_minimum_required(VERSION 2.8.3)
project(fix2pose)

find_package(catkin REQUIRED COMPONENTS
  eagleye_msgs
  roscpp
  rospy
  std_msgs
  tf
  coordinate
)

catkin_package(
  CATKIN_DEPENDS
  eagleye_msgs
  roscpp
  rospy
  std_msgs
  coordinate
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(fix2pose src/fix2pose.cpp)
target_link_libraries(fix2pose ${catkin_LIBRARIES} )
add_dependencies(fix2pose ${catkin_EXPORTED_TARGETS})

install(TARGETS
  fix2pose
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
