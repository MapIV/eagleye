cmake_minimum_required(VERSION 2.8.3)
project(imu_gnss_localizer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

 add_message_files(
   FILES
   VelocitySF.msg
   YawrateOffset.msg
   Heading_raw.msg
   Heading.msg
   Distance.msg
   UsrVel_enu.msg
   PositionDis_raw.msg
   PositionDis.msg
   RTKLIB.msg
   Debug_tool.msg
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
   CATKIN_DEPENDS message_runtime roscpp rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(RCalc_VelocitySF src/RCalc_VelocitySF.cpp)
add_dependencies(RCalc_VelocitySF ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(RCalc_VelocitySF ${catkin_LIBRARIES})

add_executable(RCalc_YawrateOffsetStop src/RCalc_YawrateOffsetStop.cpp)
add_dependencies(RCalc_YawrateOffsetStop ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(RCalc_YawrateOffsetStop ${catkin_LIBRARIES})

add_executable(RCalc_YawrateOffset src/RCalc_YawrateOffset.cpp)
add_dependencies(RCalc_YawrateOffset ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(RCalc_YawrateOffset ${catkin_LIBRARIES})

add_executable(RCalc_Heading src/RCalc_Heading.cpp)
add_dependencies(RCalc_Heading ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(RCalc_Heading ${catkin_LIBRARIES})

add_executable(RCalc_Heading_Int src/RCalc_Heading_Int.cpp)
add_dependencies(RCalc_Heading_Int ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(RCalc_Heading_Int ${catkin_LIBRARIES})

add_executable(RCalc_Distance src/RCalc_Distance.cpp)
add_dependencies(RCalc_Distance ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(RCalc_Distance ${catkin_LIBRARIES})

add_executable(RCalc_Trajectory src/RCalc_Trajectory.cpp)
add_dependencies(RCalc_Trajectory ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(RCalc_Trajectory ${catkin_LIBRARIES})

add_executable(RCalc_PositionDis src/RCalc_PositionDis.cpp)
add_dependencies(RCalc_PositionDis ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(RCalc_PositionDis ${catkin_LIBRARIES})

add_executable(RCalc_PositionDis_Int src/RCalc_PositionDis_Int.cpp)
add_dependencies(RCalc_PositionDis_Int ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(RCalc_PositionDis_Int ${catkin_LIBRARIES})

add_executable(rtklib_connecter src/rtklib_connecter.cpp)
add_dependencies(rtklib_connecter ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(rtklib_connecter ${catkin_LIBRARIES})

add_executable(Debug_tool src/Debug_tool.cpp)
add_dependencies(Debug_tool ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(Debug_tool ${catkin_LIBRARIES})
