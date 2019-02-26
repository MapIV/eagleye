cmake_minimum_required(VERSION 2.8.3)
project(imu_gnss_localizer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  #ublox_msgs
)

 add_message_files(
   FILES
   VelocitySF.msg
   YawrateOffset.msg
   Heading.msg
   Distance.msg
   UsrVel_enu.msg
   Position.msg
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
   CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(RCalc_YawrateOffsetStop src/RCalc_YawrateOffsetStop.cpp)
target_link_libraries(RCalc_YawrateOffsetStop ${catkin_LIBRARIES})

add_executable(RCalc_VelocitySF src/RCalc_VelocitySF.cpp)
target_link_libraries(RCalc_VelocitySF ${catkin_LIBRARIES})

add_executable(RCalc_Heading1st src/RCalc_Heading1st.cpp)
target_link_libraries(RCalc_Heading1st ${catkin_LIBRARIES})

add_executable(RCalc_Heading2nd src/RCalc_Heading2nd.cpp)
target_link_libraries(RCalc_Heading2nd ${catkin_LIBRARIES})

add_executable(RCalc_Heading3rd src/RCalc_Heading3rd.cpp)
target_link_libraries(RCalc_Heading3rd ${catkin_LIBRARIES})

add_executable(RCalc_YawrateOffset1st src/RCalc_YawrateOffset1st.cpp)
target_link_libraries(RCalc_YawrateOffset1st ${catkin_LIBRARIES})

add_executable(RCalc_YawrateOffset2nd src/RCalc_YawrateOffset2nd.cpp)
target_link_libraries(RCalc_YawrateOffset2nd ${catkin_LIBRARIES})

add_executable(RCalc_Trajectory src/RCalc_Trajectory.cpp)
target_link_libraries(RCalc_Trajectory ${catkin_LIBRARIES})

add_executable(RCalc_Trajectory_compare src/RCalc_Trajectory_compare.cpp)
target_link_libraries(RCalc_Trajectory_compare ${catkin_LIBRARIES})

add_executable(RCalc_Distance src/RCalc_Distance.cpp)
target_link_libraries(RCalc_Distance ${catkin_LIBRARIES})

add_executable(RCalc_PositionDis src/RCalc_PositionDis.cpp)
target_link_libraries(RCalc_PositionDis ${catkin_LIBRARIES})
