cmake_minimum_required(VERSION 2.8.3)
project(status_checker)

set(CMAKE_CXX_FLAGS "-O2 -std=c++11 -Wall")

find_package(catkin REQUIRED COMPONENTS
)

catkin_package(
  CATKIN_DEPENDS
)

add_executable(status_checker src/status_checker.cpp)
target_link_libraries(status_checker ${catkin_LIBRARIES} )
add_dependencies(status_checker ${catkin_EXPORTED_TARGETS})
