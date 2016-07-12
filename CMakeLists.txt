cmake_minimum_required(VERSION 2.8.3)
project(nav_points)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  interactive_markers
  roscpp
  std_msgs
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES nav_points
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(nav_points src/nav_points.cpp)

target_link_libraries(nav_points
  ${catkin_LIBRARIES}
)
