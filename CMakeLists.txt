cmake_minimum_required(VERSION 3.1.2)
project(multi_lander)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  geometry_msgs
  sensor_msgs
  std_msgs
  mrs_msgs
  mrs_lib
  nodelet
  )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

catkin_package(
  # INCLUDE_DIRS include
  LIBRARIES MultiLander
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs mrs_msgs mrs_lib
  )

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  )

# MultiLander

add_library(MultiLander src/multi_lander.cpp)

add_dependencies(MultiLander
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(MultiLander
  ${catkin_LIBRARIES}
  )
