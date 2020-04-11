cmake_minimum_required(VERSION 2.8.3)
project(radbot_controller)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(cmake_modules REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roscpp
  geometry_msgs
  visualization_msgs
  std_msgs
  tf
  tf_conversions
  jsk_rviz_plugins
  gridmap_planner
  radbot_serial_interface
)

find_package(Boost REQUIRED COMPONENTS system)



catkin_package(
  INCLUDE_DIRS
      include
      ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS
    gridmap_planner
    roscpp
    std_msgs
    tf
    tf_conversions
    message_runtime
    geometry_msgs
    visualization_msgs
    jsk_rviz_plugins
    radbot_serial_interface
  DEPENDS
    Boost
)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/radbot_controller.cpp
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
