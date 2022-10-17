cmake_minimum_required(VERSION 3.0.2)
project(avp_mapping)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++14)
add_definitions(-std=c++14)
set(CMAKE_CXX_STANDARD 14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  message_generation
  pcl_ros
  roscpp
  rospy
  std_msgs
  std_srvs
  cv_bridge
  image_transport
  tf
)

set(ALL_TARGET_LIBRARIES "")

include(cmake/glog.cmake)
include(cmake/YAML.cmake)
include(cmake/PCL.cmake)
include(cmake/eigen.cmake)
include(cmake/g2o.cmake)
include(cmake/opencv.cmake)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

include(cmake/global_defination.cmake)

file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/apps/*_node.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})

# message(STATUS "ALL_SRCS = ${ALL_SRCS}")

add_executable(avp_data_pretreat_node src/apps/avp_data_pretreat_node.cpp ${ALL_SRCS})
add_dependencies(avp_data_pretreat_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(avp_data_pretreat_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})



