cmake_minimum_required(VERSION 3.0.2)
project(tracer_base)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
## Set compiler to use c++ 17 features
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# cmake module path
#list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS    
  roslaunch 
  roscpp 
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2 
  tf2_ros
  ugv_sdk
  tracer_msgs
)
    
# find_package(Boost REQUIRED COMPONENTS chrono)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tracer_messenger
  CATKIN_DEPENDS ugv_sdk tracer_msgs

# CATKIN_DEPENDS tracer_msgs roscpp sensor_msgs
#   DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    /usr/include
    /home/cotek/linux-demo-ver 2.0.8-1.0.23/ver 2.0.8-1.0.23/acusb linux demo/64bit/dll
)

link_directories(
  /usr/lib
  /home/cotek/linux-demo-ver 2.0.8-1.0.23/ver 2.0.8-1.0.23/acusb linux demo/64bit/dll
)

add_library(tracer_messenger STATIC src/tracer_messenger.cpp)
target_link_libraries(tracer_messenger ${catkin_LIBRARIES})
set_property(TARGET tracer_messenger PROPERTY POSITION_INDEPENDENT_CODE ON)

add_executable(tracer_base_node src/tracer_base_node.cpp)
target_link_libraries(tracer_base_node tracer_messenger 
  ${catkin_LIBRARIES}
  CanCmd
)

add_executable(cotek_can_bridge src/cotek_can_bridge.cpp)
target_link_libraries(cotek_can_bridge
  ${catkin_LIBRARIES}
  CanCmd          # 你的 libCanCmd.so 动态库
)

# add_executable(tracer_base_sim_node src/tracer_base_sim_node.cpp)
# target_link_libraries(tracer_base_sim_node tracer_messenger ${catkin_LIBRARIES})

#############
## Install ##
#############

#roslaunch_add_file_check(launch)

#install(TARGETS tracer_messenger tracer_base_node
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#install(DIRECTORY launch urdf
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
