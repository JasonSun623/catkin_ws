cmake_minimum_required(VERSION 2.8.3)
project(robotnik_pp_planner)
## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)
#add for debug
#第一句保证了CXXFLAGS被加上了-g
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
#第二局保证了具体的Make语句被显示出来，我们可以实际检查编译选项是否正常。
set(CMAKE_VERBOSE_MAKEFILE ON)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
  diagnostic_updater
  robotnik_pp_msgs
  ackermann_msgs
  interactive_markers
  visualization_msgs
  std_srvs
  actionlib
  message_generation
  dynamic_reconfigure
)

#catkin_python_setup()
generate_dynamic_reconfigure_options(cfg/robotnik_pp_planner.cfg)

generate_messages(DEPENDENCIES std_msgs)
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES purepursuit_planner
  CATKIN_DEPENDS roscpp actionlib nav_msgs roscpp std_msgs tf
  geometry_msgs robotnik_pp_msgs ackermann_msgs
  dynamic_reconfigure
  message_generation
#  DEPENDS system_lib
)

###########
## Build ##
###########


include_directories(
  include
  include/${PROJECT_NAME}
  cfg/cpp
  ${catkin_INCLUDE_DIRS}
  #${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)
#include_directories(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
## Declare a cpp executable
add_executable(robotnik_pp_planner_node
  src/purepursuit_planner.cpp
  src/Component.cc
  src/pid.cpp
  src/Geometry.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(robotnik_pp_planner_node
  #robotnik_pp_msgs_generate_messages_cpp
  ${PROJECT_NAME}_gencfg ## make sure configure headers are built before any node using them
  )

## Specify libraries to link a library or executable target against
target_link_libraries(robotnik_pp_planner_node
   ${catkin_LIBRARIES}
)

## Declare a cpp executable
add_executable(robotnik_pp_planner_simple_client src/simple_client.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(robotnik_pp_planner_simple_client robotnik_pp_msgs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(robotnik_pp_planner_simple_client
   ${catkin_LIBRARIES}
)

#############
## Install ##
#############


install(PROGRAMS
   scripts/purepursuit_planner/path_marker.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
install( TARGETS
    ${PROJECT_NAME}_node ${PROJECT_NAME}_simple_client
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )


