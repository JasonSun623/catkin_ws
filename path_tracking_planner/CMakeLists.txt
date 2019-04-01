cmake_minimum_required(VERSION 2.8.3)
project(path_tracking_planner)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
#add for debug
#第一句保证了CXXFLAGS被加上了-g
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
#第二局保证了具体的Make语句被显示出来，我们可以实际检查编译选项是否正常。
#set(CMAKE_VERBOSE_MAKEFILE ON)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
  diagnostic_updater

  ackermann_msgs
  interactive_markers
  visualization_msgs
  std_srvs
  actionlib
  message_generation
  dynamic_reconfigure
  wash_floor_msgs
#  pcl_filters
)

find_package(Boost REQUIRED COMPONENTS system)

#find_package(Eigen REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  include
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
#  ${EIGEN_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})
add_definitions(${EIGEN_DEFINITIONS})

catkin_python_setup()

generate_dynamic_reconfigure_options(cfg/PathTrackingPlanner.cfg)

catkin_package(
  INCLUDE_DIRS include/${PROJECT_NAME} ${Boost_INCLUDE_DIRS}
#  LIBRARIES purepursuit_planner
  CATKIN_DEPENDS roscpp actionlib nav_msgs roscpp std_msgs tf
  geometry_msgs ackermann_msgs
  dynamic_reconfigure
  message_generation
  DEPENDS PCL
)



## Declare a cpp executable
add_executable(path_tracking_planner_node
  src/main.cpp
  src/path_tracking_planner.cpp
  src/path_tracking_init_marker.cpp
  src/path_algorithm.cpp
  src/Component.cc
  src/pid.cpp
  src/Geometry.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(path_tracking_planner_node rt
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   ${PCL_LIBRARIES}
#   ${Eigen_LIBRARIES}
)


add_executable(path_tracking_planner_simple_client src/simple_client.cpp)

target_link_libraries(path_tracking_planner_simple_client
   ${catkin_LIBRARIES}
)

add_executable(planner_status_client src/status_client.cpp)

target_link_libraries(planner_status_client
   ${catkin_LIBRARIES}
)



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


