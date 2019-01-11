################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(imu_publisher)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

add_definitions(-std=c++11)
################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
## Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
   INCLUDE_DIRS include ${Boost_INCLUDE_DIRS}
#   LIBRARIES bms_soc_cmd_power
   CATKIN_DEPENDS roscpp std_msgs
)

################################################################################
# Build
################################################################################
include_directories(
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(${PROJECT_NAME}	src/imu_publisher.cpp src/JY901.cpp)
# add_dependencies(hlds_laser_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})



