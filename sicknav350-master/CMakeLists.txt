cmake_minimum_required(VERSION 2.8.3)
project(sicknav350)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs nav_msgs geometry_msgs sicktoolbox rosconsole diagnostic_updater tf message_generation)

## System dependencies are found with CMake's conventions
#find_package(Threads)

generate_messages(
   DEPENDENCIES
   std_msgs
)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS
 LIBRARIES
 CATKIN_DEPENDS roscpp std_msgs sensor_msgs nav_msgs geometry_msgs sicktoolbox rosconsole diagnostic_updater tf message_runtime move_base_msgs actionlib
 DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  #${Boost_INCLUDE_DIRS}
)

#used for pub scan 
add_executable(sicknav350_scan_node src/sicknav350_scan.cpp)
#used for loc and pub scan 
add_executable(sicknav350_loc_node src/sicknav350_loc.cpp src/Geometry.cpp)
#used for mapping and pub scan 
add_executable(sicknav350_mapping_node src/sicknav350_mapping.cpp)

target_link_libraries(sicknav350_scan_node ${catkin_LIBRARIES})
target_link_libraries(sicknav350_loc_node ${catkin_LIBRARIES})
target_link_libraries(sicknav350_mapping_node ${catkin_LIBRARIES})

add_dependencies(sicknav350_scan_node sicknav350_gencpp)
add_dependencies(sicknav350_loc_node sicknav350_gencpp)
add_dependencies(sicknav350_mapping_node sicknav350_gencpp)
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS sicknav350_scan_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS sicknav350_loc_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS sicknav350_mapping_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})



