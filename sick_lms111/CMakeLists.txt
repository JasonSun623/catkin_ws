cmake_minimum_required(VERSION 2.8.3)
project(sick_lms111)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs nav_msgs geometry_msgs   tf message_generation)

## System dependencies are found with CMake's conventions
find_package(Threads REQUIRED)
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
  INCLUDE_DIRS include
  LIBRARIES SickLD SickLMS1xx 
  CATKIN_DEPENDS
  DEPENDS Threads
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

# Driver libraries
add_library(SickLD c++/drivers/ld/sickld/SickLD.cc c++/drivers/ld/sickld/SickLDBufferMonitor.cc c++/drivers/ld/sickld/SickLDMessage.cc)
target_link_libraries(SickLD ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

add_library(SickLMS1xx c++/drivers/lms1xx/sicklms1xx/SickLMS1xx.cc c++/drivers/lms1xx/sicklms1xx/SickLMS1xxBufferMonitor.cc c++/drivers/lms1xx/sicklms1xx/SickLMS1xxMessage.cc)
target_link_libraries(SickLMS1xx ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})


# Examples
add_library(LDConfigLib c++/examples/ld/ld_config/src/ConfigFile.cpp)

add_executable(sick_lms111 c++/examples/lms1xx/lms1xx_simple_app/src/main.cc)
target_link_libraries(sick_lms111 SickLMS1xx ${catkin_LIBRARIES})


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
install(TARGETS sick_lms111
  
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## TODO Move headers that aren't needed externally back into the source
## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hh"
  PATTERN ".svn" EXCLUDE
)

## TODO There are readmes, etc, create install rules for these
## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
