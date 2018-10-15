cmake_minimum_required(VERSION 2.8.3)
project(map_server)
#add for debug
#第一句保证了CXXFLAGS被加上了-g
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
#第二局保证了具体的Make语句被显示出来，我们可以实际检查编译选项是否正常。
#set(CMAKE_VERBOSE_MAKEFILE ON)

find_package(catkin REQUIRED
        COMPONENTS
            roscpp
            tf
            nav_msgs
        )

find_package(Boost REQUIRED COMPONENTS system)

find_package(PkgConfig)
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NEW_YAMLCPP_FOUND)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        image_loader
    CATKIN_DEPENDS
        roscpp
        tf
        nav_msgs
)

include_directories( include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} )
add_library(image_loader src/image_loader.cpp)
add_library(simplegridmap_loader
  src/AbstractMap/AbstractMap.cpp
  src/AbstractMap/GridMapConstants.cpp
  src/AbstractMap/Geometry.cpp
  src/AbstractMap/SimpleGridMap.cpp
  )
add_dependencies(image_loader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(image_loader SDL SDL_image ${Boost_LIBRARIES})

add_executable(map_server src/main.cpp)
add_dependencies(map_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_server
    image_loader
    yaml-cpp
    simplegridmap_loader
    ${catkin_LIBRARIES}
)

add_executable(map_server-map_saver src/map_saver.cpp)
add_dependencies(map_server-map_saver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
set_target_properties(map_server-map_saver PROPERTIES OUTPUT_NAME map_saver)
target_link_libraries(map_server-map_saver
    ${catkin_LIBRARIES}
    )

# copy test data to same place as tests are run
function(copy_test_data)
    cmake_parse_arguments(PROJECT "" "" "FILES" ${ARGN})
    foreach(datafile ${PROJECT_FILES})
        file(COPY ${datafile} DESTINATION ${PROJECT_BINARY_DIR}/test)
    endforeach()
endfunction()

## Tests
if(CATKIN_ENABLE_TESTING)
  copy_test_data( FILES
      test/testmap.bmp
      test/testmap.png )
  catkin_add_gtest(${PROJECT_NAME}_utest test/utest.cpp test/test_constants.cpp)
  target_link_libraries(${PROJECT_NAME}_utest image_loader SDL SDL_image)

  add_executable(rtest test/rtest.cpp test/test_constants.cpp)
  add_dependencies(rtest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries( rtest
      gtest
      ${catkin_LIBRARIES}
  )

  # This has to be done after we've already built targets, or catkin variables get borked
  find_package(rostest REQUIRED)
  add_rostest(test/rtest.xml)
endif()

## Install executables and/or libraries
install(TARGETS map_server-map_saver map_server image_loader
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

## Install excutable python script
install( 
    PROGRAMS
      scripts/crop_map
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
