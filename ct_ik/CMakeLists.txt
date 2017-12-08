cmake_minimum_required (VERSION 2.6)

include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/compilerSettings.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/explicitTemplateHelpers.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/clang-cxx-dev-tools.cmake)

project(ct_ik)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -pthread -fopenmp -std=c++11 -Wall")
SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -pthread -std=c++11 -Wall")
SET(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

find_package(catkin REQUIRED ct_rbd)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## assemble list of libraries that contain prespecified templates
ct_configure_explicit_templates("${CMAKE_CURRENT_SOURCE_DIR}/../ct/config/explicit_templates.cfg" "${CMAKE_CURRENT_SOURCE_DIR}/prespec/" "ct_core")
message(WARNING "CT Core: Compiling the following explict template libraries: ${PRESPEC_LIB_NAMES}")

set(INC_DIRS ${catkin_INCLUDE_DIRS}  "${CMAKE_CURRENT_SOURCE_DIR}/include")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ct_ik
  CATKIN_DEPENDS ct_rbd
#  DEPENDS system_lib
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/HyA/transform6d.cpp
)

set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-fPIC -DIKFAST_NO_MAIN -DIKFAST_CLIBRARY -shared -Wno-unused-variable")

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/ct_ik_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ct_ik.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
