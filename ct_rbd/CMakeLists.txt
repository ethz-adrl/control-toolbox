cmake_minimum_required (VERSION 3.3)

include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/compilerSettings.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/explicitTemplateHelpers.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/clang-cxx-dev-tools.cmake)

project(ct_rbd VERSION 3.0.0 LANGUAGES CXX)

include(${CMAKE_CURRENT_SOURCE_DIR}/../ct_optcon/cmake/externalSolvers.cmake)


set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wfatal-errors -std=c++14 -Wall -Wno-unknown-pragmas")
SET(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

find_package(kindr)
find_package(ct_optcon)

include_directories(
  ${kindr_INCLUDE_DIRS}
  ${ct_optcon_INCLUDE_DIRS}
  include
  include/ct/rbd
  include/ct/rbd/robot/kinematics/ikfast
  test/models/testhyq/generated
  test
)


#todo export this properly from ct_core
find_package(PythonLibs QUIET)
if (PYTHONLIBS_FOUND)
   message(STATUS "Python found")
   include_directories(${PYTHON_INCLUDE_DIRS})
   set(PLOTTING_ENABLED true)
   add_definitions(-DPLOTTING_ENABLED)
else()
   message(STATUS "Python not found")
   set(PYTHON_LIBRARY "")
endif()


## assemble list of libraries that contain prespecified templates
if(USE_PRESPEC)
    ct_configure_explicit_templates("${CMAKE_CURRENT_SOURCE_DIR}/../ct/config/explicit_templates.cfg" "${CMAKE_CURRENT_SOURCE_DIR}/prespec/" "ct_rbd")
    message(WARNING "CT RBD: Compiling the following explict template libraries: ${PRESPEC_LIB_NAMES}")
endif(USE_PRESPEC)

## used later for clang-tidy
set(INC_DIRS ${catkin_INCLUDE_DIRS} ${kindr_INCLUDE_DIRS} "${CMAKE_CURRENT_SOURCE_DIR}/include" "${CMAKE_CURRENT_SOURCE_DIR}/include/ct/rbd")
ct_configure_clang_tidy(INC_DIRS)



## add libraries for explicit template
if(USE_PRESPEC)
    ct_add_explicit_template_libs()
endif(USE_PRESPEC)



###########
# TESTING #
###########

if(BUILD_TESTS)
    #find_package(GTest QUIET)
    enable_testing()
    add_subdirectory(test)
endif()



#################
# INSTALLATION  #
#################

# for correct libraries locations across platforms
include(GNUInstallDirs) 

install(DIRECTORY include/ct/rbd DESTINATION include/ct)

### 1- local build ###

# Register the local build in case one doesn't use "make install"
export(PACKAGE ct_rbd)

# Create variable for the local build tree
get_property(ct_rbd_include_dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)

# Configure config file for local build tree
configure_file(cmake/ct_rbdConfig.cmake.in "${PROJECT_BINARY_DIR}/ct_rbdConfig.cmake" @ONLY)

### 2- installation build ###

# Change the include location for the case of an install location
set(ct_rbd_include_dirs 
    ${CMAKE_INSTALL_PREFIX}/include 
    ${ct_optcon_INCLUDE_DIRS}
    ${kindr_INCLUDE_DIRS}
    )

# We put the generated file for installation in a different repository (i.e., ./CMakeFiles/)
configure_file(cmake/ct_rbdConfig.cmake.in "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/ct_rbdConfig.cmake" @ONLY)

install(FILES "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/ct_rbdConfig.cmake"
    DESTINATION share/ct/rbd/cmake COMPONENT dev)



find_package(Doxygen)
if(DOXYGEN_FOUND)
  set(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/doc/ct_rbd.doxyfile)
  set(doxyfile ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile)

  configure_file(${doxyfile_in} ${doxyfile} @ONLY)

  add_custom_target(doc
    COMMAND ${CMAKE_COMMAND} -E echo_append "Building API Documentation..."
    COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/doc
    COMMAND ${CMAKE_COMMAND} -E echo_append "API Documentation built in ${CMAKE_CURRENT_SOURCE_DIR}/doc"
    VERBATIM)
endif()
