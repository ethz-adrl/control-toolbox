cmake_minimum_required (VERSION 3.3)

include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/compilerSettings.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/explicitTemplateHelpers.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/clang-cxx-dev-tools.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/ct-cmake-helpers.cmake)

project(ct_rbd VERSION 3.0.2 LANGUAGES CXX)


set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wfatal-errors -std=c++14 -Wall -Wno-unknown-pragmas")
SET(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

find_package(kindr REQUIRED)

###################
# BUILD LIBRARIES #
###################


## create ct_rbd library
add_library(ct_rbd INTERFACE)
add_dependencies(ct_rbd ct_optcon)
target_include_directories(ct_rbd 
INTERFACE
    ${kindr_INCLUDE_DIRS}
    ${ct_optcon_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ct>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ct/rbd>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ct/rbd/robot/kinematics/ikfast>
    $<INSTALL_INTERFACE:include>
    $<INSTALL_INTERFACE:include/ct>
    $<INSTALL_INTERFACE:include/ct/rbd>
    $<INSTALL_INTERFACE:include/ct/rbd/robot/kinematics/ikfast>
)

target_link_libraries(ct_rbd INTERFACE ControlToolbox::OptimalControl)


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

## copy the header files
install(DIRECTORY include/ct/rbd DESTINATION include/ct)
install(DIRECTORY include/ct/iit DESTINATION include/ct)

## copy the cmake files required for find_package()
install(FILES "cmake/ct_rbdConfig.cmake" DESTINATION "share/ct_rbd/cmake")

## install library and targets
install(
    TARGETS ct_rbd
    EXPORT ct_rbd_export
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )

## create the ct_rbd.cmake file which holds target includes and dependencies
install (EXPORT ct_rbd_export DESTINATION share/ct_rbd/cmake)


#################
# DOCUMENTATION #
#################
if(${BUILD_DOCS})
    add_subdirectory(doc)
endif()
