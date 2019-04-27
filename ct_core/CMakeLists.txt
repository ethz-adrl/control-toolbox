cmake_minimum_required (VERSION 3.3)

include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/compilerSettings.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/explicitTemplateHelpers.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/clang-cxx-dev-tools.cmake)

project(ct_core VERSION 3.0.0 LANGUAGES CXX)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -pthread -std=c++14 -Wall -Wfatal-errors")
SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -pthread -std=c++14 -Wall -Wfatal-errors")
SET(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

#find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS REQUIRED)

find_package(PythonLibs QUIET)
find_package(Qwt QUIET)
find_package(Qt4 QUIET)

if(QWT_FOUND AND Qt4_FOUND)
    set(QWT_ENABLED true)
    add_definitions( -DQWT_ENABLED )
    include(${QT_USE_FILE})
    include_directories(${QWT_INCLUDE_DIR}})
    message(STATUS "COMPILING WITH QWT")
    set(ct_core_extra_libs ${QWT_LIBRARY} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
else()
    message(STATUS "COMPILING WITHOUT QWT")
    set(ct_core_extra_libs "")
endif()


if (PYTHONLIBS_FOUND)
   message(STATUS "Python found")
   include_directories(${PYTHON_INCLUDE_DIRS})
   add_definitions(-DPLOTTING_ENABLED)
else()
   message(STATUS "Python not found")
   set(PYTHON_LIBRARY "")
endif()

include_directories(
    ${BOOST_INCLUDE_DIRS}
#    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    include
    include/external
)

## dummy prespec libs
set(PRESPEC_LIB_NAMES "")

## assemble list of libraries that contain prespecified templates
if(USE_PRESPEC)
    ct_configure_explicit_templates("${CMAKE_CURRENT_SOURCE_DIR}/../ct/config/explicit_templates.cfg" "${CMAKE_CURRENT_SOURCE_DIR}/prespec/" "ct_core")
    message(WARNING "CT Core: Compiling the following explict template libraries: ${PRESPEC_LIB_NAMES}")
endif(USE_PRESPEC)

#used later for clang-tidy
set(INC_DIRS ${BOOST_INCLUDE_DIRS} "${CMAKE_CURRENT_SOURCE_DIR}/include" "${CMAKE_CURRENT_SOURCE_DIR}/include/external" ${EIGEN3_INCLUDE_DIR})

#catkin_package(
#   INCLUDE_DIRS
#        ${BOOST_INCLUDE_DIRS}
#        include
#        include/external
#        examples
#        ${EIGEN3_INCLUDE_DIR}
#   LIBRARIES
#        ct_core
#        dl #required for gcc-compatibility
#        ${PRESPEC_LIB_NAMES}
#   DEPENDS
#        EIGEN3
#)

set(CODEGEN_TEMPLATE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/templates")
set(CODEGEN_OUTPUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/generated")
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/include/ct/core/templateDir.h.in ${CMAKE_CURRENT_SOURCE_DIR}/include/ct/core/templateDir.h)

add_library(ct_core
   src/core/plot/plot.cpp
   src/core/internal/autodiff/SparsityPattern.cpp
)

# add libraries for explicit templates
target_link_libraries(ct_core ${catkin_LIBRARIES} ${PYTHON_LIBRARY})
if(USE_PRESPEC)
    ct_add_explicit_template_libs()
endif(USE_PRESPEC)

if(BUILD_EXAMPLES)
    add_executable(ex_DampedOscillatorCustomController examples/DampedOscillatorCustomController.cpp)
    add_executable(ex_DampedOscillator examples/DampedOscillator.cpp)
    add_executable(ex_MasspointIntegration examples/MasspointIntegration.cpp)
endif()


###########
# TESTING #
###########
option( "Build the tests" ON)
if(PACKAGE_TESTS)
    find_package(GTest QUIET)
    enable_testing()
    #add_subdirectory(test)
endif()

#macro for conveniently adding tests
macro(package_add_test TESTNAME)
    add_executable(${TESTNAME} ${ARGN})
    target_link_libraries(${TESTNAME} gtest gtest_main ct_core)
    add_test(NAME ${TESTNAME} COMMAND ${TESTNAME})
    set_target_properties(${TESTNAME} PROPERTIES FOLDER test)
endmacro()

package_add_test(NoiseTest test/NoiseTest.cpp)
#package_add_test(SecondOrderSystemTest test/SecondOrderSystemTest.cpp)
#package_add_test(IntegrationTest test/integration/IntegrationTest.cpp)
#package_add_test(IntegratorComparison test/integration/IntegratorComparison.cpp)
#package_add_test(SymplecticIntegrationTest test/integration/SymplecticIntegrationTest.cpp)
#package_add_test(SystemDiscretizerTest test/integration/SystemDiscretizerTest.cpp)
#package_add_test(SensitivityTest test/integration/sensitivity/SensitivityTest.cpp)
#package_add_test(InterpolationTest test/InterpolationTest.cpp)
#package_add_test(DiscreteArrayTest test/DiscreteArrayTest.cpp)
#package_add_test(DiscreteTrajectoryTest test/DiscreteTrajectoryTest.cpp)

# todo add the remaining tests
#catkin_add_gtest(LinspaceTest test/LinspaceTest.cpp)
#target_link_libraries(LinspaceTest ${catkin_LIBRARIES})
#
#catkin_add_gtest(AutoDiffLinearizerTest test/AutoDiffLinearizerTest.cpp)
#target_link_libraries(AutoDiffLinearizerTest ${catkin_LIBRARIES} ct_core pthread)
#
#catkin_add_gtest(SwitchingTest test/switching/SwitchingTest.cpp)
#target_link_libraries(SwitchingTest ${catkin_LIBRARIES} ct_core)
#
#catkin_add_gtest(SwitchedControlledSystemTest test/switching/SwitchedControlledSystemTest.cpp)
#target_link_libraries(SwitchedControlledSystemTest ${catkin_LIBRARIES} ct_core)
#
#catkin_add_gtest(SwitchedDiscreteControlledSystemTest test/switching/SwitchedDiscreteControlledSystemTest.cpp)
#target_link_libraries(SwitchedDiscreteControlledSystemTest ${catkin_LIBRARIES} ct_core)
#
#catkin_add_gtest(MatrixInversionTest test/math/MatrixInversionTest.cpp)
#target_link_libraries(MatrixInversionTest ${catkin_LIBRARIES})

# Codegen tests cannot run in parallel. Thus we build and run all codegen tests in one
#catkin_add_gtest(CodegenTests test/CodegenTests.cpp)
#target_link_libraries(CodegenTests ${catkin_LIBRARIES} ct_core dl pthread)

## example for using the build-in plotting tool
add_executable(plotTest test/plot/plotTest.cpp)
target_link_libraries(plotTest ${catkin_LIBRARIES} ct_core)

if(QWT_ENABLED)
    add_executable(plotQwtTest test/plot/plotQwtTest.cpp)
    target_link_libraries(plotQwtTest ${catkin_LIBRARIES} ct_core ${ct_core_extra_libs} pthread)
endif()


## unit tests for prespec templates
if(USE_PRESPEC)
    catkin_add_gtest(IntegrationTestPrespec test/integration/IntegrationTestPrespec.cpp)
    target_link_libraries(IntegrationTestPrespec ${catkin_LIBRARIES} ct_core ${PRESPEC_LIB_NAMES})

    catkin_add_gtest(IntegratorComparisonPrespec test/integration/IntegratorComparisonPrespec.cpp)
    target_link_libraries(IntegratorComparisonPrespec ${catkin_LIBRARIES} ct_core ${PRESPEC_LIB_NAMES})
endif(USE_PRESPEC)


install(DIRECTORY include/ct/core DESTINATION include/ct)


# 1- local build #

# Register the local build in case one doesn't use "make install"
export(PACKAGE ct_core)

# Create variable for the local build tree
get_property(ct_core_include_dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)

# Configure config file for local build tree
configure_file(ct_coreConfig.cmake.in
  "${PROJECT_BINARY_DIR}/ct_coreConfig.cmake" @ONLY)

# 2- installation build #

# Change the include location for the case of an install location
set(ct_core_include_dirs ${CMAKE_INSTALL_PREFIX}/include ${EIGEN_INCLUDE_DIR})


# We put the generated file for installation in a different repository (i.e., ./CMakeFiles/)
configure_file(ct_coreConfig.cmake.in
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/ct_coreConfig.cmake" @ONLY)

install(FILES
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/ct_coreConfig.cmake"
  DESTINATION share/ct_core/cmake COMPONENT dev)



## build doxygen documentation
find_package(Doxygen)
if(DOXYGEN_FOUND)

    set(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/doc/ct_core.doxyfile)
    set(doxyfile ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile)

    configure_file(${doxyfile_in} ${doxyfile} @ONLY)

    add_custom_target(doc
        COMMAND ${CMAKE_COMMAND} -E echo_append "Building API Documentation..."
        COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/doc
        COMMAND ${CMAKE_COMMAND} -E echo_append "API Documentation built in ${CMAKE_CURRENT_SOURCE_DIR}/doc"
        VERBATIM)
endif()

ct_configure_clang_tidy(INC_DIRS)
