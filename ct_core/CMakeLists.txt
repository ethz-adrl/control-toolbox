cmake_minimum_required (VERSION 2.6)

include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/compilerSettings.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/explicitTemplateHelpers.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/clang-cxx-dev-tools.cmake)

project (ct_core)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -pthread -std=c++11 -Wall")
SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -pthread -std=c++11 -Wall")
SET(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

find_package(catkin REQUIRED)
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
  ${catkin_INCLUDE_DIRS}
)

include_directories (
    include
    include/external
    ${EIGEN3_INCLUDE_DIR}
)

## assemble list of libraries that contain prespecified templates
ct_configure_explicit_templates("${CMAKE_CURRENT_SOURCE_DIR}/../ct/config/explicit_templates.cfg" "${CMAKE_CURRENT_SOURCE_DIR}/prespec/" "ct_core")
message(WARNING "CT Core: Compiling the following explict template libraries: ${PRESPEC_LIB_NAMES}")

#used later for clang-tidy
set(INC_DIRS ${BOOST_INCLUDE_DIRS} "${CMAKE_CURRENT_SOURCE_DIR}/include" "${CMAKE_CURRENT_SOURCE_DIR}/include/external" ${EIGEN3_INCLUDE_DIR})

catkin_package(
   INCLUDE_DIRS
        ${BOOST_INCLUDE_DIRS}
        include
        include/external
        ${EIGEN3_INCLUDE_DIR}
   LIBRARIES
       dl
       ct_core
       ${PRESPEC_LIB_NAMES}
   DEPENDS
       EIGEN3
)


set(CODEGEN_TEMPLATE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/templates")
set(CODEGEN_OUTPUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/generated")
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/include/ct/core/templateDir.h.in ${CMAKE_CURRENT_SOURCE_DIR}/include/ct/core/templateDir.h)

add_library(ct_core
   src/core/plot/plot.cpp
   src/core/internal/autodiff/SparsityPattern.cpp
)

# add libraries for explicit template
target_link_libraries(ct_core ${catkin_LIBRARIES} ${PYTHON_LIBRARY})
ct_add_explicit_template_libs()

if(BUILD_EXAMPLES)
    add_executable(ex_DampedOscillatorCustomController examples/DampedOscillatorCustomController.cpp)
    add_executable(ex_DampedOscillator examples/DampedOscillator.cpp)
    add_executable(ex_MasspointIntegration examples/MasspointIntegration.cpp)
endif()

catkin_add_gtest(NoiseTest test/NoiseTest.cpp)
target_link_libraries(NoiseTest ${catkin_LIBRARIES})

catkin_add_gtest(SecondOrderSystemTest test/SecondOrderSystemTest.cpp)
target_link_libraries(SecondOrderSystemTest ${catkin_LIBRARIES})

# This test is not required to link against ${PRESPEC_LIB_NAMES} as it is not using prespec.
# However, we just test that it does not fail here.
catkin_add_gtest(IntegrationTest test/integration/IntegrationTest.cpp)
target_link_libraries(IntegrationTest ${catkin_LIBRARIES} ct_core ${PRESPEC_LIB_NAMES})
catkin_add_gtest(IntegrationTestPrespec test/integration/IntegrationTestPrespec.cpp)
target_link_libraries(IntegrationTestPrespec ${catkin_LIBRARIES} ct_core ${PRESPEC_LIB_NAMES})

catkin_add_gtest(IntegratorComparison test/integration/IntegratorComparison.cpp)
target_link_libraries(IntegratorComparison ${catkin_LIBRARIES} ct_core)
catkin_add_gtest(IntegratorComparisonPrespec test/integration/IntegratorComparisonPrespec.cpp)
target_link_libraries(IntegratorComparisonPrespec ${catkin_LIBRARIES} ct_core ${PRESPEC_LIB_NAMES})

catkin_add_gtest(SymplecticIntegrationTest test/integration/SymplecticIntegrationTest.cpp)
target_link_libraries(SymplecticIntegrationTest ${catkin_LIBRARIES} ct_core)

catkin_add_gtest(SystemDiscretizerTest test/integration/SystemDiscretizerTest.cpp)
target_link_libraries(SystemDiscretizerTest ${catkin_LIBRARIES} ct_core)

add_executable(SensitivityTest test/integration/sensitivity/SensitivityTest.cpp)
target_link_libraries(SensitivityTest ${catkin_LIBRARIES} ct_core)

catkin_add_gtest(InterpolationTest test/InterpolationTest.cpp)
target_link_libraries(InterpolationTest ${catkin_LIBRARIES})

catkin_add_gtest(DiscreteArrayTest test/DiscreteArrayTest.cpp)
target_link_libraries(DiscreteArrayTest ${catkin_LIBRARIES})

catkin_add_gtest(DiscreteTrajectoryTest test/DiscreteTrajectoryTest.cpp)
target_link_libraries(DiscreteTrajectoryTest ${catkin_LIBRARIES})

catkin_add_gtest(LinspaceTest test/LinspaceTest.cpp)
target_link_libraries(LinspaceTest ${catkin_LIBRARIES})

catkin_add_gtest(AutoDiffLinearizerTest test/AutoDiffLinearizerTest.cpp)
target_link_libraries(AutoDiffLinearizerTest ${catkin_LIBRARIES} ct_core pthread)

# Codegen tests cannot run in parallel. Thus we build and run all codegen tests in one
catkin_add_gtest(CodegenTests test/CodegenTests.cpp)
target_link_libraries(CodegenTests ${catkin_LIBRARIES} ct_core dl pthread)

# not really a test, but serves as example
add_executable(plotTest test/plot/plotTest.cpp)
target_link_libraries(plotTest ${catkin_LIBRARIES} ct_core)

if(QWT_ENABLED)
    add_executable(plotQwtTest test/plot/plotQwtTest.cpp)
    target_link_libraries(plotQwtTest ${catkin_LIBRARIES} ct_core ${ct_core_extra_libs} pthread)
endif()


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
