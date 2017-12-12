cmake_minimum_required (VERSION 2.6)

include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/compilerSettings.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/explicitTemplateHelpers.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/clang-cxx-dev-tools.cmake)

project(ct_ik)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -pthread -fopenmp -std=c++11 -Wall")
SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -pthread -std=c++11 -Wall")
SET(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

find_package(catkin REQUIRED ct_rbd ct_models)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## assemble list of libraries that contain prespecified templates
ct_configure_explicit_templates("${CMAKE_CURRENT_SOURCE_DIR}/../ct/config/explicit_templates.cfg" "${CMAKE_CURRENT_SOURCE_DIR}/prespec/" "ct_core")
message(WARNING "CT Core: Compiling the following explict template libraries: ${PRESPEC_LIB_NAMES}")

set(INC_DIRS ${catkin_INCLUDE_DIRS}  "${CMAKE_CURRENT_SOURCE_DIR}/include")

catkin_package(
  INCLUDE_DIRS ${INC_DIRS}
  LIBRARIES hya_ik
  CATKIN_DEPENDS ct_rbd
  DEPENDS system_lib
)

## Declare a C++ library
add_library(hya_ik
  src/HyA/transform6d.cpp
)
set_target_properties(hya_ik PROPERTIES COMPILE_FLAGS "-fPIC -DIKFAST_NAMESPACE=hya_ik -DIKFAST_NO_MAIN -Wno-unused-variable")

add_library(irb4600_ik
  src/Irb4600/transform6d.cpp
)
set_target_properties(irb4600_ik PROPERTIES COMPILE_FLAGS "-fPIC -DIKFAST_NAMESPACE=irb4600_ik -DIKFAST_NO_MAIN -Wno-unused-variable")

catkin_add_gtest(ik_test test/IKTest.cpp)
target_link_libraries(ik_test
                      hya_ik
                      irb4600_ik
                      ${catkin_LIBRARIES})

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
