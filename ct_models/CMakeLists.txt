cmake_minimum_required(VERSION 2.8.3)

include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/compilerSettings.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/explicitTemplateHelpers.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/clang-cxx-dev-tools.cmake)

project(ct_models)

find_package(catkin REQUIRED ct_rbd)

find_package(Boost REQUIRED system filesystem)

SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wfatal-errors -std=c++11 -Wall -pthread")
SET(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

set(ct_model_libs
    quadrotorDynamics
    HyALinearizedForward
    HyAJacInverseDynamicsReverse
    hya_ik
    irb4600_ik
)

if(BUILD_HYQ_FULL)
  set(ct_model_libs ${ct_model_libs} HyQWithContactModelLinearizedForward HyQForwardZero)
endif(BUILD_HYQ_FULL)

## assemble list of libraries that contain prespecified templates
ct_configure_explicit_templates("${CMAKE_CURRENT_SOURCE_DIR}/../ct/config/explicit_templates.cfg" "${CMAKE_CURRENT_SOURCE_DIR}/prespec/" "ct_models")
message(WARNING "CT Models: Compiling the following explict template libraries: ${PRESPEC_LIB_NAMES}")

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${ct_model_libs}
    ${PRESPEC_LIB_NAMES}
  CATKIN_DEPENDS
    ct_rbd
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(HYA_CODEGEN_OUTPUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include/ct/models/HyA/codegen")
set(HYQ_CODEGEN_OUTPUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include/ct/models/HyQ/codegen")
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/include/ct/models/CodegenOutputDirs.h.in ${CMAKE_CURRENT_SOURCE_DIR}/include/ct/models/CodegenOutputDirs.h)

# add libraries for explicit template
ct_add_explicit_template_libs()

################ HyQ #################
add_executable(HyQLinearizationCodgen src/HyQ/codegen/HyQLinearizationCodgen.cpp)
target_link_libraries(HyQLinearizationCodgen ${catkin_LIBRARIES} )

if (BUILD_HYQ_FULL)
  if (NOT USE_CLANG)
    MESSAGE(WARNING "HyQ Linearization should be build with CLANG")
  endif()
    add_library(HyQWithContactModelLinearizedForward include/ct/models/HyQ/codegen/HyQWithContactModelLinearizedForward.cpp)
    target_link_libraries(HyQWithContactModelLinearizedForward ${catkin_LIBRARIES} )

    ## Forward Dynamics Forward Zero
    add_library(HyQForwardZero include/ct/models/HyQ/codegen/HyQForwardZero.cpp)
    target_link_libraries(HyQForwardZero ${catkin_LIBRARIES})
endif(BUILD_HYQ_FULL)

if (BUILD_HYQ_LINEARIZATION_TIMINGS)
  if (NOT USE_CLANG)
    MESSAGE(FATAL_ERROR "HyQ Linearization Timings need to be build with CLANG")
  endif()

  ## Forward Dynamics
  add_library(HyQWithContactModelLinearizedReverse include/ct/models/HyQ/codegen/HyQWithContactModelLinearizedReverse.cpp)
  target_link_libraries(HyQWithContactModelLinearizedReverse ${catkin_LIBRARIES} )
  add_library(HyQBareModelLinearizedForward include/ct/models/HyQ/codegen/HyQBareModelLinearizedForward.cpp)
  target_link_libraries(HyQBareModelLinearizedForward ${catkin_LIBRARIES} )
  add_library(HyQBareModelLinearizedReverse include/ct/models/HyQ/codegen/HyQBareModelLinearizedReverse.cpp)
  target_link_libraries(HyQBareModelLinearizedReverse ${catkin_LIBRARIES} )

  ## Inverse Dynamics
  add_library(HyQJacInverseDynamicsForward include/ct/models/HyQ/codegen/HyQInverseDynJacForward.cpp)
  target_link_libraries(HyQJacInverseDynamicsForward ${catkin_LIBRARIES} )
  add_library(HyQJacInverseDynamicsReverse include/ct/models/HyQ/codegen/HyQInverseDynJacReverse.cpp)
  target_link_libraries(HyQJacInverseDynamicsReverse ${catkin_LIBRARIES} )

  ## ForwardKinematics
  add_library(HyQJacForwardKinForward include/ct/models/HyQ/codegen/HyQForwardKinJacForward.cpp)
  target_link_libraries(HyQJacForwardKinForward ${catkin_LIBRARIES} )
  add_library(HyQJacForwardKinReverse include/ct/models/HyQ/codegen/HyQForwardKinJacReverse.cpp)
  target_link_libraries(HyQJacForwardKinReverse ${catkin_LIBRARIES} )


  add_executable(HyQcompareForwardReverseFD src/HyQ/codegen/compareForwardReverseFD.cpp)
  target_link_libraries(HyQcompareForwardReverseFD
                        HyQWithContactModelLinearizedForward
                        HyQWithContactModelLinearizedReverse
                        HyQBareModelLinearizedForward
                        HyQBareModelLinearizedReverse
                        ${catkin_LIBRARIES})

  add_executable(HyQcompareForwardReverseID src/HyQ/codegen/compareForwardReverseID.cpp)
  target_link_libraries(HyQcompareForwardReverseID
                        HyQJacInverseDynamicsForward
                        HyQJacInverseDynamicsReverse
                        ${catkin_LIBRARIES})

  add_executable(HyQcompareForwardReverseKin src/HyQ/codegen/compareForwardReverseKin.cpp)
  target_link_libraries(HyQcompareForwardReverseKin
                        HyQJacForwardKinForward
                        HyQJacForwardKinReverse
                        ${catkin_LIBRARIES})

    add_executable(HyQcompareForwardZero src/HyQ/codegen/compareForwardZero.cpp)
    target_link_libraries(HyQcompareForwardZero
                          HyQForwardZero
                          ${catkin_LIBRARIES})
endif(BUILD_HYQ_LINEARIZATION_TIMINGS)

################ HyA #################
#add_executable(HyALinearizationCodegen src/HyA/codegen/HyALinearizationCodeGen.cpp)
#target_link_libraries(HyALinearizationCodegen ${catkin_LIBRARIES})

add_library(HyALinearizedForward include/ct/models/HyA/codegen/HyALinearizedForward.cpp)
target_link_libraries(HyALinearizedForward ${catkin_LIBRARIES} )

add_library(HyAJacInverseDynamicsReverse include/ct/models/HyA/codegen/HyAInverseDynJacReverse.cpp)
target_link_libraries(HyAJacInverseDynamicsReverse ${catkin_LIBRARIES} )

if(BUILD_HYA_LINEARIZATION_TIMINGS)
  add_library(HyALinearizedReverse include/ct/models/HyA/codegen/HyALinearizedReverse.cpp)
  target_link_libraries(HyALinearizedReverse ${catkin_LIBRARIES} )

  add_library(HyAJacInverseDynamicsForward include/ct/models/HyA/codegen/HyAInverseDynJacForward.cpp)
  target_link_libraries(HyAJacInverseDynamicsForward ${catkin_LIBRARIES} )

  add_executable(HyAcompareForwardReverse src/HyA/codegen/compareForwardReverse.cpp)
  target_link_libraries(HyAcompareForwardReverse
                        HyALinearizedForward
                        HyALinearizedReverse
                        HyAJacInverseDynamicsForward
                        HyAJacInverseDynamicsReverse
                        ${catkin_LIBRARIES})
endif(BUILD_HYA_LINEARIZATION_TIMINGS)


################ Quadrotor #################
## Declare a cpp library for the ordinary quadrotor
add_library(quadrotorDynamics
            src/Quadrotor/quadrotor_ode.cpp
            src/Quadrotor/A_quadrotor.cpp
            src/Quadrotor/B_quadrotor.cpp
            src/Quadrotor/C_quadrotor.cpp)

catkin_add_gtest(QuadrotorWithLoadTest test/QuadrotorWithLoad/QuadrotorWithLoadTest.cpp)
target_link_libraries(QuadrotorWithLoadTest
                      ${catkin_LIBRARIES})


## Unit tests
catkin_add_gtest(HyaTest test/HyA/HyATest.cpp)
target_link_libraries(HyaTest
                      ${catkin_LIBRARIES}
                      HyALinearizedForward)

################ Inverse Kinematics #################
add_library(hya_ik
  src/HyA/transform6d.cpp
)
set_target_properties(hya_ik PROPERTIES COMPILE_FLAGS "-std=c++98 -fPIC -DIKFAST_NAMESPACE=hya_ik -DIKFAST_NO_MAIN -Wno-unused-variable")

add_library(irb4600_ik
  src/Irb4600/transform6d.cpp
)
set_target_properties(irb4600_ik PROPERTIES COMPILE_FLAGS "-std=c++98 -fPIC -DIKFAST_NAMESPACE=irb4600_ik -DIKFAST_NO_MAIN -Wno-unused-variable")


if(BUILD_EXAMPLES)
    message(STATUS "Building with examples.")

    set(CT_MODELS_EXAMPLE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/examples")
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/examples/exampleDir.h.in ${CMAKE_CURRENT_SOURCE_DIR}/examples/exampleDir.h)

    add_executable(ex_NLOC_MPC_invertedPendulum examples/InvertedPendulum/NLOC_MPC.cpp)
    target_link_libraries(ex_NLOC_MPC_invertedPendulum ${catkin_LIBRARIES})
endif()

# Tests.
catkin_add_gtest(ikfast_test test/IKFast/IKFastTest.cpp)
target_link_libraries(ikfast_test
                      hya_ik
                      irb4600_ik
                      ${catkin_LIBRARIES})


find_package(Doxygen)
if(DOXYGEN_FOUND)
    set(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/doc/ct_models.doxyfile)
    set(doxyfile ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile)

    configure_file(${doxyfile_in} ${doxyfile} @ONLY)

    add_custom_target(doc
        COMMAND ${CMAKE_COMMAND} -E echo_append "Building API Documentation..."
        COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/doc
        COMMAND ${CMAKE_COMMAND} -E echo_append "API Documentation built in ${CMAKE_CURRENT_SOURCE_DIR}/doc"
        VERBATIM)
endif()
