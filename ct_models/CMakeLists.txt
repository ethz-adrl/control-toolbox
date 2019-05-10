cmake_minimum_required (VERSION 3.3)

include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/compilerSettings.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/explicitTemplateHelpers.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/clang-cxx-dev-tools.cmake)

project(ct_models VERSION 3.0.1 LANGUAGES CXX)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wfatal-errors -std=c++14 -Wall -Wno-unknown-pragmas")
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

find_package(ct_rbd REQUIRED)
find_package(Boost REQUIRED system filesystem)

## add compiler definitions exported by ct_optcon
add_definitions(${ct_rbd_COMPILE_DEFINITIONS})


## define the directories to be included in all ct_rbd targets
set(ct_models_target_include_dirs
    ${ct_rbd_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)


## define placeholder for ct model libraries
set(CT_MODEL_LIBS "")

set(IP_CODEGEN_OUTPUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include/ct/models/InvertedPendulum/codegen")
set(HYA_CODEGEN_OUTPUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include/ct/models/HyA/codegen")
set(HYQ_CODEGEN_OUTPUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include/ct/models/HyQ/codegen")
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/include/ct/models/CodegenOutputDirs.h.in ${CMAKE_CURRENT_SOURCE_DIR}/include/ct/models/CodegenOutputDirs.h)

################ HyQ #################
add_executable(HyQLinearizationCodegen src/HyQ/codegen/HyQLinearizationCodegen.cpp)
target_include_directories(HyQLinearizationCodegen PUBLIC ${ct_models_target_include_dirs})
target_link_libraries(HyQLinearizationCodegen ct_rbd)

if (BUILD_HYQ_FULL)
  if (NOT USE_CLANG)
    MESSAGE(WARNING "HyQ Linearization should be build with CLANG")
  endif()
    add_library(HyQWithContactModelLinearizedForward include/ct/models/HyQ/codegen/HyQWithContactModelLinearizedForward.cpp)
    target_link_libraries(HyQWithContactModelLinearizedForward ct_rbd)
    list(APPEND CT_MODEL_LIBS HyQWithContactModelLinearizedForward)

    ## Forward Dynamics Forward Zero
    add_library(HyQForwardZero include/ct/models/HyQ/codegen/HyQForwardZero.cpp)
    target_link_libraries(HyQForwardZero ct_rbd)
    list(APPEND CT_MODEL_LIBS HyQForwardZero)
endif(BUILD_HYQ_FULL)

if(BUILD_HYQ_LINEARIZATION_TIMINGS)
  if (NOT USE_CLANG)
    MESSAGE(WARNING "HyQ Linearization Timings need to be build with CLANG")
  endif()

  ## Forward Dynamics
  add_library(HyQWithContactModelLinearizedReverse include/ct/models/HyQ/codegen/HyQWithContactModelLinearizedReverse.cpp)
  target_link_libraries(HyQWithContactModelLinearizedReverse ct_rbd)
  list(APPEND CT_MODEL_LIBS HyQWithContactModelLinearizedReverse)

  add_library(HyQBareModelLinearizedForward include/ct/models/HyQ/codegen/HyQBareModelLinearizedForward.cpp)
  target_link_libraries(HyQBareModelLinearizedForward ct_rbd )
  list(APPEND CT_MODEL_LIBS HyQBareModelLinearizedForward)

  add_library(HyQBareModelLinearizedReverse include/ct/models/HyQ/codegen/HyQBareModelLinearizedReverse.cpp)
  target_link_libraries(HyQBareModelLinearizedReverse ct_rbd )
  list(APPEND CT_MODEL_LIBS HyQBareModelLinearizedReverse)

  ## Inverse Dynamics
  add_library(HyQJacInverseDynamicsForward include/ct/models/HyQ/codegen/HyQInverseDynJacForward.cpp)
  target_link_libraries(HyQJacInverseDynamicsForward ct_rbd )
  list(APPEND CT_MODEL_LIBS HyQJacInverseDynamicsForward)

  add_library(HyQJacInverseDynamicsReverse include/ct/models/HyQ/codegen/HyQInverseDynJacReverse.cpp)
  target_link_libraries(HyQJacInverseDynamicsReverse ct_rbd )
  list(APPEND CT_MODEL_LIBS HyQJacInverseDynamicsReverse)

  ## ForwardKinematics
  add_library(HyQJacForwardKinForward include/ct/models/HyQ/codegen/HyQForwardKinJacForward.cpp)
  target_link_libraries(HyQJacForwardKinForward ct_rbd )
  list(APPEND CT_MODEL_LIBS HyQJacForwardKinForward)

  add_library(HyQJacForwardKinReverse include/ct/models/HyQ/codegen/HyQForwardKinJacReverse.cpp)
  target_link_libraries(HyQJacForwardKinReverse ct_rbd )
  list(APPEND CT_MODEL_LIBS HyQJacForwardKinReverse)

  add_executable(HyQcompareForwardReverseFD src/HyQ/codegen/compareForwardReverseFD.cpp)
  target_link_libraries(HyQcompareForwardReverseFD
                        HyQWithContactModelLinearizedForward
                        HyQWithContactModelLinearizedReverse
                        HyQBareModelLinearizedForward
                        HyQBareModelLinearizedReverse
                        ct_rbd)

  add_executable(HyQcompareForwardReverseID src/HyQ/codegen/compareForwardReverseID.cpp)
  target_link_libraries(HyQcompareForwardReverseID
                        HyQJacInverseDynamicsForward
                        HyQJacInverseDynamicsReverse
                        ct_rbd)

  add_executable(HyQcompareForwardReverseKin src/HyQ/codegen/compareForwardReverseKin.cpp)
  target_link_libraries(HyQcompareForwardReverseKin
                        HyQJacForwardKinForward
                        HyQJacForwardKinReverse
                        ct_rbd)

  add_executable(HyQcompareForwardZero src/HyQ/codegen/compareForwardZero.cpp)
  target_link_libraries(HyQcompareForwardZero
                          HyQForwardZero
                          ct_rbd)
endif(BUILD_HYQ_LINEARIZATION_TIMINGS)

########## Inverted Pendulum #########
add_executable(InvertedPendulumWithActuatorCodeGen src/InvertedPendulum/codegen/InvertedPendulumWithActuatorCodeGen.cpp)
target_include_directories(InvertedPendulumWithActuatorCodeGen PUBLIC ${ct_models_target_include_dirs})
target_link_libraries(InvertedPendulumWithActuatorCodeGen ct_rbd)
list(APPEND CT_MODEL_LIBS InvertedPendulumWithActuatorCodeGen)

add_library(InvertedPendulumActDynLinearizedForward include/ct/models/InvertedPendulum/codegen/InvertedPendulumActDynLinearizedForward.cpp)
target_include_directories(InvertedPendulumWithActuatorCodeGen PUBLIC ${ct_models_target_include_dirs})
target_link_libraries(InvertedPendulumActDynLinearizedForward ct_rbd)
list(APPEND CT_MODEL_LIBS InvertedPendulumActDynLinearizedForward)


################ HyA #################
add_executable(HyALinearizationCodegen src/HyA/codegen/HyALinearizationCodeGen.cpp)
target_include_directories(HyALinearizationCodegen PUBLIC ${ct_models_target_include_dirs})
target_link_libraries(HyALinearizationCodegen ct_rbd)

add_library(HyALinearizedForward include/ct/models/HyA/codegen/HyALinearizedForward.cpp)
target_include_directories(HyALinearizedForward PUBLIC ${ct_models_target_include_dirs})
target_link_libraries(HyALinearizedForward ct_rbd )
list(APPEND CT_MODEL_LIBS HyALinearizedForward)

add_library(HyAJacInverseDynamicsReverse include/ct/models/HyA/codegen/HyAInverseDynJacReverse.cpp)
target_include_directories(HyAJacInverseDynamicsReverse PUBLIC ${ct_models_target_include_dirs})
target_link_libraries(HyAJacInverseDynamicsReverse ct_rbd)
list(APPEND CT_MODEL_LIBS HyAJacInverseDynamicsReverse)

if(BUILD_HYA_LINEARIZATION_TIMINGS)
    add_library(HyALinearizedReverse include/ct/models/HyA/codegen/HyALinearizedReverse.cpp)
    target_include_directories(HyALinearizedReverse PUBLIC ${ct_models_target_include_dirs})
    target_link_libraries(HyALinearizedReverse ct_rbd )
    list(APPEND CT_MODEL_LIBS HyALinearizedReverse)

    add_library(HyAJacInverseDynamicsForward include/ct/models/HyA/codegen/HyAInverseDynJacForward.cpp)
    target_include_directories(HyAJacInverseDynamicsForward PUBLIC ${ct_models_target_include_dirs})
    target_link_libraries(HyAJacInverseDynamicsForward ct_rbd )
    list(APPEND CT_MODEL_LIBS HyAJacInverseDynamicsForward)

    add_executable(HyAcompareForwardReverse src/HyA/codegen/compareForwardReverse.cpp)
    target_include_directories(HyAcompareForwardReverse PUBLIC ${ct_models_target_include_dirs})
    target_link_libraries(HyAcompareForwardReverse
        HyALinearizedForward
        HyALinearizedReverse
        HyAJacInverseDynamicsForward
        HyAJacInverseDynamicsReverse
        ct_rbd
        )
endif(BUILD_HYA_LINEARIZATION_TIMINGS)


## Declare a cpp library for the ordinary quadrotor
add_library(quadrotorDynamics
    src/Quadrotor/quadrotor_ode.cpp
    src/Quadrotor/A_quadrotor.cpp
    src/Quadrotor/B_quadrotor.cpp
    src/Quadrotor/C_quadrotor.cpp
    )
target_include_directories(quadrotorDynamics PUBLIC ${ct_models_target_include_dirs})
target_link_libraries(quadrotorDynamics ct_core)
list(APPEND CT_MODEL_LIBS quadrotorDynamics)


## Inverse Kinematics

add_library(hya_ik src/HyA/transform6d.cpp)
target_include_directories(hya_ik PUBLIC ${ct_models_target_include_dirs})
set_target_properties(hya_ik PROPERTIES COMPILE_FLAGS "-std=c++98 -fPIC -DIKFAST_NAMESPACE=hya_ik -DIKFAST_NO_MAIN -Wno-unused-variable")
target_link_libraries(hya_ik ct_rbd)
list(APPEND CT_MODEL_LIBS hya_ik)

add_library(irb4600_ik src/Irb4600/transform6d.cpp)
target_include_directories(irb4600_ik PUBLIC ${ct_models_target_include_dirs})
set_target_properties(irb4600_ik PROPERTIES COMPILE_FLAGS "-std=c++98 -fPIC -DIKFAST_NAMESPACE=irb4600_ik -DIKFAST_NO_MAIN -Wno-unused-variable")
target_link_libraries(irb4600_ik ct_rbd)
list(APPEND CT_MODEL_LIBS irb4600_ik)


############
# EXAMPLES #
############
if(BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()


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

## get all compile definitions registered so far and create variable
get_directory_property( CompileDefs DIRECTORY ${CMAKE_SOURCE_DIR} COMPILE_DEFINITIONS )
foreach( d ${CompileDefs} )
    set(ct_models_compile_definitions  ${ct_models_compile_definitions} -D${d} ) # recursively add compile definitions
endforeach()

## generate files required for installation
configure_file(cmake/ct_modelsConfig.cmake.in "${PROJECT_BINARY_DIR}/ct_modelsConfig.cmake" @ONLY)

## copy the header files
install(DIRECTORY include/ct/models DESTINATION include/ct)

## copy the cmake files required for find_package()
install(FILES "${PROJECT_BINARY_DIR}/ct_modelsConfig.cmake" DESTINATION "share/ct_models/cmake")

## install library and targets
install(
    TARGETS ${CT_MODEL_LIBS}
    EXPORT ct_models_export
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )

## create the ct_rbd.cmake file which holds target includes and dependencies
install (EXPORT ct_models_export DESTINATION share/ct_models/cmake)


#################
# DOCUMENTATION #
#################
add_subdirectory(doc)