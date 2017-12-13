cmake_minimum_required (VERSION 2.6)

include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/compilerSettings.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/explicitTemplateHelpers.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/clang-cxx-dev-tools.cmake)

project (ct_rbd)

SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wfatal-errors -std=c++11 -Wall")
SET(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

find_package(catkin REQUIRED ct_core ct_optcon)
find_package(kindr)


## assemble list of libraries that contain prespecified templates
ct_configure_explicit_templates("${CMAKE_CURRENT_SOURCE_DIR}/../ct/config/explicit_templates.cfg" "${CMAKE_CURRENT_SOURCE_DIR}/prespec/" "ct_rbd")
message(WARNING "CT RBD: Compiling the following explict template libraries: ${PRESPEC_LIB_NAMES}")

#used later for clang-tidy
set(INC_DIRS ${catkin_INCLUDE_DIRS} ${kindr_INCLUDE_DIRS} "${CMAKE_CURRENT_SOURCE_DIR}/include" "${CMAKE_CURRENT_SOURCE_DIR}/include/ct/rbd")


catkin_package(
  INCLUDE_DIRS
    ${kindr_INCLUDE_DIRS}
    include
    include/ct/rbd
    include/ct/rbd/robot/kinematics/ikfast
    test
  LIBRARIES
    ${PRESPEC_LIB_NAMES}
  CATKIN_DEPENDS
    ct_core
    ct_optcon
    kindr
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${kindr_INCLUDE_DIRS}
  include/ct/rbd
  include/ct/rbd/robot/kinematics/ikfast
  test/models/testhyq/generated
  include
  test
)


# add libraries for explicit template
ct_add_explicit_template_libs()


## TESTING

catkin_add_gtest(RigidBodyPoseTest test/state/RigidBodyPoseTest.cpp)
target_link_libraries(RigidBodyPoseTest ${catkin_LIBRARIES})

catkin_add_gtest(JointStateTest test/state/JointStateTest.cpp)
target_link_libraries(JointStateTest ${catkin_LIBRARIES})

catkin_add_gtest(RobCoGenContainerTest test/robot/robcogen/RobCoGenContainerTest.cpp)
target_link_libraries(RobCoGenContainerTest ${catkin_LIBRARIES})

catkin_add_gtest(KinematicsTest test/robot/kinematics/KinematicsTest.cpp)
target_link_libraries(KinematicsTest ${catkin_LIBRARIES})

catkin_add_gtest(KinematicsTestAd test/robot/kinematics/KinematicsTestAd.cpp)
target_link_libraries(KinematicsTestAd ${catkin_LIBRARIES})

catkin_add_gtest(OperationalSpaceTest test/operationalSpace/OperationalSpaceTest.cpp)
target_link_libraries(OperationalSpaceTest ${catkin_LIBRARIES})

catkin_add_gtest(DataMapTests test/robot/dynamics/DataMapTests.cpp)
target_link_libraries(DataMapTests ${catkin_LIBRARIES})

catkin_add_gtest(DynamicsTest test/robot/dynamics/DynamicsTests.cpp)
target_link_libraries(DynamicsTest ${catkin_LIBRARIES})

catkin_add_gtest(DynamicsTestFixBase test/robot/dynamics/DynamicsTestsFixBase.cpp)
target_link_libraries(DynamicsTestFixBase ${catkin_LIBRARIES})

catkin_add_gtest(FloatingBaseFDSystemTest test/systems/FloatingBaseFDSystemTest.cpp)
target_link_libraries(FloatingBaseFDSystemTest ${catkin_LIBRARIES})

catkin_add_gtest(FixBaseFDSystemTest test/systems/FixBaseFDSystemTest.cpp)
target_link_libraries(FixBaseFDSystemTest ${catkin_LIBRARIES})

catkin_add_gtest(ProjectedFDSystemTest test/systems/ProjectedFDSystemTest.cpp)
target_link_libraries(ProjectedFDSystemTest ${catkin_LIBRARIES})

catkin_add_gtest(RBDLinearizerTest test/systems/linear/RBDLinearizerTest.cpp)
target_link_libraries(RBDLinearizerTest ${catkin_LIBRARIES})

catkin_add_gtest(EEKinematicsTest test/robot/kinematics/EEKinematicsTest.cpp)
target_link_libraries(EEKinematicsTest ${catkin_LIBRARIES})

catkin_add_gtest(EEContactModelTest test/physics/EEContactModelTest.cpp)
target_link_libraries(EEContactModelTest ${catkin_LIBRARIES})

# todo make unit test again
catkin_add_gtest(TaskSpaceCfTest test/robot/costfunction/TaskspaceCostFunctionTest.cpp)
target_link_libraries(TaskSpaceCfTest ${catkin_LIBRARIES})
catkin_add_gtest(rbdJITtests test/robot/costfunction/rbdJITtests.cpp)
target_link_libraries(rbdJITtests ${catkin_LIBRARIES})
catkin_add_gtest(kindrJITtest test/robot/costfunction/kindrJITtest.cpp)
target_link_libraries(kindrJITtest ${catkin_LIBRARIES})

#catkin_add_gtest(ControllerTests test/robot/control/ControllerTests.cpp)
#target_link_libraries(ControllerTests ${catkin_LIBRARIES})

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

ct_configure_clang_tidy(INC_DIRS)
