cmake_minimum_required (VERSION 2.6) 

include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/compilerSettings.cmake)

project (ct_rbd) 

SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wfatal-errors -std=c++11 -Wall")

find_package(catkin REQUIRED ct_core ct_optcon cmake_modules)
find_package(kindr)
find_package(Eigen3)

include_directories(
  ${catkin_INCLUDE_DIRS}  
)

## assemble list of libraries that contain prespecified templates
set(PRESPEC_LIBS "")
file(GLOB files "prespec/*.cpp")
foreach(file ${files})
  get_filename_component(raw_filename ${file} NAME_WE)
  LIST(APPEND PRESPEC_LIBS "ct_rbd_${raw_filename}")
endforeach()

catkin_package(
   INCLUDE_DIRS
        ${kindr_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        include
        include/ct/rbd
        test
   LIBRARIES
        ${PRESPEC_LIBS}
   CATKIN_DEPENDS 
       roscpp
       ct_core
       ct_optcon
       kindr
)

include_directories(
    ${kindr_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    include/ct/rbd
    test/models/testhyq/generated
    include
    test
)


##compile template prespec libs
file(GLOB files "prespec/*.cpp")
foreach(file ${files})
  get_filename_component(raw_filename ${file} NAME_WE)
  add_library(ct_rbd_${raw_filename}
   ${file}
  )
  target_link_libraries(ct_rbd_${raw_filename} ${catkin_LIBRARIES})
endforeach()



## TESTING    
    
catkin_add_gtest(RigidBodyPoseTest test/state/RigidBodyPoseTest.cpp)
target_link_libraries(RigidBodyPoseTest ${catkin_LIBRARIES})
    
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

catkin_add_gtest(TaskSpaceCfTest test/robot/costfunction/TaskspaceCostFunctionTest.cpp)
target_link_libraries(TaskSpaceCfTest ${catkin_LIBRARIES})

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