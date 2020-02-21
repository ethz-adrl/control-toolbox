
#macro for conveniently adding tests and linking against correct libs
macro(package_add_test TESTNAME)
    add_executable(${TESTNAME} ${ARGN})
    target_link_libraries(${TESTNAME} gtest gtest_main ct_rbd)
    add_test(NAME ${TESTNAME} COMMAND ${TESTNAME})
    set_target_properties(${TESTNAME} PROPERTIES FOLDER test)
    list(APPEND UNIT_TEST_TARGETS ${TESTNAME})
endmacro()


if(BUILD_TESTS)

    package_add_test(RigidBodyPoseTest state/RigidBodyPoseTest.cpp)
    
    package_add_test(JointStateTest state/JointStateTest.cpp)
    
    package_add_test(RobotStateTest state/RobotStateTest.cpp)
    
    package_add_test(RobCoGenContainerTest robot/robcogen/RobCoGenContainerTest.cpp)
    
    package_add_test(KinematicsTest robot/kinematics/KinematicsTest.cpp)
    
    package_add_test(KinematicsTestAd robot/kinematics/KinematicsTestAd.cpp)
    
    package_add_test(OperationalSpaceTest operationalSpace/OperationalSpaceTest.cpp)
    
    package_add_test(DataMapTests robot/dynamics/DataMapTests.cpp)
    
    package_add_test(DynamicsTest robot/dynamics/DynamicsTests.cpp)
    
    package_add_test(DynamicsTestFixBase robot/dynamics/DynamicsTestsFixBase.cpp)
    
    package_add_test(FloatingBaseFDSystemTest systems/FloatingBaseFDSystemTest.cpp)
    
    package_add_test(FixBaseFDSystemTest systems/FixBaseFDSystemTest.cpp)
    
    
    package_add_test(RBDLinearizerTest systems/linear/RBDLinearizerTest.cpp)
    
    package_add_test(EEKinematicsTest robot/kinematics/EEKinematicsTest.cpp)
    
    package_add_test(EEContactModelTest physics/EEContactModelTest.cpp)
    
    package_add_test(jacobianTests robot/jacobian/JacobianTests.cpp)
    
    if(CPPADCG)
        message(STATUS "ct_rbd: building unit tests requiring CPPADCG")
        package_add_test(ProjectedFDSystemTest systems/ProjectedFDSystemTest.cpp)
        package_add_test(TaskSpaceCfTest robot/costfunction/TaskspaceCostFunctionTest.cpp)
        package_add_test(rbdJITtests robot/costfunction/rbdJITtests.cpp)
        package_add_test(kindrJITtest robot/costfunction/kindrJITtest.cpp)
    endif()
    
    
    if(BUILD_WITH_IPOPT_SUPPORT)
        message(STATUS "ct_rbd: building unit tests requiring IPOPT")
        package_add_test(FixBaseIKTest robot/kinematics/FixBaseIKTest.cpp)
    endif()
    
    # todo transform this into a proper test
    #add_executable(simpleArmTrajectoryTest src/SimpleArmTrajectoryGeneratorTest.cpp)
    
    
    # Run all unit tests post-build.
    add_custom_target(run_tests DEPENDS ${UNIT_TEST_TARGETS})
    add_custom_command(TARGET run_tests 
        COMMENT "Running tests"
        POST_BUILD COMMAND ${PROJECT_SOURCE_DIR}/../ct/run_tests.py ${CMAKE_CURRENT_BINARY_DIR} ${UNIT_TEST_TARGETS}
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        )
    
    ## install tests
    include(GNUInstallDirs)
    install(
        TARGETS ${UNIT_TEST_TARGETS}
        RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}/ct_rbd
        )
endif()