
#macro for conveniently adding tests and linking against correct libs
macro(package_add_test TESTNAME)
    add_executable(${TESTNAME} ${ARGN})
    target_link_libraries(${TESTNAME} 
        gtest 
        gtest_main  
        ${PRESPEC_LIB_NAMES}
        ${PYTHON_LIBRARY}
        ${SNOPT_LIBS} 
        ${IPOPT_LIBS} 
        dl
        pthread
        #ct_optcon # how ugly
        )
    add_test(NAME ${TESTNAME} COMMAND ${TESTNAME})
    set_target_properties(${TESTNAME} PROPERTIES FOLDER test)
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
    
    package_add_test(ProjectedFDSystemTest systems/ProjectedFDSystemTest.cpp)
    
    package_add_test(RBDLinearizerTest systems/linear/RBDLinearizerTest.cpp)
    
    package_add_test(EEKinematicsTest robot/kinematics/EEKinematicsTest.cpp)
    
    package_add_test(EEContactModelTest physics/EEContactModelTest.cpp)
    
    package_add_test(TaskSpaceCfTest robot/costfunction/TaskspaceCostFunctionTest.cpp)
    
    package_add_test(rbdJITtests robot/costfunction/rbdJITtests.cpp)
    
    package_add_test(kindrJITtest robot/costfunction/kindrJITtest.cpp)
    
    package_add_test(jacobianTests robot/jacobian/JacobianTests.cpp)
    
    if(BUILD_WITH_IPOPT_SUPPORT)
        package_add_test(FixBaseIKTest robot/kinematics/FixBaseIKTest.cpp)
    endif()
    
    # todo transform this into a proper test
    #add_executable(simpleArmTrajectoryTest src/SimpleArmTrajectoryGeneratorTest.cpp)

endif() # build tests