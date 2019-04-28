
#macro for conveniently adding tests and linking against correct libs
macro(package_add_test TESTNAME)
    add_executable(${TESTNAME} ${ARGN})
    target_link_libraries(${TESTNAME} gtest gtest_main ct_core ${PRESPEC_LIB_NAMES})
    add_test(NAME ${TESTNAME} COMMAND ${TESTNAME})
    set_target_properties(${TESTNAME} PROPERTIES FOLDER test)
endmacro()


if(BUILD_TESTS)

    package_add_test(NoiseTest NoiseTest.cpp)
    package_add_test(SecondOrderSystemTest SecondOrderSystemTest.cpp)
    package_add_test(IntegrationTest integration/IntegrationTest.cpp)
    package_add_test(IntegratorComparison integration/IntegratorComparison.cpp)
    package_add_test(SymplecticIntegrationTest integration/SymplecticIntegrationTest.cpp)
    package_add_test(SystemDiscretizerTest integration/SystemDiscretizerTest.cpp)
    #package_add_test(SensitivityTest integration/sensitivity/SensitivityTest.cpp) #todo make this a proper test
    package_add_test(InterpolationTest InterpolationTest.cpp)
    package_add_test(DiscreteArrayTest DiscreteArrayTest.cpp)
    package_add_test(DiscreteTrajectoryTest DiscreteTrajectoryTest.cpp)
    package_add_test(LinspaceTest LinspaceTest.cpp)
    package_add_test(AutoDiffLinearizerTest AutoDiffLinearizerTest.cpp)
    package_add_test(SwitchingTest switching/SwitchingTest.cpp)
    package_add_test(SwitchedControlledSystemTest switching/SwitchedControlledSystemTest.cpp)
    package_add_test(SwitchedDiscreteControlledSystemTest switching/SwitchedDiscreteControlledSystemTest.cpp)
    package_add_test(MatrixInversionTest math/MatrixInversionTest.cpp)
    package_add_test(CodegenTests CodegenTests.cpp)

    ## unit tests for prespec libraries
    if(USE_PRESPEC)
        package_add_test(IntegrationTestPrespec integration/IntegrationTestPrespec.cpp)
        package_add_test(IntegratorComparisonPrespec integration/IntegratorComparisonPrespec.cpp)
    endif(USE_PRESPEC)

endif() # build tests
