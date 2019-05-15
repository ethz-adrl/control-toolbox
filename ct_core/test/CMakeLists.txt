# Download and unpack googletest at configure time
configure_file(GTestCMakeLists.txt.in googletest-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
if(result)
  message(FATAL_ERROR "CMake step for googletest failed: ${result}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
if(result)
  message(FATAL_ERROR "Build step for googletest failed: ${result}")
endif()

# Prevent overriding the parent project's compiler/linker settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Add googletest directly to the build. This defines the gtest and gtest_main targets.
add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/googletest-src
                 ${CMAKE_CURRENT_BINARY_DIR}/googletest-build
                 EXCLUDE_FROM_ALL)


#macro for conveniently adding tests and linking against correct libs
macro(package_add_test TESTNAME)
    add_executable(${TESTNAME} ${ARGN})
    target_link_libraries(${TESTNAME} gtest gtest_main ct_core)
    add_test(NAME ${TESTNAME} COMMAND ${TESTNAME})
    set_target_properties(${TESTNAME} PROPERTIES FOLDER test)
    list(APPEND UNIT_TEST_TARGETS ${TESTNAME})
endmacro()


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


# Run all unit tests post-build.
add_custom_target(run_tests ALL DEPENDS ${UNIT_TEST_TARGETS})
add_custom_command(TARGET run_tests 
    COMMENT "Running tests"
    POST_BUILD COMMAND ${PROJECT_SOURCE_DIR}/test/run_tests.py ${CMAKE_CURRENT_BINARY_DIR} ${UNIT_TEST_TARGETS}
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    )
