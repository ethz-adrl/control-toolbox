
#macro for conveniently adding tests and linking against correct libs
macro(package_add_test TESTNAME)
    add_executable(${TESTNAME} ${ARGN})
    target_link_libraries(${TESTNAME} gtest gtest_main ct_optcon)
    add_test(NAME ${TESTNAME} COMMAND ${TESTNAME})
    set_target_properties(${TESTNAME} PROPERTIES FOLDER test)
    list(APPEND UNIT_TEST_TARGETS ${TESTNAME})
endmacro()


if(BUILD_TESTS)

    ## create directory includes
    get_filename_component(NLOC_TEST_DIR "nloc" ABSOLUTE)
    configure_file(nloc/nloc_test_dir.h.in test/nloc/nloc_test_dir.h)
    include_directories(${CMAKE_CURRENT_BINARY_DIR}/test/nloc)
    get_filename_component(COSTFUNCTION_TEST_DIR "costfunction" ABSOLUTE)
    configure_file(costfunction/costfunction_test_dir.h.in test/costfunction/costfunction_test_dir.h)
    include_directories(${CMAKE_CURRENT_BINARY_DIR}/test/costfunction)
    get_filename_component(MPC_TEST_MAT_DIR "mpc/matfiles" ABSOLUTE)
    configure_file(mpc/mpcTestSettings.h.in test/mpc/mpcTestSettings.h)
    include_directories(${CMAKE_CURRENT_BINARY_DIR}/test/mpc)
    get_filename_component(DMS_OSC_TEST_MAT_DIR "dms/oscillator/matfiles" ABSOLUTE)
    configure_file(dms/oscillator/oscDMSTest_settings.h.in test/dms/oscillator/oscDMSTest_settings.h)
    include_directories(${CMAKE_CURRENT_BINARY_DIR}/test/dms/oscillator)
    
    ## some legacy executables (TODO: make example or make test)
    #add_executable(SymplecticTest nloc/SymplecticTest.cpp)
    #target_link_libraries(SymplecticTest ct_optcon)
    
    add_executable(matFilesGenerator dms/oscillator/matfiles/matFilesGenerator.cpp) # todo convert to proper test
    target_link_libraries(matFilesGenerator ct_optcon)
    
    
    ## tests
    package_add_test(LqrTest lqr/LqrTest.cpp)
    package_add_test(iLQRTest nloc/nonlinear/iLQRTest.cpp)
    package_add_test(LinearSystemTest nloc/LinearSystemTest.cpp)
    package_add_test(NonlinearSystemTest nloc/nonlinear/NonlinearSystemTest.cpp)
    package_add_test(NLOC_MPCTest mpc/NLOC_MPCTest.cpp)
    #package_add_test(SymplecticTest nloc/SymplecticTest.cpp) # make proper test
    package_add_test(SparseBoxConstraintTest constraint/SparseBoxConstraintTest.cpp)
    if(CPPADCG)
        message(STATUS "ct_optcon: building unit tests requiring CPPADCG")
        package_add_test(constraint_comparison constraint/ConstraintComparison.cpp)
        package_add_test(constraint_test constraint/ConstraintTest.cpp)
        package_add_test(CostFunctionTests costfunction/CostFunctionTests.cpp)
        package_add_test(LoadFromFileTest costfunction/LoadFromFileTest.cpp)
    endif()
    
    package_add_test(dms_test dms/oscillator/oscDMSTest.cpp)
    package_add_test(dms_test_all_var dms/oscillator/oscDMSTestAllVariants.cpp)
    package_add_test(system_interface_test system_interface/SystemInterfaceTest.cpp)
    
    if(HPIPM)
        message(STATUS "ct_optcon: building unit tests requiring HPIPM")
        ## some legacy executables (TODO: make example or make test)
        add_executable(LQOCSolverTiming solver/linear/LQOCSolverTiming.cpp)
        target_link_libraries(LQOCSolverTiming ct_optcon)
        
        package_add_test(LQOCSolverTest solver/linear/LQOCSolverTest.cpp)
        package_add_test(ConstrainedLQOCSolverTest solver/linear/ConstrainedLQOCSolverTest.cpp)
        package_add_test(LinearSystemSolverComparison nloc/LinearSystemSolverComparison.cpp)
        if(CPPADCG)
            message(STATUS "ct_optcon: building unit tests requiring CPPADCG and HPIPM")
            package_add_test(ConstrainedNLOCTest nloc/constrained/ConstrainedNLOCTest.cpp)
        endif()
    endif()
    
    
    ## additional prespec tests
    if(USE_PRESPEC)    
        package_add_test(LqrTestPrespec lqr/LqrTestPrespec.cpp)
        package_add_test(LinearSystemTestPrespec nloc/LinearSystemTestPrespec.cpp)    
        package_add_test(constraint_test_prespec constraint/ConstraintTestPrespec.cpp)
        package_add_test(NLOC_MPCTest_prespec mpc/NLOC_MPCTest_prespec.cpp)
        package_add_test(system_interface_test_prespec system_interface/SystemInterfaceTest-Prespec.cpp)
        #add_executable(SymplecticTestPrespec nloc/SymplecticTestPrespec.cpp)
        if(HPIPM)
            package_add_test(LQOCSolverTestPrespec solver/linear/LQOCSolverTestPrespec.cpp)
        endif(HPIPM)
    endif(USE_PRESPEC)
    
    
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
        RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}/ct_optcon
        )
endif()