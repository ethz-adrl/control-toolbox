macro(package_add_test TESTNAME)
    add_executable(${TESTNAME} ${ARGN})
    add_test(NAME ${TESTNAME} COMMAND ${TESTNAME})
    set_target_properties(${TESTNAME} PROPERTIES FOLDER test)
    list(APPEND UNIT_TEST_TARGETS ${TESTNAME})
endmacro()

if(BUILD_TESTS)
    
    package_add_test(QuadrotorWithLoadTest QuadrotorWithLoad/QuadrotorWithLoadTest.cpp)
    target_include_directories(QuadrotorWithLoadTest PUBLIC ${ct_models_target_include_dirs})
    target_link_libraries(QuadrotorWithLoadTest gtest gtest_main ct_rbd)
    
    package_add_test(HyaTest HyA/HyATest.cpp)
    target_include_directories(HyaTest PUBLIC ${ct_models_target_include_dirs})
    target_link_libraries(HyaTest gtest gtest_main ct_rbd HyALinearizedForward)
    
    package_add_test(ikfast_test_irb IKFast/IKFastTestIrb4600.cpp)
    target_include_directories(ikfast_test_irb PUBLIC ${ct_models_target_include_dirs})
    target_link_libraries(ikfast_test_irb gtest gtest_main irb4600_ik ct_rbd)
    
    package_add_test(ikfast_test_hya IKFast/IKFastTestHyA.cpp)
    target_include_directories(ikfast_test_hya PUBLIC ${ct_models_target_include_dirs})
    target_link_libraries(ikfast_test_hya gtest gtest_main hya_ik ct_rbd)
    
    
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
        RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}/ct_models
        )
endif()