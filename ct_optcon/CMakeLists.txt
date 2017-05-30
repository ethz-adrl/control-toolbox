cmake_minimum_required (VERSION 2.6) 

include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/compilerSettings.cmake)

project (ct_optcon) 

find_package(catkin REQUIRED 
    ct_core
    cmake_modules 
    std_msgs
    message_generation 
    message_runtime
)
find_package(Eigen REQUIRED)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wfatal-errors -std=c++11 -Wall")

option(MATLAB "Compile with matlab support" OFF)
option(MATLAB_FULL_LOG "Expose all variables to Matlab (very slow)" OFF)
option(DEBUG_PRINT "Print debug messages" OFF)

option(BUILD_WITH_IPOPT_SUPPORT "Build with IPOPT support" true)
option(BUILD_WITH_SNOPT_SUPPORT "Build with SNOPT support" true)

if(DEBUG_PRINT)
    message(WARNING "WARNING: Debug Print ON")
    add_definitions ( -DDEBUG_PRINT )
endif(DEBUG_PRINT)

if(MATLAB_FULL_LOG)
    message(WARNING "WARNING: Compiling with full log to matlab. Execution will be very slow.")
    set(MATLAB ON)
    add_definitions ( -DMATLAB_FULL_LOG )
endif(MATLAB_FULL_LOG)

if(MATLAB)
    message(WARNING "MATLAB support ON")
    find_package(matlab_cpp_interface QUIET)
    if(matlab_cpp_interface_FOUND)
        include_directories(${matlab_cpp_interface_INCLUDE_DIRS})
        set(MATLAB_LIB_DIR ${matlab_cpp_interface_LIBRARIES})
        add_definitions( -DMATLAB )
    endif()    
endif(MATLAB)
set(MATLAB_LIBS ${MATLAB_LIB_DIR})


if(BUILD_WITH_IPOPT_SUPPORT)
    if(NOT DEFINED ENV{IPOPT_SOURCE_DIR})
        message(FATAL_ERROR "ERROR: Ipopt source directory environment variable not set! Set IPOPT_SOURCE_DIR environment variable!")
    endif(NOT DEFINED ENV{IPOPT_SOURCE_DIR})

    message(WARNING "Build with IPOPT")
    include_directories( "$ENV{IPOPT_SOURCE_DIR}/build/include/coin")
    add_definitions( -DBUILD_WITH_IPOPT_SUPPORT )
    link_directories($ENV{IPOPT_SOURCE_DIR}/build/lib)
    set(IPOPT_LIBS ipopt dl coinmumps coinhsl lapack blas
        m quadmath coinmetis)

endif(BUILD_WITH_IPOPT_SUPPORT)

if(BUILD_WITH_SNOPT_SUPPORT)
    if(NOT DEFINED ENV{SNOPT_SOURCE_DIR})
        message(FATAL_ERROR "ERROR: Snopt source directory environment variable not set! Set SNOPT_SOURCE_DIR environment variable!")
    endif(NOT DEFINED ENV{SNOPT_SOURCE_DIR})
    message(WARNING "Build with SNOPT")
    include_directories( "$ENV{SNOPT_SOURCE_DIR}/include")

    add_definitions( -DBUILD_WITH_SNOPT_SUPPORT )
    find_library(SNOPT_LIBRARY1 snopt7_cpp   $ENV{SNOPT_SOURCE_DIR}/lib REQUIRED)
    find_library(SNOPT_LIBRARY2 snopt7       $ENV{SNOPT_SOURCE_DIR}/lib REQUIRED)
    set(SNOPT_LIBS ${SNOPT_LIBRARY1} ${SNOPT_LIBRARY2})

endif(BUILD_WITH_SNOPT_SUPPORT)

set(NLP_LIBS ${IPOPT_LIBS} ${SNOPT_LIBS})

include_directories(
    ${EIGEN_INCLUDE_DIRS} 
    include
    ${catkin_INCLUDE_DIRS}
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Control.msg
   iLQG.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)


catkin_package(
   INCLUDE_DIRS
        include
        ${EIGEN_INCLUDE_DIRS}
   DEPENDS eigen lapack dl
   LIBRARIES lapack dl
   CATKIN_DEPENDS ct_core
)



if(BUILD_EXAMPLES)
    set(CT_OPTCON_EXAMPLE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/examples")
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/examples/exampleDir.h.in ${CMAKE_CURRENT_SOURCE_DIR}/examples/exampleDir.h)

    add_executable(ex_LQR examples/LQR.cpp)
    target_link_libraries(ex_LQR ${catkin_LIBRARIES} lapack dl)
endif()



get_filename_component(DMS_OSC_TEST_MAT_DIR "test/dms/oscillator/matfiles" ABSOLUTE)
configure_file(test/dms/oscillator/oscDMSTest_settings.h.in test/dms/oscillator/oscDMSTest_settings.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/test/dms/oscillator)

catkin_add_gtest(LqrTest test/lqr/LqrTest.cpp)
target_link_libraries(LqrTest ${catkin_LIBRARIES} lapack ${MATLAB_LIBS})

catkin_add_gtest(iLQGCTest test/ilqg/iLQGCTest.cpp)
target_link_libraries(iLQGCTest ${catkin_LIBRARIES} ${MATLAB_LIBS})

catkin_add_gtest(constraint_test test/constraint/ConstraintTest.cpp)
target_link_libraries(constraint_test ${catkin_LIBRARIES} dl)

catkin_add_gtest(CostFunctionTest test/costfunction/CostFunctionTest.cpp)
target_link_libraries(CostFunctionTest ${catkin_LIBRARIES} ${MATLAB_LIBS})

catkin_add_gtest(CfTracking test/costfunction/TrackingTest.cpp)
target_link_libraries(CfTracking ${catkin_LIBRARIES})

catkin_add_gtest(ADTest_timeVar test/costfunction/ADTest_timeDependent.cpp)
target_link_libraries(ADTest_timeVar ${catkin_LIBRARIES})

catkin_add_gtest(dms_test test/dms/oscillator/oscDMSTest.cpp)
target_link_libraries(dms_test ${catkin_LIBRARIES} ${MATLAB_LIBS} ${NLP_LIBS})

catkin_add_gtest(dms_test_all_var test/dms/oscillator/oscDMSTestAllVariants.cpp)
target_link_libraries(dms_test_all_var ${catkin_LIBRARIES} ${MATLAB_LIBS} ${NLP_LIBS})

add_executable(bareTest test/dms/oscillator/bareTest.cpp)
target_link_libraries(bareTest ${catkin_LIBRARIES} ${MATLAB_LIBS} ${NLP_LIBS} dl)

add_executable(matFilesGenerator test/dms/oscillator/matfiles/matFilesGenerator.cpp)
target_link_libraries(matFilesGenerator ${catkin_LIBRARIES} ${MATLAB_LIBS} ${NLP_LIBS})

add_executable(timeKeeperTest test/mpc/TimeKeeperTest.cpp)
target_link_libraries(timeKeeperTest ${catkin_LIBRARIES})

get_filename_component(MPC_TEST_MAT_DIR "test/mpc/matfiles" ABSOLUTE)
configure_file(test/mpc/mpcTestSettings.h.in test/mpc/mpcTestSettings.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/test/mpc)

catkin_add_gtest(iLQGMPCTest test/mpc/iLQGMPCTest.cpp)
target_link_libraries(iLQGMPCTest ${catkin_LIBRARIES} ${MATLAB_LIBS})



find_package(Doxygen)
if(DOXYGEN_FOUND)
    set(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/doc/ct_optcon.doxyfile)
    set(doxyfile ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile)

    configure_file(${doxyfile_in} ${doxyfile} @ONLY)

    add_custom_target(doc
        COMMAND ${CMAKE_COMMAND} -E echo_append "Building API Documentation..."
        COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/doc
        COMMAND ${CMAKE_COMMAND} -E echo_append "API Documentation built in ${CMAKE_CURRENT_SOURCE_DIR}/doc"
        VERBATIM)
endif()