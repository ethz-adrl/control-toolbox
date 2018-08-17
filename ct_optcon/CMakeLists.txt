cmake_minimum_required (VERSION 2.6)

include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/compilerSettings.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/explicitTemplateHelpers.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/clang-cxx-dev-tools.cmake)

project (ct_optcon)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/externalSolvers.cmake)

find_package(catkin REQUIRED
    ct_core
)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -pthread -fopenmp -std=c++11 -Wall")
SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -pthread -std=c++11 -Wall")
SET(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

option(MATLAB "Compile with matlab support" OFF)
option(MATLAB_FULL_LOG "Expose all variables to Matlab (very slow)" OFF)
option(DEBUG_PRINT "Print debug messages" OFF)


if(DEBUG_PRINT)
    message(STATUS "WARNING: Debug Print ON")
    add_definitions ( -DDEBUG_PRINT )
endif(DEBUG_PRINT)

if(MATLAB_FULL_LOG)
    message(WARNING "WARNING: Compiling with full log to matlab. Execution will be very slow.")
    set(MATLAB ON)
    add_definitions ( -DMATLAB_FULL_LOG )
endif(MATLAB_FULL_LOG)

if(MATLAB)
    message(STATUS "MATLAB support ON")
    find_package(matlab_cpp_interface REQUIRED)
    if(matlab_cpp_interface_FOUND)
        include_directories(${matlab_cpp_interface_INCLUDE_DIRS})
        set(MATLAB_LIB_DIR ${matlab_cpp_interface_LIBRARIES})
        add_definitions( -DMATLAB )
    endif()
endif(MATLAB)
set(MATLAB_LIBS ${MATLAB_LIB_DIR})

#todo export this properly from ct_core
find_package(PythonLibs QUIET)
if (PYTHONLIBS_FOUND)
   message(STATUS "Python found")
   include_directories(${PYTHON_INCLUDE_DIRS})
   set(PLOTTING_ENABLED true)
   add_definitions(-DPLOTTING_ENABLED)
else()
   message(STATUS "Python not found")
   set(PYTHON_LIBRARY "")
endif()


## lapack libs
set(LAPACK_LIBS "")
find_package(LAPACK QUIET)
if(LAPACK_FOUND)
    message(STATUS "Found LAPACK library")
    set(LAPACK_LIBS lapack)
    add_definitions (-DCT_USE_LAPACK)
    set(USE_LAPACK ON)
else(LAPACK_FOUND)
    message(WARNING "Could not find LAPACK library")
endif(LAPACK_FOUND)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

## assemble list of libraries that contain prespecified templates
if(USE_PRESPEC)
    ct_configure_explicit_templates("${CMAKE_CURRENT_SOURCE_DIR}/../ct/config/explicit_templates.cfg" "${CMAKE_CURRENT_SOURCE_DIR}/prespec/" "ct_optcon")
    message(WARNING "CT Optcon: Compiling the following explict template libraries: ${PRESPEC_LIB_NAMES}")
endif(USE_PRESPEC)

#used later for clang-tidy
set(INC_DIRS ${catkin_INCLUDE_DIRS}  "${CMAKE_CURRENT_SOURCE_DIR}/include" ${HPIPM_INCLUDE_DIR} ${BLASFEO_INCLUDE_DIR})

catkin_package(
   INCLUDE_DIRS
        include
        ${HPIPM_INCLUDE_DIR}
        ${BLASFEO_INCLUDE_DIR}
   LIBRARIES
        ${LAPACK_LIBS}
        ${CT_SNOPT_LIBS}
        ${HPIPM_LIBRARY}
        ${BLASFEO_LIBRARY}
        ${PRESPEC_LIB_NAMES}
   CATKIN_DEPENDS
        ct_core
)

## build SNOPT library
if(BUILD_WITH_SNOPT_SUPPORT)
    add_library(ct_snopt_interface src/nlp/solver/SnoptSolver.cpp)
    target_link_libraries(ct_snopt_interface ${catkin_LIBRARIES} ${SNOPT_LIBS})
endif()


# add libraries for explicit template
if(USE_PRESPEC)
    ct_add_explicit_template_libs()
endif(USE_PRESPEC)

# link against addigional external libraries, such as lapack, if applicable
if(USE_LAPACK)
    ct_link_external_library(${LAPACK_LIBS})
endif(USE_LAPACK)
if(HPIPM)
   ct_link_external_library(${BLASFEO_LIBRARY})
   ct_link_external_library(${HPIPM_LIBRARY})
endif(HPIPM)


if(BUILD_EXAMPLES)
    message(STATUS "Building with examples.")

    set(CT_OPTCON_EXAMPLE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/examples")
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/examples/exampleDir.h.in ${CMAKE_CURRENT_SOURCE_DIR}/examples/exampleDir.h)

    add_executable(ex_ConstraintOutput examples/ConstraintExampleOutput.cpp)
    target_link_libraries(ex_ConstraintOutput ${catkin_LIBRARIES})

    add_executable(ex_LQR examples/LQR.cpp)
    target_link_libraries(ex_LQR ${catkin_LIBRARIES} ${LAPACK_LIBS})

    add_executable(ex_NLOC examples/NLOC.cpp)
    target_link_libraries(ex_NLOC ${catkin_LIBRARIES} ${HPIPM_LIBS} ${MATLAB_LIBS})

    add_executable(ex_NLOC_MPC examples/NLOC_MPC.cpp)
    target_link_libraries(ex_NLOC_MPC ${catkin_LIBRARIES} ${HPIPM_LIBS} ${MATLAB_LIBS})

    add_executable(ex_DMS examples/DMS.cpp)
    target_link_libraries(ex_DMS ${catkin_LIBRARIES} ${NLP_LIBS})

    add_executable(ex_KalmanFiltering examples/KalmanFiltering.cpp)
    target_link_libraries(ex_KalmanFiltering ${catkin_LIBRARIES})

    add_executable(ex_KalmanDisturbanceFiltering examples/KalmanDisturbanceFiltering.cpp)
    target_link_libraries(ex_KalmanDisturbanceFiltering ${catkin_LIBRARIES})
endif()


get_filename_component(DMS_OSC_TEST_MAT_DIR "test/dms/oscillator/matfiles" ABSOLUTE)
configure_file(test/dms/oscillator/oscDMSTest_settings.h.in test/dms/oscillator/oscDMSTest_settings.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/test/dms/oscillator)

catkin_add_gtest(LqrTest test/lqr/LqrTest.cpp)
target_link_libraries(LqrTest ${catkin_LIBRARIES} ${LAPACK_LIBS} ${MATLAB_LIBS})

catkin_add_gtest(iLQRTest test/nloc/iLQRTest.cpp)
target_link_libraries(iLQRTest ${catkin_LIBRARIES} ${MATLAB_LIBS} ${HPIPM_LIBS})

if(HPIPM)
    catkin_add_gtest(LQOCSolverTest test/solver/linear/LQOCSolverTest.cpp)
    target_link_libraries(LQOCSolverTest ${catkin_LIBRARIES} ${MATLAB_LIBS} ${HPIPM_LIBS})

    catkin_add_gtest(ConstrainedLQOCSolverTest test/solver/linear/ConstrainedLQOCSolverTest.cpp)
    target_link_libraries(ConstrainedLQOCSolverTest ${catkin_LIBRARIES} ${MATLAB_LIBS} ${HPIPM_LIBS})

    add_executable(LQOCSolverTiming test/solver/linear/LQOCSolverTiming.cpp)
    target_link_libraries(LQOCSolverTiming ${catkin_LIBRARIES} ${MATLAB_LIBS} ${HPIPM_LIBS})

    if(BUILD_EXAMPLES)
        add_executable(ex_NLOC_boxConstrained examples/NLOC_boxConstrained.cpp)
        target_link_libraries(ex_NLOC_boxConstrained ${catkin_LIBRARIES} ${MATLAB_LIBS} ${HPIPM_LIBS})

        add_executable(ex_NLOC_generalConstrained examples/NLOC_generalConstrained.cpp)
        target_link_libraries(ex_NLOC_generalConstrained ${catkin_LIBRARIES} ${MATLAB_LIBS} ${HPIPM_LIBS})

        add_executable(switched_continuous_optcon_example examples/switched_systems_optcon/switched_continuous_optcon.cpp)
        target_link_libraries(switched_continuous_optcon_example ${catkin_LIBRARIES} ${MATLAB_LIBS} ${HPIPM_LIBS})
    endif(BUILD_EXAMPLES)
endif(HPIPM)


get_filename_component(NLOC_TEST_DIR "test/nloc" ABSOLUTE)
configure_file(test/nloc/nloc_test_dir.h.in test/nloc/nloc_test_dir.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/test/nloc)

catkin_add_gtest(LinearSystemTest test/nloc/LinearSystemTest.cpp)
target_link_libraries(LinearSystemTest ${catkin_LIBRARIES} ${HPIPM_LIBS} ${MATLAB_LIBS})

catkin_add_gtest(NonlinearSystemTest test/nloc/NonlinearSystemTest.cpp)
target_link_libraries(NonlinearSystemTest ${catkin_LIBRARIES} ${HPIPM_LIBS} ${MATLAB_LIBS})

add_executable(SymplecticTest test/nloc/SymplecticTest.cpp)
target_link_libraries(SymplecticTest ${catkin_LIBRARIES} ${HPIPM_LIBS} ${MATLAB_LIBS})

catkin_add_gtest(constraint_comparison test/constraint/ConstraintComparison.cpp)
target_link_libraries(constraint_comparison ${catkin_LIBRARIES})

catkin_add_gtest(constraint_test test/constraint/ConstraintTest.cpp)
target_link_libraries(constraint_test ${catkin_LIBRARIES} )

catkin_add_gtest(SparseBoxConstraintTest test/constraint/SparseBoxConstraintTest.cpp)
target_link_libraries(SparseBoxConstraintTest ${catkin_LIBRARIES})

get_filename_component(COSTFUNCTION_TEST_DIR "test/costfunction" ABSOLUTE)
configure_file(test/costfunction/costfunction_test_dir.h.in test/costfunction/costfunction_test_dir.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/test/costfunction)
catkin_add_gtest(CostFunctionTests test/costfunction/CostFunctionTests.cpp)
target_link_libraries(CostFunctionTests ${catkin_LIBRARIES} ${MATLAB_LIBS})
catkin_add_gtest(LoadFromFileTest test/costfunction/LoadFromFileTest.cpp)
target_link_libraries(LoadFromFileTest ${catkin_LIBRARIES} ${MATLAB_LIBS})

catkin_add_gtest(dms_test test/dms/oscillator/oscDMSTest.cpp)
target_link_libraries(dms_test ${catkin_LIBRARIES} ${MATLAB_LIBS} ${NLP_LIBS})

catkin_add_gtest(dms_test_all_var test/dms/oscillator/oscDMSTestAllVariants.cpp)
target_link_libraries(dms_test_all_var ${catkin_LIBRARIES} ${MATLAB_LIBS} ${NLP_LIBS})

add_executable(matFilesGenerator test/dms/oscillator/matfiles/matFilesGenerator.cpp)
target_link_libraries(matFilesGenerator ${catkin_LIBRARIES} ${MATLAB_LIBS} ${NLP_LIBS})

get_filename_component(MPC_TEST_MAT_DIR "test/mpc/matfiles" ABSOLUTE)
configure_file(test/mpc/mpcTestSettings.h.in test/mpc/mpcTestSettings.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/test/mpc)

catkin_add_gtest(NLOC_MPCTest test/mpc/NLOC_MPCTest.cpp)
target_link_libraries(NLOC_MPCTest ${catkin_LIBRARIES} ${MATLAB_LIBS} ${HPIPM_LIBS})

catkin_add_gtest(system_interface_test test/system_interface/SystemInterfaceTest.cpp)
target_link_libraries(system_interface_test ${catkin_LIBRARIES})


## additional prespec tests
if(USE_PRESPEC)
    catkin_add_gtest(LqrTestPrespec test/lqr/LqrTestPrespec.cpp)
    target_link_libraries(LqrTestPrespec ${catkin_LIBRARIES} ${LAPACK_LIBS} ${MATLAB_LIBS} ${PRESPEC_LIB_NAMES})

    if(HPIPM)
        catkin_add_gtest(LQOCSolverTestPrespec test/solver/linear/LQOCSolverTestPrespec.cpp)
        target_link_libraries(LQOCSolverTestPrespec ${catkin_LIBRARIES} ${MATLAB_LIBS} ${HPIPM_LIBS} ${PRESPEC_LIB_NAMES})
    endif(HPIPM)

    catkin_add_gtest(LinearSystemTestPrespec test/nloc/LinearSystemTestPrespec.cpp)
    target_link_libraries(LinearSystemTestPrespec ${catkin_LIBRARIES} ${HPIPM_LIBS} ${MATLAB_LIBS} ${PRESPEC_LIB_NAMES})

    #add_executable(SymplecticTestPrespec test/nloc/SymplecticTestPrespec.cpp)
    #target_link_libraries(SymplecticTestPrespec ${catkin_LIBRARIES} ${HPIPM_LIBS} ${MATLAB_LIBS} ${PRESPEC_LIB_NAMES})

    catkin_add_gtest(constraint_test_prespec test/constraint/ConstraintTestPrespec.cpp)
    target_link_libraries(constraint_test_prespec ${catkin_LIBRARIES} ${PRESPEC_LIB_NAMES})

    catkin_add_gtest(NLOC_MPCTest_prespec test/mpc/NLOC_MPCTest_prespec.cpp)
    target_link_libraries(NLOC_MPCTest_prespec ${catkin_LIBRARIES} ${MATLAB_LIBS} ${HPIPM_LIBS} ${PRESPEC_LIB_NAMES})

    catkin_add_gtest(system_interface_test_prespec test/system_interface/SystemInterfaceTest-Prespec.cpp)
    target_link_libraries(system_interface_test_prespec ${catkin_LIBRARIES} ${PRESPEC_LIB_NAMES})
endif(USE_PRESPEC)

## build doxygen documentation
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


ct_configure_clang_tidy(INC_DIRS)
