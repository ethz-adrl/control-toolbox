cmake_minimum_required (VERSION 3.3)

include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/compilerSettings.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/explicitTemplateHelpers.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/clang-cxx-dev-tools.cmake)


project(ct_optcon VERSION 3.0.0 LANGUAGES CXX)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/externalSolvers.cmake)


set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fopenmp -std=c++14 -Wall -Wno-unknown-pragmas -Wfatal-errors")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pthread -std=c++14 -Wall -Wfatal-errors")
SET(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

## find and include required dependencies
find_package(ct_core REQUIRED)

include_directories(
    include
    ${ct_core_INCLUDE_DIRS}
)
# add compiler definitions exported by ct_core
add_definitions(${ct_core_COMPILE_DEFINITIONS}) #todo can we properly export those with an install command?


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



## assemble list of libraries that contain prespecified templates
if(USE_PRESPEC)
    ct_configure_explicit_templates("${CMAKE_CURRENT_SOURCE_DIR}/../ct/config/explicit_templates.cfg" "${CMAKE_CURRENT_SOURCE_DIR}/prespec/" "ct_optcon")
    message(WARNING "CT Optcon: Compiling the following explict template libraries: ${PRESPEC_LIB_NAMES}")
endif(USE_PRESPEC)

## configure clang-tidy
set(INC_DIRS ${catkin_INCLUDE_DIRS}  "${CMAKE_CURRENT_SOURCE_DIR}/include" ${HPIPM_INCLUDE_DIR} ${BLASFEO_INCLUDE_DIR})
ct_configure_clang_tidy(INC_DIRS)


## build SNOPT library
if(BUILD_WITH_SNOPT_SUPPORT)
    add_library(ct_snopt_interface src/nlp/solver/SnoptSolver.cpp)
    target_link_libraries(ct_snopt_interface  ${SNOPT_LIBS})
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


##################
# BUILD EXAMPLES #
##################

if(BUILD_EXAMPLES)
    set(CT_OPTCON_EXAMPLE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/examples")
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/examples/exampleDir.h.in ${CMAKE_CURRENT_SOURCE_DIR}/examples/exampleDir.h)

    add_executable(ex_ConstraintOutput examples/ConstraintExampleOutput.cpp)
    target_link_libraries(ex_ConstraintOutput )
    
    add_executable(ex_LQR examples/LQR.cpp)
    target_link_libraries(ex_LQR ct_core dl ${LAPACK_LIBS})
    
    add_executable(ex_NLOC examples/NLOC.cpp)
    target_link_libraries(ex_NLOC ct_core  ${HPIPM_LIBS} ${MATLAB_LIBS})
    
    add_executable(ex_NLOC_MPC examples/NLOC_MPC.cpp)
    target_link_libraries(ex_NLOC_MPC  ${HPIPM_LIBS} ${MATLAB_LIBS})
    
    add_executable(ex_DMS examples/DMS.cpp)
    target_link_libraries(ex_DMS ct_core  ${NLP_LIBS})

    add_executable(ex_KalmanFiltering examples/KalmanFiltering.cpp)
    target_link_libraries(ex_KalmanFiltering ct_core)

    add_executable(ex_KalmanDisturbanceFiltering examples/KalmanDisturbanceFiltering.cpp)
    target_link_libraries(ex_KalmanDisturbanceFiltering ct_core )
    
    add_executable(ex_Nlp_2D examples/nlp/Nlp2D.cpp)
    target_link_libraries(ex_Nlp_2D  ${NLP_LIBS})
    
    add_executable(ex_Nlp_3D examples/nlp/Nlp3D.cpp)
    target_link_libraries(ex_Nlp_3D  ${NLP_LIBS})

    if(HPIPM)
        add_executable(ex_NLOC_boxConstrained examples/NLOC_boxConstrained.cpp)
        target_link_libraries(ex_NLOC_boxConstrained  ${MATLAB_LIBS} ${HPIPM_LIBS})
        
        add_executable(ex_NLOC_generalConstrained examples/NLOC_generalConstrained.cpp)
        target_link_libraries(ex_NLOC_generalConstrained  ${MATLAB_LIBS} ${HPIPM_LIBS})
        
        add_executable(switched_continuous_optcon_example examples/switched_systems_optcon/switched_continuous_optcon.cpp)
        target_link_libraries(switched_continuous_optcon_example  ${MATLAB_LIBS} ${HPIPM_LIBS})
    endif(HPIPM)
 endif() # build examples


###########
# TESTING #
###########

if(BUILD_TESTS)
    #find_package(GTest QUIET)
    enable_testing()
    add_subdirectory(test)
endif()


#################
# INSTALLATION  #
#################

# for correct libraries locations across platforms
include(GNUInstallDirs) 

install(DIRECTORY include/ct/optcon DESTINATION include/ct)

### 1- local build ###

# Register the local build in case one doesn't use "make install"
export(PACKAGE ct_optcon)

# Create variable for the local build tree
get_property(ct_optcon_include_dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)

# Configure config file for local build tree
configure_file(cmake/ct_optconConfig.cmake.in "${PROJECT_BINARY_DIR}/ct_optconConfig.cmake" @ONLY)

### 2- installation build ###

# Change the include location for the case of an install location
set(ct_optcon_include_dirs 
    ${CMAKE_INSTALL_PREFIX}/include 
    ${ct_core_INCLUDE_DIRS}
    )

# We put the generated file for installation in a different repository (i.e., ./CMakeFiles/)
configure_file(cmake/ct_optconConfig.cmake.in "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/ct_optconConfig.cmake" @ONLY)

install(FILES "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/ct_optconConfig.cmake"
    DESTINATION share/ct/optcon/cmake COMPONENT dev)


# Install a Catkin 'package.xml' file.
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

 
#################
# DOCUMENTATION #
#################
add_subdirectory(doc)
