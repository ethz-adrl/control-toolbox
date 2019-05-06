cmake_minimum_required (VERSION 3.3)

include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/compilerSettings.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/explicitTemplateHelpers.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/clang-cxx-dev-tools.cmake)


project(ct_optcon VERSION 3.0.0 LANGUAGES CXX)


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fopenmp -std=c++14 -Wall -Wno-unknown-pragmas -Wfatal-errors")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pthread -std=c++14 -Wall -Wfatal-errors")
SET(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

## find and include required dependencies
find_package(ct_core REQUIRED)

include_directories(
    include
    ${ct_core_INCLUDE_DIRS}
)

## add compiler definitions exported by ct_core
add_definitions(${ct_core_COMPILE_DEFINITIONS})


## find and include optional dependencies
option(MATLAB "Compile with matlab support" OFF)
option(MATLAB_FULL_LOG "Expose all variables to Matlab (very slow)" OFF)
option(DEBUG_PRINT "Print debug messages" OFF)


if(DEBUG_PRINT)
    message(STATUS "Debug Print ON")
    add_definitions (-DDEBUG_PRINT)
endif(DEBUG_PRINT)

if(MATLAB_FULL_LOG)
    message(WARNING "Compiling with full log to matlab. Execution will be very slow.")
    set(MATLAB ON)
    add_definitions (-DMATLAB_FULL_LOG)
endif(MATLAB_FULL_LOG)

if(MATLAB)
    message(STATUS "MATLAB support ON")
    find_package(matlab_cpp_interface REQUIRED)
    if(matlab_cpp_interface_FOUND)
        include_directories(${matlab_cpp_interface_INCLUDE_DIRS})
        add_definitions( -DMATLAB )
    endif()
endif(MATLAB)


## lapack libs
find_package(LAPACK QUIET)
set(LAPACK_LIBS "")
if(LAPACK_FOUND)
    message(STATUS "Found LAPACK library")
    add_definitions (-DCT_USE_LAPACK)
    list(APPEND LAPACK_LIBS lapack)
else(LAPACK_FOUND)
    message(STATUS "Could not find LAPACK library")
endif(LAPACK_FOUND)



## include BLASFEO AND HPIPM
set(HPIPM_LIBS "")
if(HPIPM)
    message(STATUS "Switching HPIPM ON")
    if(DEFINED ENV{BLASFEO_DIR})
        if(DEFINED ENV{HPIPM_DIR})
            message(WARNING "...Found HPIPM and BLASFEO environment variables")

            set(BLASFEO_INCLUDE_DIR $ENV{BLASFEO_DIR}/include)
            set(HPIPM_INCLUDE_DIR $ENV{HPIPM_DIR}/include)
            set(BLASFEO_DIR $ENV{BLASFEO_DIR})
            set(HPIPM_DIR $ENV{HPIPM_DIR})

            include_directories(${BLASFEO_INCLUDE_DIR})
            include_directories(${HPIPM_INCLUDE_DIR})
            link_directories(${BLASFEO_DIR}/lib)
            link_directories(${HPIPM_DIR}/lib)

            find_library(BLASFEO_LIBRARY blasfeo ${BLASFEO_DIR}/lib REQUIRED)
            find_library(HPIPM_LIBRARY hpipm ${HPIPM_DIR}/lib REQUIRED)

            list(APPEND HPIPM_LIBS hpipm blasfeo)

            add_definitions (-DHPIPM)

        else(DEFINED ENV{HPIPM_DIR})
            message(WARNING "Trying to build with HPIPM, but no ENV variable for HPIPM found.")
        endif(DEFINED ENV{HPIPM_DIR})
    else(DEFINED ENV{BLASFEO_DIR})
        message(WARNING "Trying to build with HPIPM, but no ENV variable for Blasfeo found.")
    endif(DEFINED ENV{BLASFEO_DIR})
endif(HPIPM)


## include IPOPT
find_package(ipopt QUIET)
if(DEFINED ENV{IPOPT_SOURCE_DIR} OR ipopt_FOUND)
    set(BUILD_WITH_IPOPT_SUPPORT ON)
    message(STATUS "Found IPOPT - building with IPOPT support")
    if(ipopt_FOUND)
      message(STATUS "Using LOCAL installation of IPOPT")
      set(IPOPT_BUILD_DIR ${CMAKE_BINARY_DIR}/build/include/coin)
    elseif(DEFINED ENV{IPOPT_SOURCE_DIR})
      message(STATUS "using GLOBAL installation of IPOPT")
      set(IPOPT_BUILD_DIR $ENV{IPOPT_SOURCE_DIR}/build)
    else()
      message(FATAL_ERROR "ERROR: Ipopt source directory environment variable not set! Set IPOPT_SOURCE_DIR environment variable!")
    endif(ipopt_FOUND)

    include_directories("${IPOPT_BUILD_DIR}/include/coin")
    add_definitions( -DBUILD_WITH_IPOPT_SUPPORT )
    link_directories(${IPOPT_BUILD_DIR}/lib)
    
    set(IPOPT_LIBS ipopt dl coinmumps coinhsl lapack blas gfortran m quadmath coinmetis)

endif(DEFINED ENV{IPOPT_SOURCE_DIR} OR ipopt_FOUND)


## include SNOPT
set(SNOPT_LIBS "")
set(SNOPT_TARGET "")
if(DEFINED ENV{SNOPT_SOURCE_DIR})
    set(BUILD_WITH_SNOPT_SUPPORT ON)
    message(STATUS "Found SNOPT - building with SNOPT support")
    include_directories( "$ENV{SNOPT_SOURCE_DIR}/include")

    add_definitions( -DBUILD_WITH_SNOPT_SUPPORT )
    find_library(SNOPT_LIBRARY1 snopt7_cpp   $ENV{SNOPT_SOURCE_DIR}/lib REQUIRED)
    find_library(SNOPT_LIBRARY2 snopt7       $ENV{SNOPT_SOURCE_DIR}/lib REQUIRED)
    set(SNOPT_LIBS ${SNOPT_LIBRARY1} ${SNOPT_LIBRARY2})
    
    add_library(ct_snopt_interface src/nlp/solver/SnoptSolver.cpp)
    target_link_libraries(ct_snopt_interface ${SNOPT_LIBRARY1} ${SNOPT_LIBRARY2})
    
    list(APPEND SNOPT_TARGET ct_snopt_interface)
    list(APPEND SNOPT_LIBS ct_snopt_interface ${SNOPT_LIBRARY1} ${SNOPT_LIBRARY2})

endif(DEFINED ENV{SNOPT_SOURCE_DIR})



## declare prespec libraries
set(PRESPEC_LIB_NAMES "")

## assemble list of libraries that contain prespecified templates
if(USE_PRESPEC)
    ct_configure_explicit_templates("${CMAKE_CURRENT_SOURCE_DIR}/../ct/config/explicit_templates.cfg"
        "${CMAKE_CURRENT_SOURCE_DIR}/prespec/"
        "ct_optcon"
    )
    message(STATUS "CT Optcon: Compiling the following explict template libraries: ${PRESPEC_LIB_NAMES}")
    ct_add_explicit_template_libs()
endif(USE_PRESPEC)

## configure clang-tidy
#set(INC_DIRS ${catkin_INCLUDE_DIRS}  "${CMAKE_CURRENT_SOURCE_DIR}/include" ${HPIPM_INCLUDE_DIR} ${BLASFEO_INCLUDE_DIR})
#ct_configure_clang_tidy(INC_DIRS)


## create ct_optcon library
add_library(ct_optcon SHARED
    src/ct_optcon.cpp
    )

target_link_libraries(ct_optcon INTERFACE
    ct_core
    ${matlab_cpp_interface_LIBRARIES}
    ${LAPACK_LIBS}
    ${HPIPM_LIBS}
    ${IPOPT_LIBS}
    ${SNOPT_LIBS}
    ${PRESPEC_LIB_NAMES}
    )


##################
# BUILD EXAMPLES #
##################

add_executable(ex_KalmanFiltering examples/KalmanFiltering.cpp)
target_link_libraries(ex_KalmanFiltering ct_optcon)
if(BUILD_EXAMPLES)
    set(CT_OPTCON_EXAMPLE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/examples")
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/examples/exampleDir.h.in ${CMAKE_CURRENT_SOURCE_DIR}/examples/exampleDir.h)

    add_executable(ex_ConstraintOutput examples/ConstraintExampleOutput.cpp)
    target_link_libraries(ex_ConstraintOutput ct_optcon)
    
    add_executable(ex_LQR examples/LQR.cpp)
    target_link_libraries(ex_LQR ct_optcon)
    
    add_executable(ex_NLOC examples/NLOC.cpp)
    target_link_libraries(ex_NLOC ct_optcon)
    
    add_executable(ex_NLOC_MPC examples/NLOC_MPC.cpp)
    target_link_libraries(ex_NLOC_MPC ct_optcon)
    
    add_executable(ex_DMS examples/DMS.cpp)
    target_link_libraries(ex_DMS ct_optcon)


    add_executable(ex_KalmanDisturbanceFiltering examples/KalmanDisturbanceFiltering.cpp)
    target_link_libraries(ex_KalmanDisturbanceFiltering ct_optcon )
    
    add_executable(ex_Nlp_2D examples/nlp/Nlp2D.cpp)
    target_link_libraries(ex_Nlp_2D ct_optcon)
    
    add_executable(ex_Nlp_3D examples/nlp/Nlp3D.cpp)
    target_link_libraries(ex_Nlp_3D ct_optcon)

    if(HPIPM)
        add_executable(ex_NLOC_boxConstrained examples/NLOC_boxConstrained.cpp)
        target_link_libraries(ex_NLOC_boxConstrained ct_optcon)
        
        add_executable(ex_NLOC_generalConstrained examples/NLOC_generalConstrained.cpp)
        target_link_libraries(ex_NLOC_generalConstrained ct_optcon)
        
        add_executable(switched_continuous_optcon_example examples/switched_systems_optcon/switched_continuous_optcon.cpp)
        target_link_libraries(switched_continuous_optcon_example ct_optcon)
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

## get all compile definitions registered so far and create variable
get_directory_property( CompileDefs DIRECTORY ${CMAKE_SOURCE_DIR} COMPILE_DEFINITIONS )
foreach( d ${CompileDefs} )
    set(ct_optcon_compile_definitions  ${ct_optcon_compile_definitions} -D${d} ) # recursively add compile definitions
endforeach()

### 1- local build ###

# Register the local build in case one doesn't use "make install"
export(PACKAGE ct_optcon)

# Create variable for the local build tree
get_property(ct_optcon_include_dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
message(STATUS "-------------------")
message(STATUS ${ct_optcon_include_dirs})
message(STATUS "-------------------")

# Configure config file for local build tree
configure_file(cmake/ct_optconConfig.cmake.in "${PROJECT_BINARY_DIR}/ct_optconConfig.cmake" @ONLY)

### 2- installation build ###

# Change the include location for the case of an install location
set(ct_optcon_include_dirs 
    ${CMAKE_INSTALL_PREFIX}/include 
    ${ct_core_INCLUDE_DIRS}
    ) #todo complete those

# We put the generated file for installation in a different repository (i.e., ./CMakeFiles/)
configure_file(cmake/ct_optconConfig.cmake.in "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/ct_optconConfig.cmake" @ONLY)
message(STATUS "-------------------")
message(STATUS ${ct_optcon_include_dirs})
message(STATUS "-------------------")

install(FILES "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/ct_optconConfig.cmake"
    DESTINATION share/ct/optcon/cmake COMPONENT dev)

install(
    TARGETS ct_optcon ${SNOPT_TARGET} ${PRESPEC_LIB_NAMES}
    EXPORT ct_optcon
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )

install (EXPORT ct_optcon DESTINATION ${CMAKE_INSTALL_PREFIX}/share/ct/optcon/cmake)

    
#################
# DOCUMENTATION #
#################
add_subdirectory(doc)
