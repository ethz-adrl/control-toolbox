cmake_minimum_required (VERSION 3.3)

include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/compilerSettings.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/explicitTemplateHelpers.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/clang-cxx-dev-tools.cmake)


project(ct_optcon VERSION 3.0.1 LANGUAGES CXX)


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fopenmp -std=c++14 -Wno-unknown-pragmas -Wall -Wfatal-errors")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pthread -std=c++14 -Wfatal-errors")
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)


## find and include required dependencies
find_package(ct_core REQUIRED)

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
    add_definitions( -DMATLAB )
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
set(HPIPM_INCLUDE_DIRS "")
set(HPIPM_LIBS "")
if(HPIPM)
    if(DEFINED ENV{BLASFEO_DIR})
        if(DEFINED ENV{HPIPM_DIR})
            message(WARNING "...Found HPIPM and BLASFEO environment variables")
            add_definitions (-DHPIPM)

            list(APPEND HPIPM_INCLUDE_DIRS $ENV{BLASFEO_DIR}/include)
            list(APPEND HPIPM_INCLUDE_DIRS $ENV{HPIPM_DIR}/include)
    
            link_directories($ENV{BLASFEO_DIR}/lib)
            link_directories($ENV{HPIPM_DIR}/lib)
            list(APPEND HPIPM_LIBS hpipm blasfeo)

        else(DEFINED ENV{HPIPM_DIR})
            message(WARNING "Trying to build with HPIPM, but no ENV variable for HPIPM found.")
        endif(DEFINED ENV{HPIPM_DIR})
    else(DEFINED ENV{BLASFEO_DIR})
        message(WARNING "Trying to build with HPIPM, but no ENV variable for Blasfeo found.")
    endif(DEFINED ENV{BLASFEO_DIR})
endif(HPIPM)


## include IPOPT (after installation via sudo apt-get install coinor-libipopt-dev)
find_package(IPOPT QUIET)
if(IPOPT_FOUND)
    set(BUILD_WITH_IPOPT_SUPPORT ON)
    add_definitions(-DBUILD_WITH_IPOPT_SUPPORT)
    add_definitions(${IPOPT_DEFINITIONS})
    message(STATUS "Found IPOPT - building with IPOPT support") 
    set(IPOPT_LIBS ipopt) # dl mumps coinhsl lapack blas gfortran m quadmath coinmetis)
endif()


## include SNOPT -- todo SNOPT not tested in version 3.0.1, temporarily deactivated
#set(SNOPT_TARGET "")
#if(DEFINED ENV{SNOPT_SOURCE_DIR})
#    set(BUILD_WITH_SNOPT_SUPPORT ON)
#    message(STATUS "Found SNOPT - building with SNOPT support")
#    include_directories( "$ENV{SNOPT_SOURCE_DIR}/include")
#
#    add_definitions( -DBUILD_WITH_SNOPT_SUPPORT )
#    find_library(SNOPT_LIBRARY1 snopt7_cpp   $ENV{SNOPT_SOURCE_DIR}/lib REQUIRED)
#    find_library(SNOPT_LIBRARY2 snopt7       $ENV{SNOPT_SOURCE_DIR}/lib REQUIRED)
#    set(SNOPT_LIBS ${SNOPT_LIBRARY1} ${SNOPT_LIBRARY2})
#    
#    add_library(ct_snopt_interface src/nlp/solver/SnoptSolver.cpp)
#    target_link_libraries(ct_snopt_interface ${SNOPT_LIBRARY1} ${SNOPT_LIBRARY2})
#    
#    list(APPEND SNOPT_TARGET ct_snopt_interface)
#    list(APPEND SNOPT_LIBS ct_snopt_interface ${SNOPT_LIBRARY1} ${SNOPT_LIBRARY2})
#
#endif(DEFINED ENV{SNOPT_SOURCE_DIR})


###################
# BUILD LIBRARIES #
###################

## define the directories to be included in all ct_optcon targets
set(ct_optcon_target_include_dirs
    ${ct_core_INCLUDE_DIRS}
    ${matlab_cpp_interface_INCLUDE_DIRS}
    ${HPIPM_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

## declare prespec libraries
set(PRESPEC_LIB_NAMES "")

## assemble list of libraries that contain prespecified templates
if(USE_PRESPEC)
    ct_configure_explicit_templates("${CMAKE_CURRENT_SOURCE_DIR}/../ct/config/explicit_templates.cfg"
        "${CMAKE_CURRENT_SOURCE_DIR}/prespec/"
        "ct_optcon"
    )
    message(STATUS "CT Optcon: Compiling the following explict template libraries: ${PRESPEC_LIB_NAMES}")
    # create libraries
    foreach(lib_name ${PRESPEC_LIB_NAMES})
        add_library(${lib_name} SHARED ${${lib_name}_SRCS})
        target_include_directories(${lib_name} PUBLIC ${ct_optcon_target_include_dirs})
        target_link_libraries(${lib_name} 
            ct_core
            ${matlab_cpp_interface_LIBRARIES}
            ${LAPACK_LIBS}
            ${HPIPM_LIBS}
            ${IPOPT_LIBS}
        )
    endforeach()
endif(USE_PRESPEC)


## create ct_optcon library
add_library(ct_optcon SHARED
    src/ct_optcon.cpp
    )
target_include_directories(ct_optcon PUBLIC ${ct_optcon_target_include_dirs})
    target_link_libraries(ct_optcon
    ct_core
    ${matlab_cpp_interface_LIBRARIES}
    ${LAPACK_LIBS}
    ${HPIPM_LIBS}
    ${IPOPT_LIBS}
    #${SNOPT_LIBS}
    ${PRESPEC_LIB_NAMES}
    )


##################
# BUILD EXAMPLES #
##################
if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()


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

## generate files required for installation
configure_file(cmake/ct_optconConfig.cmake.in "${PROJECT_BINARY_DIR}/ct_optconConfig.cmake" @ONLY)

## copy the header files
install(DIRECTORY include/ct/optcon DESTINATION include/ct)

## copy the cmake files required for find_package()
install(FILES "${PROJECT_BINARY_DIR}/ct_optconConfig.cmake" DESTINATION "share/ct_optcon/cmake")

## install library and targets
install(
    TARGETS ct_optcon ${PRESPEC_LIB_NAMES}
    EXPORT ct_optcon_export
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )

## create the ct_core.cmake file which holds target includes and dependencies
install (EXPORT ct_optcon_export DESTINATION share/ct_optcon/cmake)

    
#################
# DOCUMENTATION #
#################
add_subdirectory(doc)
