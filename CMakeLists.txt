cmake_minimum_required (VERSION 3.3)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/compilerSettings.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/explicitTemplateHelpers.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/clang-cxx-dev-tools.cmake)

project(ControlToolbox VERSION 3.0.2 LANGUAGES CXX)
set (CMAKE_CXX_STANDARD 14)
set(MASTER_PROJECT_NAME ${PROJECT_NAME})

option(CMAKE_BUILD_TYPE "Build mode." Release) # original author requires this to default to Release
option(BUILD_EXAMPLES "Whether to build examples." false)
option(BUILD_DOCS "Generate Documentary." false)

#Make sure metapackage does not fail when building documentation
if(${BUILD_DOCS})
    add_custom_target(doc
            COMMAND ${CMAKE_COMMAND} -E echo_append "Building CT Documentation..."
            VERBATIM)
endif()

add_subdirectory("ct_optcon/")
add_subdirectory("ct_rbd/")
add_subdirectory("ct_models/")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -pthread -Wall -Wfatal-errors")
set(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -pthread -Wall -Wfatal-errors")
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

set(ct_core_LIBS "")

## find and include required dependencies
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS REQUIRED)

## find and include optional dependencies
find_package(llvm QUIET)
find_package(clang QUIET)
if(LLVM_FOUND AND CLANG_FOUND)
    message(STATUS "Found LLVM/CLANG version " ${LLVM_VERSION_MAJOR}.${LLVM_VERSION_MINOR})
    set (LLVM ON)
    list(APPEND ct_core_COMPILE_DEFINITIONS LLVM_VERSION_MAJOR=${LLVM_VERSION_MAJOR})
    list(APPEND ct_core_COMPILE_DEFINITIONS LLVM_VERSION_MINOR=${LLVM_VERSION_MINOR})
    list(APPEND ct_core_LIBS ${CLANG_LIBS})
    list(APPEND ct_core_LIBS ${LLVM_MODULE_LIBS})
    list(APPEND ct_core_target_include_dirs ${LLVM_INCLUDE_DIRS})
    list(APPEND ct_core_target_include_dirs ${CLANG_INCLUDE_DIRS})
else()
    message(STATUS "Could not find LLVM/CLANG, LLVM-JIT will not be available")
endif()
find_package(CppAD QUIET)
if(CPPAD_FOUND)
    set(CPPAD ON)
    list(APPEND ct_core_COMPILE_DEFINITIONS CPPAD)
    list(APPEND ct_core_target_include_dirs ${CPPAD_INCLUDE_DIRS})
else()
    message(STATUS "Could not find CppAD, auto-diff will not be available")
endif()
find_package(CppADCG QUIET)
if(CPPADCG_FOUND)
    set(CPPADCG ON)
    list(APPEND ct_core_COMPILE_DEFINITIONS CPPADCG)
    list(APPEND ct_core_target_include_dirs ${CPPADCG_INCLUDE_DIRS})
else()
    message(STATUS "Could not find CppADCodeGen, derivative code generation will not be available")
endif()

find_package(Qwt QUIET)
find_package(Qt4 QUIET)
if(QWT_FOUND AND Qt4_FOUND)
    message(STATUS "Qwt and QT found.")
    set(QWT_ENABLED ON)
    include(${QT_USE_FILE})
    list(APPEND ct_core_LIBS ${QWT_LIBRARY} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
else()
    message(STATUS "COMPILING WITHOUT QWT")
endif()

find_package(PythonLibs 2.7 QUIET)
if (PYTHONLIBS_FOUND)
   message(STATUS "Found python 2.7")
   list(APPEND ct_core_COMPILE_DEFINITIONS PLOTTING_ENABLED)
   message(STATUS "Python library path ... " ${PYTHON_LIBRARY})
   list(APPEND ct_core_LIBS ${PYTHON_LIBRARY})
else() 
   message(STATUS "Python not found, plotting will not be available") 
endif()


## configure files required for code-generation
set(CODEGEN_TEMPLATE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/templates")
set(CODEGEN_OUTPUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/generated")
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/include/ct/core/templateDir.h.in ${CMAKE_CURRENT_SOURCE_DIR}/include/ct/core/templateDir.h)


###################
# BUILD LIBRARIES #
###################

## define the directories to be included in all ct_core targets
list(APPEND ct_core_target_include_dirs ${EIGEN3_INCLUDE_DIR})
list(APPEND ct_core_target_include_dirs ${PYTHON_INCLUDE_DIRS})
list(APPEND ct_core_target_include_dirs ${QWT_INCLUDE_DIR})
list(APPEND ct_core_target_include_dirs $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
list(APPEND ct_core_target_include_dirs $<INSTALL_INTERFACE:include>)

if(${BUILD_EXAMPLES})
    list(APPEND ct_core_target_include_dirs $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/examples/include>)
endif()

## declare prespec libraries
set(PRESPEC_LIB_NAMES "")

## define list of libraries that contain prespecified templates
if(USE_PRESPEC)
    # extract the prespec parameters from user-input
    ct_configure_explicit_templates("${CMAKE_CURRENT_SOURCE_DIR}/../ct/config/explicit_templates.cfg" 
        "${CMAKE_CURRENT_SOURCE_DIR}/prespec/" 
        "ct_core"
    )
    message(STATUS "CT Core: Compiling the following explict template libraries: ${PRESPEC_LIB_NAMES}")
    # create libraries
    foreach(lib_name ${PRESPEC_LIB_NAMES})
        add_library(${lib_name} SHARED ${${lib_name}_SRCS})
        target_include_directories(${lib_name} PUBLIC ${ct_core_target_include_dirs})
    endforeach()
endif()


## create ct_core libraries
add_library(ControlToolbox_plot SHARED src/core/plot/plot.cpp)
target_include_directories(ControlToolbox_plot PUBLIC ${ct_core_target_include_dirs})
target_compile_definitions(ControlToolbox_plot PUBLIC ${ct_core_COMPILE_DEFINITIONS})
target_link_libraries(ControlToolbox_plot ${ct_core_LIBS})
add_library(ControlToolbox::plot ALIAS ControlToolbox_plot)

add_library(ControlToolbox INTERFACE)
target_include_directories(ControlToolbox INTERFACE ${ct_core_target_include_dirs})
target_compile_definitions(ControlToolbox INTERFACE ${ct_core_COMPILE_DEFINITIONS})
target_link_libraries(ControlToolbox INTERFACE
    ${ct_core_LIBS}
    ${PRESPEC_LIB_NAMES}
    ControlToolbox::plot
    pthread
    dl # required for gcc compatibility
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
## requires gtest to be installed, e.g. via 
## sudo apt install libgtest-dev
## cd /usr/src/gtest/
## sudo cmake -DBUILD_SHARED_LIBS=ON
## sudo make
## sudo cp *.so /usr/lib
if(BUILD_TESTS)
    find_package(GTest REQUIRED)
    enable_testing()
    add_subdirectory(test)
endif()


#################
# INSTALLATION  #
#################

# for correct libraries locations across platforms
include(GNUInstallDirs)

## copy the header files
install(DIRECTORY include/ct/core DESTINATION include/ct)
if(${BUILD_EXAMPLES})
    install(DIRECTORY examples/include/ct/core DESTINATION include/ct)
endif()

## copy the cmake files required for find_package()
install(FILES "cmake/ct_coreConfig.cmake" DESTINATION "share/ct_core/cmake")

## install library and targets
install(
    TARGETS ControlToolbox ControlToolbox_plot ${PRESPEC_LIB_NAMES}
    EXPORT ct_core_export
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

## create the ct_core.cmake file which holds target includes and dependencies
install (EXPORT ct_core_export DESTINATION share/ct_core/cmake)


#################
# DOCUMENTATION #
#################
if(${BUILD_DOCS})
    add_subdirectory(doc)
endif()
