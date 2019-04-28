cmake_minimum_required (VERSION 3.3)

include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/compilerSettings.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/explicitTemplateHelpers.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/clang-cxx-dev-tools.cmake)


project(ct_core VERSION 3.0.0 LANGUAGES CXX)


set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -pthread -std=c++14 -Wall -Wfatal-errors")
SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -pthread -std=c++14 -Wall -Wfatal-errors")
SET(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)


## find and include required dependencies
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS REQUIRED)

include_directories(
    ${BOOST_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    include
    include/external
    examples
)


## find and include optional dependencies
find_package(Qwt QUIET)
find_package(Qt4 QUIET)
if(QWT_FOUND AND Qt4_FOUND)
    set(QWT_ENABLED true)
    add_definitions( -DQWT_ENABLED )
    include(${QT_USE_FILE})
    include_directories(${QWT_INCLUDE_DIR}})
    message(STATUS "COMPILING WITH QWT")
    set(ct_core_extra_libs ${QWT_LIBRARY} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
else()
    message(STATUS "COMPILING WITHOUT QWT")
    set(ct_core_extra_libs "")
endif()

## find and include optional dependencies
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


## configure clang-tidy
set(INC_DIRS ${BOOST_INCLUDE_DIRS} "${CMAKE_CURRENT_SOURCE_DIR}/include" "${CMAKE_CURRENT_SOURCE_DIR}/include/external" ${EIGEN3_INCLUDE_DIR})
ct_configure_clang_tidy(INC_DIRS)

## configure files required for code-generation
set(CODEGEN_TEMPLATE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/templates")
set(CODEGEN_OUTPUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/generated")
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/include/ct/core/templateDir.h.in ${CMAKE_CURRENT_SOURCE_DIR}/include/ct/core/templateDir.h)


## compile default library elements
add_library(ct_core
   src/core/plot/plot.cpp
   src/core/internal/autodiff/SparsityPattern.cpp
)
target_link_libraries(ct_core dl ${PYTHON_LIBRARY})


## declare prespec libraries
set(PRESPEC_LIB_NAMES "")

## define list of libraries that contain prespecified templates
if(USE_PRESPEC)
    ct_configure_explicit_templates("${CMAKE_CURRENT_SOURCE_DIR}/../ct/config/explicit_templates.cfg" "${CMAKE_CURRENT_SOURCE_DIR}/prespec/" "ct_core")
    message(WARNING "CT Core: Compiling the following explict template libraries: ${PRESPEC_LIB_NAMES}")
    ct_add_explicit_template_libs()
endif(USE_PRESPEC)



##################
# BUILD EXAMPLES #
##################
if(BUILD_EXAMPLES)
    add_executable(ex_DampedOscillatorCustomController examples/DampedOscillatorCustomController.cpp)
    add_executable(ex_DampedOscillator examples/DampedOscillator.cpp)
    add_executable(ex_MasspointIntegration examples/MasspointIntegration.cpp)
endif()

add_executable(plotTest test/plot/plotTest.cpp)
target_link_libraries(plotTest ct_core)

if(QWT_ENABLED)
    add_executable(plotQwtTest test/plot/plotQwtTest.cpp)
    target_link_libraries(plotQwtTest ct_core ${ct_core_extra_libs} pthread)
endif()


###########
# TESTING #
###########

if(BUILD_TESTS)
    find_package(GTest QUIET)
    enable_testing()
    add_subdirectory(test)
endif()


#################
# INSTALLATION  #
#################

# for correct libraries locations across platforms
include(GNUInstallDirs) 

#copy the header files
install(DIRECTORY include/ct/core DESTINATION include/ct)

### 1- local build ###

# Register the local build in case one doesn't use "make install"
export(PACKAGE ct_core)

# Create variable for the local build tree
get_property(ct_core_include_dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)

# Configure config file for local build tree
configure_file(cmake/ct_coreConfig.cmake.in "${PROJECT_BINARY_DIR}/ct_coreConfig.cmake" @ONLY)

### 2- installation build ###

# Change the include location for the case of an install location
set(ct_core_include_dirs ${CMAKE_INSTALL_PREFIX}/include ${EIGEN_INCLUDE_DIR})

# We put the generated file for installation in a different repository (i.e., ./CMakeFiles/)
configure_file(cmake/ct_coreConfig.cmake.in "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/ct_coreConfig.cmake" @ONLY)

install(FILES "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/ct_coreConfig.cmake"
    DESTINATION share/ct/core/cmake COMPONENT dev)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}-targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )



#################
# DOCUMENTATION #
#################
find_package(Doxygen)
if(DOXYGEN_FOUND)

    set(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/doc/ct_core.doxyfile)
    set(doxyfile ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile)

    configure_file(${doxyfile_in} ${doxyfile} @ONLY)

    add_custom_target(doc
        COMMAND ${CMAKE_COMMAND} -E echo_append "Building API Documentation..."
        COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/doc
        COMMAND ${CMAKE_COMMAND} -E echo_append "API Documentation built in ${CMAKE_CURRENT_SOURCE_DIR}/doc"
        VERBATIM)
endif()