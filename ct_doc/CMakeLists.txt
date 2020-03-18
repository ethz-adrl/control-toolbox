cmake_minimum_required (VERSION 3.3)

project (ct_doc VERSION 3.0.2 )

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)


## find and include required dependencies
find_package(ct_core REQUIRED)
find_package(ct_optcon REQUIRED)
find_package(ct_rbd REQUIRED)
find_package(ct_models REQUIRED)

find_package(Doxygen REQUIRED)

if(DOXYGEN_FOUND)

    set(doxyfile ${CMAKE_CURRENT_SOURCE_DIR}/doc/ct_doc.doxyfile)

    add_custom_target(doc
        COMMAND ${CMAKE_COMMAND} -E echo_append "Building API Documentation..."
        COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/doc
        COMMAND ${CMAKE_COMMAND} -E echo_append "API Documentation built in ${CMAKE_CURRENT_SOURCE_DIR}/doc ."
        COMMAND sensible-browser ${CMAKE_CURRENT_SOURCE_DIR}/doc/html/index.html
        VERBATIM)

else(DOXYGEN_FOUND)
    message("Doxygen need to be installed to generate the doxygen documentation")
endif(DOXYGEN_FOUND)

# a temporary placeholder file to avoid cmake error
install(FILES doc/html/ct_doc_placeholder.txt DESTINATION doc/ct_doc/doc/html)

# add uninstall target
if(NOT TARGET uninstall)
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/../ct/cmake/cmake_uninstall.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/../ct/cmake/cmake_uninstall.cmake"
        IMMEDIATE @ONLY)

    add_custom_target(uninstall
        COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/../ct/cmake/cmake_uninstall.cmake)
endif()

add_custom_target(clang-format
        COMMAND ${CMAKE_COMMAND} -E echo_append ""
        VERBATIM)
