cmake_minimum_required (VERSION 2.6) 

project (ct_doc) 

find_package(catkin REQUIRED)

catkin_package(
    CATKIN_DEPENDS ct_core ct_optcon ct_rbd ct_models
)

find_package(Doxygen)
if(DOXYGEN_FOUND)
    
    set(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/doc/ct_doc.doxyfile)
    set(doxyfile ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile)

    configure_file(${doxyfile_in} ${doxyfile} @ONLY)

    add_custom_target(doc
        COMMAND ${CMAKE_COMMAND} -E echo_append "Building API Documentation..."
        COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/doc
        COMMAND ${CMAKE_COMMAND} -E echo_append "API Documentation built in ${CMAKE_CURRENT_SOURCE_DIR}/doc"
#       COMMAND sensible-browser ${CMAKE_CURRENT_SOURCE_DIR}/doc/html/index.html
        VERBATIM)
endif()

        
add_custom_target(clang-format
        COMMAND ${CMAKE_COMMAND} -E echo_append ""
        VERBATIM)

add_custom_target(clang-tidy
        COMMAND ${CMAKE_COMMAND} -E echo_append ""
        VERBATIM)    


    