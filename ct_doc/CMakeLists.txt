cmake_minimum_required (VERSION 3.3)

project (ct_doc VERSION 3.0.0 ) 

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")


## find and include required dependencies
find_package(ct_core REQUIRED)
find_package(ct_optcon REQUIRED)
find_package(ct_rbd REQUIRED)
find_package(ct_models REQUIRED)

find_package(Doxygen)
if(DOXYGEN_FOUND)
    
    set(doxyfile ${CMAKE_CURRENT_SOURCE_DIR}/doc/ct_doc.doxyfile)

    add_custom_target(doc
        COMMAND ${CMAKE_COMMAND} -E echo_append "Building API Documentation..."
        COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/doc
        COMMAND ${CMAKE_COMMAND} -E echo_append "API Documentation built in ${CMAKE_CURRENT_SOURCE_DIR}/doc ."
        COMMAND sensible-browser ${CMAKE_CURRENT_SOURCE_DIR}/doc/html/index.html
        VERBATIM)

endif()

        
add_custom_target(clang-format
        COMMAND ${CMAKE_COMMAND} -E echo_append ""
        VERBATIM)