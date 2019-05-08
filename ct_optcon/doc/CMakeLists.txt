find_package(Doxygen)
if(DOXYGEN_FOUND)

    set(doxyfile_in ct_optcon.doxyfile)
    set(doxyfile ${CMAKE_CURRENT_SOURCE_DIR}/../Doxyfile)

    configure_file(${doxyfile_in} ${doxyfile} @ONLY)

    add_custom_target( doc
        COMMAND ${CMAKE_COMMAND} -E echo_append "Building API Documentation..."
        COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMAND ${CMAKE_COMMAND} -E echo_append "API Documentation built in ${CMAKE_CURRENT_SOURCE_DIR}"
        VERBATIM)
    
    # todo ct_core docu is currently broken
    #install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/html DESTINATION share/ct_optcon/doc)
endif()