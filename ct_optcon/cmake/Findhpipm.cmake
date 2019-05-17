## find hpipm library based on user-defined environment variable
if(DEFINED ENV{HPIPM_DIR})
    message(STATUS "Found HPIPM environment variable.")
    
    list(APPEND hpipm_INCLUDE_DIRS $ENV{HPIPM_DIR}/include)

    find_library(hpipm_libs_found hpipm $ENV{HPIPM_DIR}/lib)

    add_library(hpipm STATIC IMPORTED)
    set_target_properties(hpipm PROPERTIES IMPORTED_LOCATION ${hpipm_libs_found})

    set_target_properties(hpipm PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "$ENV{HPIPM_DIR}/include"
        INTERFACE_LINK_LIBRARIES ${hpipm_libs_found}
        )
    set(hpipm_LIBRARIES ${hpipm_libs_found})

    set(hpipm_FOUND true)
else()
    message(STATUS "No ENV variable for HPIPM found.")
    set(hpipm_FOUND false)
endif()

