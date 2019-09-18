## find hpipm library based on user-defined environment variable
IF(EXISTS "/opt/hpipm")
    message(STATUS "Found hpipm directory /opt/hpipm ...")
    list(APPEND hpipm_INCLUDE_DIRS /opt/hpipm/include)

    find_library(hpipm_libs_found hpipm /opt/hpipm/lib)

    add_library(hpipm STATIC IMPORTED)
    set_target_properties(hpipm PROPERTIES IMPORTED_LOCATION ${hpipm_libs_found})

    set_target_properties(hpipm PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "/opt/hpipm/include"
        INTERFACE_LINK_LIBRARIES ${hpipm_libs_found}
        )

    set(hpipm_LIBRARIES ${hpipm_libs_found})

    set(hpipm_FOUND true)
else()
    message(STATUS "hpipm not found.")
    set(hpipm_FOUND false)
endif()