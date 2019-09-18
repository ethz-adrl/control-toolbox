## find blasfeo library based on user-defined environment variable
IF(EXISTS "/opt/blasfeo")
    message(STATUS "Found BLASFEO directory /opt/blasfeo ...")
    list(APPEND blasfeo_INCLUDE_DIRS /opt/blasfeo/include)

    find_library(blasfeo_libs_found blasfeo /opt/blasfeo/lib)

    add_library(blasfeo STATIC IMPORTED)
    set_target_properties(blasfeo PROPERTIES IMPORTED_LOCATION ${blasfeo_libs_found})

    set_target_properties(blasfeo PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "/opt/blasfeo/include"
        INTERFACE_LINK_LIBRARIES ${blasfeo_libs_found}
        )

    set(blasfeo_LIBRARIES ${blasfeo_libs_found})

    set(blasfeo_FOUND true)
else()
    message(STATUS "BLASFEO not found.")
    set(blasfeo_FOUND false)
endif()