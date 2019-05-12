## find blasfeo library based on user-defined environment variable
if(DEFINED ENV{BLASFEO_DIR})
    message(STATUS "Found BLASFEO environment variable.")
    list(APPEND blasfeo_INCLUDE_DIRS $ENV{BLASFEO_DIR}/include)

    find_library(blasfeo_libs_found blasfeo $ENV{BLASFEO_DIR}/lib)

    add_library(blasfeo STATIC IMPORTED)
    set_target_properties(blasfeo PROPERTIES IMPORTED_LOCATION ${blasfeo_libs_found})

    set_target_properties(blasfeo PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "$ENV{BLASFEO_DIR}/include"
        INTERFACE_LINK_LIBRARIES ${blasfeo_libs_found}
        )

    set(blasfeo_LIBRARIES ${blasfeo_libs_found})

    set(blasfeo_FOUND true)
else()
    message(STATUS "No ENV variable for blasfeo found.")
    set(blasfeo_FOUND false)
endif()