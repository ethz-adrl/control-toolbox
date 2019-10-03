## ----------------------------------------------
## find blasfeo library
## The Control Toolbox, Copyright M. Giftthaler
## ----------------------------------------------

# default init
set(blasfeo_FOUND false) 

## if the user installed blasfeo to /opt/blasfeo, use find_package MODULE mode
IF(EXISTS "/opt/blasfeo")
    message(STATUS "Found BLASFEO in /opt/blasfeo ...")
    
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
    ## if the user installed blasfeo using cmake, fall back to find_package CONFIG mode

    find_package(blasfeo CONFIG)

    set(blasfeo_LIBRARIES blasfeo)

    if(blasfeo_FOUND)
        message(STATUS "BLASFEO found in PACKAGE mode.")
    endif()
endif()

if(NOT blasfeo_FOUND)
    message(STATUS "BLASFEO NOT found.")
endif()