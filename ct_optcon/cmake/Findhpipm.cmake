## ----------------------------------------------
## find hpipm library
## The Control Toolbox, Copyright M. Giftthaler
## ----------------------------------------------

# default init
set(hpipm_FOUND false) 

## if the user installed hpipm to /opt/hpipm, use find_package MODULE mode
IF(EXISTS "/opt/hpipm")
    message(STATUS "Found HPIPM in /opt/hpipm ...")
    
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
    ## if the user installed hpipm using cmake, fall back to find_package CONFIG mode

    find_package(hpipm CONFIG)

    set(hpipm_LIBRARIES hpipm)

    if(hpipm_FOUND)
        message(STATUS "HPIPM found in PACKAGE mode.")
    endif()
endif()

if(NOT hpipm_FOUND)
    message(STATUS "HPIPM NOT found.")
endif()