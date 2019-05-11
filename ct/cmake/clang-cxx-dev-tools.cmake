##############
# CLANG-FORMAT
##############

# Additional targets to perform clang-format

# function to exclude user-defined folders from the clang format- and tidy process     
function(filter_ct_directories allItems excludeDir)
    foreach (TMP_PATH ${${allItems}})
        if ("${TMP_PATH}" MATCHES ${excludeDir})
            list (REMOVE_ITEM ${allItems} ${TMP_PATH})
        endif()
    endforeach(TMP_PATH)
    set(${allItems} ${${allItems}} PARENT_SCOPE)
endfunction(filter_ct_directories)

function(ct_get_all_srcs ADD_HEADERS)

    set(EXTENSIONS "*.cpp")
    if (ADD_HEADERS)
      list(APPEND EXTENSIONS "*.hpp" "*.h")
    endif()
    
    # Get all project files
    file(GLOB_RECURSE
     ALL_CXX_SOURCE_FILES
     ${PROJECT_SOURCE_DIR}
     ${EXTENSIONS}
     )
     
     
     # list "external" sources, to be excluded from the format- and tidy process: 
    filter_ct_directories(ALL_CXX_SOURCE_FILES "/external/") # excludes CppAD
    filter_ct_directories(ALL_CXX_SOURCE_FILES ".tpl.cpp") # excludes CppAD
    filter_ct_directories(ALL_CXX_SOURCE_FILES "/iit/rbd/")   # excludes iit rbd folders 
    filter_ct_directories(ALL_CXX_SOURCE_FILES "/testIrb4600/generated/")   # excludes generated code
    filter_ct_directories(ALL_CXX_SOURCE_FILES "/testhyq/generated/")   # excludes generated code
    filter_ct_directories(ALL_CXX_SOURCE_FILES "/HyA/generated/")   # excludes generated code
    filter_ct_directories(ALL_CXX_SOURCE_FILES "/HyQ/generated/")   # excludes generated code
    filter_ct_directories(ALL_CXX_SOURCE_FILES "/QuadrotorWithLoad/generated/")   # excludes generated code
    filter_ct_directories(ALL_CXX_SOURCE_FILES "transform6d.cpp") #excludes generated inverse kinematics code
    filter_ct_directories(ALL_CXX_SOURCE_FILES "/ikfast/ikfast.h") #excludes ik fast header
        
     #message(FATAL_ERROR "sources list: ${ALL_CXX_SOURCE_FILES}")
     set(ALL_CXX_SOURCE_FILES ${ALL_CXX_SOURCE_FILES} PARENT_SCOPE)
endfunction()



# Adding clang-format target if executable is found
ct_get_all_srcs(TRUE)
find_program(CLANG_FORMAT_BIN "clang-format")
if(NOT CLANG_FORMAT_BIN)
  find_program(CLANG_FORMAT_BIN "clang-format-3.9")
endif()
message(STATUS "CLANG FORMAT IS: " ${CLANG_FORMAT_BIN})
if(NOT CLANG_FORMAT_BIN)
    message (STATUS "CLANG-FORMAT not found. You can ignore this message if you are not a CT developer.")
    add_custom_target(clang-format
        COMMAND ${CMAKE_COMMAND} -E echo_append "clang-format executable not found"
        VERBATIM)
else()
  message (STATUS "FOUND CLANG-FORMAT")
  add_custom_target(
    clang-format
    COMMAND ${CLANG_FORMAT_BIN}
    -i
    -style=file
    ${ALL_CXX_SOURCE_FILES}
    VERBATIM
    )
endif()
