# CLANG-TIDY AND CLANG-FORMAT
##############################

# Additional targets to perform clang-format/clang-tidy


# function to exclude user-defined folders from the clang format- and tidy process     
function(filter_ct_directories allItems excludeDir)
    foreach (TMP_PATH ${${allItems}})
        if ("${TMP_PATH}" MATCHES ${excludeDir})
            list (REMOVE_ITEM ${allItems} ${TMP_PATH})
        endif()
    endforeach(TMP_PATH)
    set(${allItems} ${${allItems}} PARENT_SCOPE)
endfunction(filter_ct_directories)

function(ct_get_all_srcs)
    # Get all project files
    file(GLOB_RECURSE
     ALL_CXX_SOURCE_FILES
     ${PROJECT_SOURCE_DIR}
     *.cpp
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
          
     #message(FATAL_ERROR "sources list: ${ALL_CXX_SOURCE_FILES}$")
     set(ALL_CXX_SOURCE_FILES ${ALL_CXX_SOURCE_FILES} PARENT_SCOPE)
endfunction()



# Adding clang-format target if executable is found
find_program(CLANG_FORMAT_BIN "clang-format-3.9")
message(${CLANG_FORMAT_BIN})
if(NOT CLANG_FORMAT_BIN)
    add_custom_target(clang-format
        COMMAND ${CMAKE_COMMAND} -E echo_append "clang-format executable not found"
        VERBATIM)
else()
  message (WARNING "USING CLANG-FORMAT. This re-formats the source-code in a well-defined style.")
  add_custom_target(
    clang-format
    COMMAND clang-format-3.9
    -i
    -style=file
    ${ALL_CXX_SOURCE_FILES}
    VERBATIM
    )
endif()

# Adding clang-tidy target if clang-tidy executable is found
function(ct_configure_clang_tidy TIDY_INC_DIRS)

    ct_get_all_srcs()

    set(CURRENT_INC_DIRS "")
    
    #message(FATAL_ERROR "Inc dirs: ${${TIDY_INC_DIRS}}")
    
    foreach (THIS_INC_DIR ${${TIDY_INC_DIRS}})
        #message(WARNING "this inc dir: ${THIS_INC_DIR}")
        list(APPEND CURRENT_INC_DIRS "-I${THIS_INC_DIR}")
    endforeach()
    
    #message(FATAL_ERROR "Current inc dirs: ${CURRENT_INC_DIRS}")

    find_program(CLANG_TIDY_BIN "clang-tidy-3.9")
        message(${CLANG_TIDY_BIN})
    if(NOT CLANG_TIDY_BIN)
        add_custom_target(clang-tidy
            COMMAND ${CMAKE_COMMAND} -E echo_append "clang-tidy executable not found"
            VERBATIM)
    else()
        message (WARNING "USING CLANG-TIDY to analyze the code for formatting issues...")
        set(CLANG_TIDY_COMMAND COMMAND clang-tidy-3.9 ${ALL_CXX_SOURCE_FILES} -config='' -- -std=c++11 ${CURRENT_INC_DIRS})
            
        add_custom_target(
            clang-tidy
            COMMAND ${CLANG_TIDY_COMMAND}
            COMMENT "Launching clang-tidy"
            WORKING_DIRECTORY ${CMAKE_PROJECT_DIR}
            
        )
    endif() #CLANG_TIDY_BIN
    
endfunction()
