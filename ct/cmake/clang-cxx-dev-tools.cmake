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
message(WARNING "CLANG FORMAT IS: " ${CLANG_FORMAT_BIN})
if(NOT CLANG_FORMAT_BIN)
    message (WARNING "CLANG-FORMAT not found. You can ignore this message if you are not a CT developer.")
    add_custom_target(clang-format
        COMMAND ${CMAKE_COMMAND} -E echo_append "clang-format executable not found"
        VERBATIM)
else()
  message (WARNING "FOUND CLANG-FORMAT")
  add_custom_target(
    clang-format
    COMMAND ${CLANG_FORMAT_BIN}
    -i
    -style=file
    ${ALL_CXX_SOURCE_FILES}
    VERBATIM
    )
endif()

# Adding clang-tidy target if clang-tidy executable is found
function(ct_configure_clang_tidy TIDY_INC_DIRS)

    ct_get_all_srcs(FALSE)

    set(CURRENT_INC_DIRS "")
    
    #message(FATAL_ERROR "Inc dirs: ${${TIDY_INC_DIRS}}")
    
    foreach (THIS_INC_DIR ${${TIDY_INC_DIRS}})
        #message(WARNING "this inc dir: ${THIS_INC_DIR}")
        list(APPEND CURRENT_INC_DIRS "-I${THIS_INC_DIR}")
    endforeach()
    
    #message(FATAL_ERROR "Current inc dirs: ${CURRENT_INC_DIRS}")

    find_program(CLANG_TIDY_BIN "clang-tidy")
    if(NOT CLANG_TIDY_BIN)
      find_program(CLANG_TIDY_BIN "clang-tidy-3.9")
    endif()
    if(NOT CLANG_TIDY_BIN)
      find_program(CLANG_TIDY_BIN "clang_tidy")
    endif()
        message(${CLANG_TIDY_BIN})
    if(NOT CLANG_TIDY_BIN)
        add_custom_target(clang-tidy
            COMMAND ${CMAKE_COMMAND} -E echo_append "clang-tidy executable not found"
            VERBATIM)
    else()
        message (WARNING "FOUND CLANG-TIDY")
        set(CLANG_TIDY_COMMAND COMMAND ${CLANG_TIDY_BIN} ${ALL_CXX_SOURCE_FILES} -config='' -header-filter=\".*\\/ct\\/.*\" -- -std=c++11 -fopenmp ${CURRENT_INC_DIRS})
            
        add_custom_target(
            clang-tidy
            COMMAND ${CLANG_TIDY_COMMAND}
            COMMENT "Launching clang-tidy"
            WORKING_DIRECTORY ${CMAKE_PROJECT_DIR}
            
        )
    endif() #CLANG_TIDY_BIN
    
endfunction()
