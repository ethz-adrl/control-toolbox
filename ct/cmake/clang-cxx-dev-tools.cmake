# CLANG-TIDY AND CLANG-FORMAT
##############################

# Additional targets to perform clang-format/clang-tidy
# Get all project files
file(GLOB_RECURSE
     ALL_CXX_SOURCE_FILES
     *.[chi]pp *.[chi]xx *.cc *.hh *.ii *.[CHI] *.h
     )

# function to exclude user-defined folders from the clang format- and tidy process     
function(filter_ct_directories allItems excludeDir)
    foreach (TMP_PATH ${${allItems}})
        if ("${TMP_PATH}" MATCHES ${excludeDir})
            list (REMOVE_ITEM ${allItems} ${TMP_PATH})
        endif()
    endforeach(TMP_PATH)
    set(${allItems} ${${allItems}} PARENT_SCOPE)
endfunction(filter_ct_directories)

# list "external" sources, to be excluded from the format- and tidy process: 
filter_ct_directories(ALL_CXX_SOURCE_FILES "/external/") # excludes CppAD
filter_ct_directories(ALL_CXX_SOURCE_FILES "/iit/rbd/")   # excludes iit rbd folders 
filter_ct_directories(ALL_CXX_SOURCE_FILES "/testIrb4600/generated/")   # excludes generated code
filter_ct_directories(ALL_CXX_SOURCE_FILES "/testhyq/generated/")   # excludes generated code
filter_ct_directories(ALL_CXX_SOURCE_FILES "/HyA/generated/")   # excludes generated code
filter_ct_directories(ALL_CXX_SOURCE_FILES "/HyQ/generated/")   # excludes generated code
filter_ct_directories(ALL_CXX_SOURCE_FILES "/QuadrotorWithLoad/generated/")   # excludes generated code
          

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
find_program(CLANG_TIDY_BIN "clang-tidy-3.9")
    message(${CLANG_TIDY_BIN})
if(NOT CLANG_TIDY_BIN)
    add_custom_target(clang-tidy
        COMMAND ${CMAKE_COMMAND} -E echo_append "clang-tidy executable not found"
        VERBATIM)
else()
    message (WARNING "USING CLANG-TIDY to analyze the code for formatting issues...")
    add_custom_target(
        clang-tidy
        COMMAND clang-tidy-3.9 # todo how to get rid of version number?
        ${ALL_CXX_SOURCE_FILES}
        -config='' #uses the config file .clang-tidy (located in the root directory)
        --
        -std=c++11
        ${INCLUDE_DIRECTORIES}
        COMMENT "Launching clang-tidy"
        VERBATIM
    )
endif() #CLANG_TIDY_BIN
