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

# we exclude "external" (i.e. CppAD) sources from the format- and tidy process     
filter_ct_directories(ALL_CXX_SOURCE_FILES "/external/")     
          

# Adding clang-format target if executable is found
if(CLANG_FORMAT)
    find_program(CLANG_FORMAT_BIN "clang-format-3.9")
    message(${CLANG_FORMAT_BIN})
    if(NOT CLANG_FORMAT_BIN)
        message(WARNING "CLANG-FORMAT not found")
    else(NOT CLANG_FORMAT_BIN)
      message (WARNING "USING CLANG-FORMAT. This re-formats the source-code in a well-defined style.")
      add_custom_target(
        clang-format ALL
        COMMAND clang-format-3.9
        -i
        -style=file
        ${ALL_CXX_SOURCE_FILES}
        )
    endif()
endif(CLANG_FORMAT)


# Adding clang-tidy target if clang-tidy executable is found
if(CLANG_TIDY)
    find_program(CLANG_TIDY_BIN "clang-tidy-3.9")
        message(${CLANG_TIDY_BIN})
    if(NOT CLANG_TIDY_BIN)
        message(WARNING "CLANG-TIDY not found")
    else(NOT CLANG_TIDY_BIN)
        message (WARNING "USING CLANG-TIDY to analyze the code for formatting issues...")
        add_custom_target(
            clang-tidy ALL
            COMMAND clang-tidy-3.9 # todo how to get rid of version number?
            ${ALL_CXX_SOURCE_FILES}
            -config='' #uses the config file .clang-tidy (located in the root directory)
            --
            -std=c++11
            ${INCLUDE_DIRECTORIES}
            COMMENT "Launching clang-tidy"
        )
endif() #CLANG_TIDY_BIN
endif() #CLANG_TIDY
