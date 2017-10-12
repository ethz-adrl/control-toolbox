# CLANG-TIDY AND CLANG-FORMAT
##############################

# Additional targets to perform clang-format/clang-tidy
# Get all project files
file(GLOB_RECURSE
     ALL_CXX_SOURCE_FILES
     *.[chi]pp *.[chi]xx *.cc *.hh *.ii *.[CHI] *.h
     )
          

# Adding clang-format target if executable is found
#find_program(CLANG_FORMAT_BIN "clang-format-3.9")
#message(${CLANG_FORMAT_BIN})
#if(NOT CLANG_FORMAT_BIN)
#    message(WARNING "CLANG-FORMAT not found")
#else(NOT CLANG_FORMAT_BIN)
#  message (WARNING "USING CLANG-FORMAT. This re-formats the source-code in a well-defined style.")
#  add_custom_target(
#    clang-format ALL
#    COMMAND clang-format-3.9
#    -i
#    -style=Google
#    ${ALL_CXX_SOURCE_FILES}
#    )
#endif()

# Adding clang-tidy target if clang-tidy executable is found
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
endif()
