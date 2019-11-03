cmake_minimum_required(VERSION 3.3)
project(ct VERSION 3.0.2 LANGUAGES CXX)

#Make sure metapackage does not fail when building documentation
add_custom_target(doc
        COMMAND ${CMAKE_COMMAND} -E echo_append "Building CT Documentation..."
        VERBATIM)
        
add_custom_target(clang-format
        COMMAND ${CMAKE_COMMAND} -E echo_append "Running clang-format on CT..."
        VERBATIM)