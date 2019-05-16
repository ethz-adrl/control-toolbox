# cmake-onfig file for the ct_models package

find_package(ct_rbd REQUIRED)

include(${CMAKE_CURRENT_LIST_DIR}/ct_models_export.cmake)

#define includes in legacy mode
get_target_property(ct_models_INCLUDE_DIRS ct_models INTERFACE_INCLUDE_DIRECTORIES)

# define libraries in legacy mode
get_property(ct_models_LIB TARGET ct_models PROPERTY INTERFACE_LINK_LIBRARIES)
list(APPEND ct_models_LIBRARIES ${ct_models_LIB})

# Required for catkin_simple to link against these libraries
set(ct_models_FOUND_CATKIN_PROJECT true)

