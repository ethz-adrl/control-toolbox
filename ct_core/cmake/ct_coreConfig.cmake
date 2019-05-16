## cmake-config file for the ct_core package

include(${CMAKE_CURRENT_LIST_DIR}/ct_core_export.cmake)

#define includes in legacy mode
get_target_property(ct_core_INCLUDE_DIRS ct_core INTERFACE_INCLUDE_DIRECTORIES)

# define libraries in legacy mode
get_property(ct_core_LIB TARGET ct_plot PROPERTY LOCATION)
list(APPEND ct_core_LIBRARIES ${ct_core_LIB})

# Required for catkin_simple to link against these libraries
set(ct_core_FOUND_CATKIN_PROJECT true)

