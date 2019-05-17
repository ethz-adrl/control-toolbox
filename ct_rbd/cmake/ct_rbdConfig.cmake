# cmake-onfig file for the ct_rbd package

find_package(ct_optcon REQUIRED)

include(${CMAKE_CURRENT_LIST_DIR}/ct_rbd_export.cmake)

#define includes in legacy mode
get_target_property(ct_rbd_INCLUDE_DIRS ct_rbd INTERFACE_INCLUDE_DIRECTORIES)

# define libraries in legacy mode
get_property(ct_rbd_LIB TARGET ct_rbd PROPERTY INTERFACE_LINK_LIBRARIES)
list(APPEND ct_rbd_LIBRARIES ${ct_rbd_LIB})

# Required for catkin_simple to link against these libraries
set(ct_rbd_FOUND_CATKIN_PROJECT true)

