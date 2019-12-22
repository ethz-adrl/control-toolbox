
# import interface compile definitions from ext_target as cmake option
function(importInterfaceCompileDefinitionsAsOptions ext_target)

    get_property(_if_compile_defs TARGET ${ext_target} PROPERTY INTERFACE_COMPILE_DEFINITIONS)

    foreach( i ${_if_compile_defs} )
        set(${i} ON PARENT_SCOPE) # mark all interface compile definitions as options
        message(STATUS "Importing compile definition " ${i} " as option.")
    endforeach()

endfunction()