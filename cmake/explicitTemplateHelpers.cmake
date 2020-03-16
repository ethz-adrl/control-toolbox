# reads the templates from file
function(ct_getDimensionsFromLine LineContent)
    string(REGEX REPLACE "," ";" LineContent ${LineContent})

    foreach(NameAndValue ${LineContent})    
      # Strip leading spaces
      string(REGEX REPLACE "^[ ]+" "" NameAndValue ${NameAndValue})
      # Find variable name
      string(REGEX MATCH "^[^=]+" Name ${NameAndValue})
      # Find the value
      string(REPLACE "${Name}=" "" Value ${NameAndValue})
      
      # Set the variable
      if(Name STREQUAL "STATE_DIM")
          set(STATE_DIM_PRESPEC "${Value}" PARENT_SCOPE)
      endif()
      
        # Set the variable
      if(Name STREQUAL "CONTROL_DIM")
          set(CONTROL_DIM_PRESPEC "${Value}" PARENT_SCOPE)
      endif()
      
      if(Name STREQUAL "SCALAR")
          set(SCALAR_PRESPEC "${Value}" PARENT_SCOPE)
      endif()
      
      if(Name STREQUAL "POS_DIM")
          set(POS_DIM_PRESPEC "${Value}" PARENT_SCOPE)
      endif()
      
      if(Name STREQUAL "VEL_DIM")
          set(VEL_DIM_PRESPEC "${Value}" PARENT_SCOPE)
      endif()
    endforeach()

endfunction()


# reads explicit templates from config file and gathers sources and libs
function(ct_configure_explicit_templates ConfigFile ConfigDir LibPrefix)

  FILE(READ "${ConfigFile}" contents)

  STRING(REGEX REPLACE ";" "\\\\;" contents "${contents}")
  STRING(REGEX REPLACE "\n" ";" contents "${contents}")
  
  #message(WARNING "file content: ${contents}")
  
  foreach(line ${contents})
      #message(WARNING "extracting variables from line: ${line}")
      set(STATE_DIM_PRESPEC "")
      set(CONTROL_DIM_PRESPEC "")
      set(SCALAR_PRESPEC "")
      set(POS_DIM_PRESPEC "0")
      set(VEL_DIM_PRESPEC "0")
      set(CURRENT_SRCS "")
  
      ct_getDimensionsFromLine(${line})
      
      #message(WARNING "extracted: STATE_DIM=${STATE_DIM_PRESPEC}, CONTROL_DIM=${STATE_DIM_PRESPEC}, SCALAR=${SCALAR_PRESPEC}")
      
      string(REGEX REPLACE "[^0-9a-zA-Z]+" "" SCALAR_PRESPEC_CLEAN ${SCALAR_PRESPEC})
      set(CURRENT_LIB_NAME "${LibPrefix}-${STATE_DIM_PRESPEC}-${CONTROL_DIM_PRESPEC}-${SCALAR_PRESPEC_CLEAN}-${POS_DIM_PRESPEC}-${VEL_DIM_PRESPEC}")
      
      if(STATE_DIM_PRESPEC AND CONTROL_DIM_PRESPEC AND SCALAR_PRESPEC)
          #message(WARNING "Will configure now")
          ct_configureFiles(${ConfigDir} ${STATE_DIM_PRESPEC}, ${CONTROL_DIM_PRESPEC}, ${SCALAR_PRESPEC} ${POS_DIM_PRESPEC} ${VEL_DIM_PRESPEC})
      elseif()
          #message(WARNING "Nothing to configure")
      endif()
      
      if(CURRENT_SRCS)
          set(${CURRENT_LIB_NAME}_SRCS ${CURRENT_SRCS} PARENT_SCOPE)
          list(APPEND LIB_NAMES "${CURRENT_LIB_NAME}")
      endif()
  endforeach()
  
  set(PRESPEC_LIB_NAMES ${LIB_NAMES} PARENT_SCOPE)

endfunction()


# finds cpp.in files and configures them
function(ct_configureFiles ConfigDir STATE_DIM_PRESPEC, CONTROL_DIM_PRESPEC, SCALAR_PRESPEC, POS_DIM_PREPSEC, VEL_DIM_PRESPEC)
    set(CURRENT_SRCS "")
    file(GLOB_RECURSE files "${ConfigDir}*.in")
    #message(WARNING "files to configure in directory ${ConfigDir}:\n ${files}")
    foreach(file ${files})
        string(REGEX REPLACE "[^0-9a-zA-Z]+" "" SCALAR_PRESPEC_CLEAN ${SCALAR_PRESPEC})
        string(REPLACE "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}" outputFile ${file})
        string(REPLACE ".cpp.in" "" outputFile ${outputFile})
        set(outputFile "${outputFile}-${STATE_DIM_PRESPEC}-${CONTROL_DIM_PRESPEC}-${SCALAR_PRESPEC_CLEAN}-${POS_DIM_PRESPEC}-${VEL_DIM_PRESPEC}.cpp")
        #message(WARNING "configuring file \n ${file} to \n ${outputFile} ")
        set(DOUBLE_OR_FLOAT false)
        if((SCALAR_PRESPEC MATCHES "double") OR (SCALAR_PRESPEC MATCHES "float")) #STREQUAL did not work
            set(DOUBLE_OR_FLOAT true)
        endif()
        configure_file(${file} ${outputFile})
        list(APPEND CURRENT_SRCS ${outputFile})
    endforeach()
   
    #(WARNING "CURRENT_SRCS: ${CURRENT_SRCS}")
    set(CURRENT_SRCS "${CURRENT_SRCS}" PARENT_SCOPE)
endfunction()


# link external library (for example to link optcon against lapack) # todo this should go away
function(ct_link_external_library extLibs)
foreach(lib_name ${PRESPEC_LIB_NAMES})
      target_link_libraries(${lib_name} "${extLibs}")
    endforeach()
endfunction()