
UNSET(CPPAD_INCLUDE_DIRS CACHE)
UNSET(CPPAD_LIBRARIES CACHE)


IF (CPPAD_INCLUDES AND CPPAD_LIBRARIES)
  SET(CPPAD_FIND_QUIETLY TRUE)
ENDIF ()


IF(DEFINED CPPAD_HOME)
 
  FIND_PATH(CPPAD_INCLUDE_DIR NAMES cppad/cppad.hpp
            PATHS  "${CPPAD_HOME}"
            NO_DEFAULT_PATH)

  FIND_LIBRARY(CPPAD_IPOPT_LIBRARY
               cppad_ipopt
               PATHS  "${CPPAD_HOME}/lib"
               NO_DEFAULT_PATH)

  SET(CPPAD_INCLUDE_DIRS ${CPPAD_INCLUDE_DIR})
  SET(CPPAD_LIBRARIES ${CPPAD_IPOPT_LIBRARY})

ELSE()

  FIND_PACKAGE(PkgConfig)

  IF( PKG_CONFIG_FOUND )
    pkg_check_modules( CPPAD QUIET cppad)
  ENDIF()


  IF( NOT CPPAD_FOUND )
    FIND_PATH(CPPAD_INCLUDE_DIR NAMES cppad/cppad.hpp
              HINTS  "$ENV{CPPAD_HOME}"
                     "/usr/include" )

    FIND_LIBRARY(CPPAD_IPOPT_LIBRARY
                 cppad_ipopt
                 HINTS "$ENV{CPPAD_HOME}/lib"
                       "/usr/lib" )

    IF( CPPAD_INCLUDE_DIR )
      SET(CPPAD_INCLUDE_DIRS ${CPPAD_INCLUDE_DIR})
    ENDIF()

    IF( CPPAD_IPOPT_LIBRARY )
        SET(CPPAD_LIBRARIES ${CPPAD_IPOPT_LIBRARY})
    ENDIF()

    INCLUDE(FindPackageHandleStandardArgs)
    # handle the QUIETLY and REQUIRED arguments and set CPPAD_FOUND to TRUE
    # if all listed variables are TRUE
    find_package_handle_standard_args(CppAD  DEFAULT_MSG
                                      CPPAD_INCLUDE_DIRS)

    MARK_AS_ADVANCED(CPPAD_INCLUDE_DIRS CPPAD_LIBRARIES)

  ENDIF()
ENDIF()


IF( CPPAD_FOUND AND NOT CPPAD_FIND_QUIETLY )
  MESSAGE(STATUS "package CppAD found")
ENDIF()

