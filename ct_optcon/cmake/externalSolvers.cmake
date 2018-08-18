option(HPIPM "Compile with HPIPM support" OFF)

## dummy HPIPM libs
set(HPIPM_LIBS "")

## include BLASFEO AND HPIPM
if(HPIPM)
    message(STATUS "Switching HPIPM ON")
    if(DEFINED ENV{BLASFEO_DIR})
        if(DEFINED ENV{HPIPM_DIR})
            message(WARNING "...Found HPIPM and BLASFEO environment variables")

            set(BLASFEO_INCLUDE_DIR $ENV{BLASFEO_DIR}/include)
            set(HPIPM_INCLUDE_DIR $ENV{HPIPM_DIR}/include)
            set(BLASFEO_DIR $ENV{BLASFEO_DIR})
            set(HPIPM_DIR $ENV{HPIPM_DIR})

            include_directories(${BLASFEO_INCLUDE_DIR})
            include_directories(${HPIPM_INCLUDE_DIR})
            link_directories(${BLASFEO_DIR}/lib)
            link_directories(${HPIPM_DIR}/lib)

            find_library(BLASFEO_LIBRARY blasfeo  ${BLASFEO_DIR}/lib REQUIRED)
            find_library(HPIPM_LIBRARY hpipm  ${HPIPM_DIR}/lib REQUIRED)

            set(HPIPM_LIBS hpipm blasfeo)

            add_definitions (-DHPIPM)

        else(DEFINED ENV{HPIPM_DIR})
            message(WARNING "WARNING: Trying to build with HPIPM, but no ENV variable for HPIPM found.")
        endif(DEFINED ENV{HPIPM_DIR})
    else(DEFINED ENV{BLASFEO_DIR})
        message(WARNING "WARNING: Trying to build with HPIPM, but no ENV variable for Blasfeo found.")
    endif(DEFINED ENV{BLASFEO_DIR})
endif(HPIPM)

## dummy nlp libs
set(NLP_LIBS "")

## include IPOPT
find_package(ipopt QUIET)
if(DEFINED ENV{IPOPT_SOURCE_DIR} OR ipopt_FOUND)
    set(BUILD_WITH_IPOPT_SUPPORT ON)
    message(STATUS "Found IPOPT - building with IPOPT support")
    if(ipopt_FOUND)
      message(STATUS "Using LOCAL installation of IPOPT")
      set(IPOPT_BUILD_DIR ${CMAKE_BINARY_DIR}/build/include/coin)
    elseif(DEFINED ENV{IPOPT_SOURCE_DIR})
      message(STATUS "using GLOBAL installation of IPOPT")
      set(IPOPT_BUILD_DIR $ENV{IPOPT_SOURCE_DIR}/build)
    else()
      message(FATAL_ERROR "ERROR: Ipopt source directory environment variable not set! Set IPOPT_SOURCE_DIR environment variable!")
    endif(ipopt_FOUND)

    include_directories("${IPOPT_BUILD_DIR}/include/coin")
    add_definitions( -DBUILD_WITH_IPOPT_SUPPORT )
    link_directories(${IPOPT_BUILD_DIR}/lib)
    set(IPOPT_LIBS ipopt dl coinmumps coinhsl lapack blas gfortran
        m quadmath coinmetis)

endif(DEFINED ENV{IPOPT_SOURCE_DIR} OR ipopt_FOUND)


## include SNOPT
if(DEFINED ENV{SNOPT_SOURCE_DIR})
    set(BUILD_WITH_SNOPT_SUPPORT ON)
    message(STATUS "Found SNOPT - building with SNOPT support")
    include_directories( "$ENV{SNOPT_SOURCE_DIR}/include")

    add_definitions( -DBUILD_WITH_SNOPT_SUPPORT )
    find_library(SNOPT_LIBRARY1 snopt7_cpp   $ENV{SNOPT_SOURCE_DIR}/lib REQUIRED)
    find_library(SNOPT_LIBRARY2 snopt7       $ENV{SNOPT_SOURCE_DIR}/lib REQUIRED)
    set(SNOPT_LIBS ${SNOPT_LIBRARY1} ${SNOPT_LIBRARY2})

    set(CT_SNOPT_LIBS ct_snopt_interface)
endif(DEFINED ENV{SNOPT_SOURCE_DIR})

set(NLP_LIBS ${IPOPT_LIBS} ${CT_SNOPT_LIBS})

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)