option(USE_CLANG "Use CLANG instead of gcc for faster compilation" false)
option(USE_INTEL "Use Intel ICC compiler" false)
option(BUILD_EXAMPLES "Compile all examples for ct" false)
option(BUILD_HYQ_FULL "Compile all examples for HyQ (takes long, should use clang)" false)
option(BUILD_HYQ_LINEARIZATION_TIMINGS "Build linearization timing tests for HyQ (takes long, should use clang)" false)
option(BUILD_HYA_LINEARIZATION_TIMINGS "Build linearization timing tests for HyA (takes long, should use clang)" false)
option(HPIPM "Build HPIPM Optimal Control solver" false)

## option to activate/deactivate explicit template prespecs
option(USE_PRESPEC "Compile with explicit template prespec" false)


if (USE_CLANG AND USE_INTEL)
    message (FATAL_ERROR "Please choose either intel or clang compiler or neither.")
endif()

set(CLANG_C_COMPILER "/usr/bin/clang" CACHE STRING "Path to Clang C compiler binary")
set(CLANG_CXX_COMPILER "/usr/bin/clang++" CACHE STRING "Path to Clang C++ compiler binary")
set(INTEL_C_COMPILER "/opt/intel/bin/icc" CACHE STRING "Path to Intel C compiler binary")
set(INTEL_CXX_COMPILER "/opt/intel/bin/icpc" CACHE STRING "Path to Intel C++ compiler binary")

if(USE_CLANG)
    message (WARNING "USING CLANG with bin ${CLANG_C_COMPILER} and ${CLANG_CXX_COMPILER}. This will make compilation faster but execution could be slower.")
    SET (CMAKE_C_COMPILER             ${CLANG_C_COMPILER})
    SET (CMAKE_CXX_COMPILER           ${CLANG_CXX_COMPILER})
endif(USE_CLANG)

if(USE_INTEL)
    message (WARNING "USING INTEL compiler with bin ${CLANG_C_COMPILER} and ${CLANG_CXX_COMPILER}.")
    SET (CMAKE_C_COMPILER             ${INTEL_C_COMPILER})
    SET (CMAKE_CXX_COMPILER           ${INTEL_CXX_COMPILER})
    set(CMAKE_CXX_LINKER_FLAGS "${CMAKE_CXX_LINKER_FLAGS} -L${MKLROOT}/lib/intel64 -llibblas -lmkl_intel_lp64 -lmkl_sequential -lmkl_core -lpthread -lm -llapacke -lblas -llapack -lliblapack -liblapacke")
endif(USE_INTEL)