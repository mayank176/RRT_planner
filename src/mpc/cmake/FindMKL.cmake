# FindMKL.cmake

if(NOT DEFINED ENV{MKLROOT})
    set(MKLROOT "/opt/intel/oneapi/mkl/latest")
endif()

# MKL include directory
find_path(MKL_INCLUDE_DIR
    NAMES mkl.h
    PATHS ${MKLROOT}/include
)

# MKL libraries
find_library(MKL_CORE_LIB
    NAMES mkl_core
    PATHS ${MKLROOT}/lib/intel64
)

find_library(MKL_INTEL_LP64_LIB
    NAMES mkl_intel_lp64
    PATHS ${MKLROOT}/lib/intel64
)

find_library(MKL_INTEL_THREAD_LIB
    NAMES mkl_intel_thread
    PATHS ${MKLROOT}/lib/intel64
)

find_library(MKL_SEQUENTIAL_LIB
    NAMES mkl_sequential
    PATHS ${MKLROOT}/lib/intel64
)

# Set MKL_LIBRARIES
set(MKL_LIBRARIES
    ${MKL_INTEL_LP64_LIB}
    ${MKL_CORE_LIB}
    ${MKL_SEQUENTIAL_LIB}
)

# Handle REQUIRED and QUIET arguments
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(MKL DEFAULT_MSG
    MKL_INCLUDE_DIR
    MKL_CORE_LIB
    MKL_INTEL_LP64_LIB
    MKL_SEQUENTIAL_LIB
)

mark_as_advanced(
    MKL_INCLUDE_DIR
    MKL_CORE_LIB
    MKL_INTEL_LP64_LIB
    MKL_SEQUENTIAL_LIB
    MKL_LIBRARIES
)
