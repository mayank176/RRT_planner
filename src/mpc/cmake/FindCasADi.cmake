# Find CasADi
find_path(CASADI_INCLUDE_DIR
    NAMES casadi/casadi.hpp
    PATHS "/usr/local/include"
          "/usr/include"
)

find_library(CASADI_LIBRARY
    NAMES casadi
    PATHS "/usr/local/lib"
          "/usr/lib"
          "/usr/lib/x86_64-linux-gnu"
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CasADi DEFAULT_MSG
    CASADI_LIBRARY
    CASADI_INCLUDE_DIR
)

if(CASADI_FOUND)
    set(CASADI_LIBRARIES ${CASADI_LIBRARY})
    set(CASADI_INCLUDE_DIRS ${CASADI_INCLUDE_DIR})
endif()

mark_as_advanced(CASADI_INCLUDE_DIR CASADI_LIBRARY)
