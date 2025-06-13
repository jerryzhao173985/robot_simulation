# FindODE.cmake - Find ODE (Open Dynamics Engine)
# This module defines:
#  ODE_FOUND - if ODE was found
#  ODE_INCLUDE_DIRS - include directories for ODE
#  ODE_LIBRARIES - libraries to link against ODE
#  ODE_LIBRARY_DIRS - library directories

find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
    pkg_check_modules(PC_ODE QUIET ode)
endif()

# Find the include directory
find_path(ODE_INCLUDE_DIR
    NAMES ode/ode.h
    HINTS ${PC_ODE_INCLUDE_DIRS}
    PATHS
        /usr/local/include
        /opt/homebrew/include
        /usr/include
        ${CMAKE_PREFIX_PATH}/include
)

# Find the library
find_library(ODE_LIBRARY
    NAMES ode
    HINTS ${PC_ODE_LIBRARY_DIRS}
    PATHS
        /usr/local/lib
        /opt/homebrew/lib
        /usr/lib
        ${CMAKE_PREFIX_PATH}/lib
)

# Handle the QUIETLY and REQUIRED arguments
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ODE
    FOUND_VAR ODE_FOUND
    REQUIRED_VARS ODE_LIBRARY ODE_INCLUDE_DIR
    VERSION_VAR PC_ODE_VERSION
)

if(ODE_FOUND)
    set(ODE_LIBRARIES ${ODE_LIBRARY})
    set(ODE_INCLUDE_DIRS ${ODE_INCLUDE_DIR})
    
    # Extract library directory
    get_filename_component(ODE_LIBRARY_DIRS ${ODE_LIBRARY} DIRECTORY)
    
    if(NOT TARGET ODE::ODE)
        add_library(ODE::ODE UNKNOWN IMPORTED)
        set_target_properties(ODE::ODE PROPERTIES
            IMPORTED_LOCATION "${ODE_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${ODE_INCLUDE_DIR}"
        )
    endif()
endif()

mark_as_advanced(ODE_INCLUDE_DIR ODE_LIBRARY)
