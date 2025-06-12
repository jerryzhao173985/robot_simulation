# FindODE.cmake – helper for Home-brew’s ODE keg (no ODEConfig.cmake)

find_path(ODE_INCLUDE_DIR NAMES ode/ode.h
          HINTS ENV ODE_DIR $ENV{HOMEBREW_PREFIX}/opt/ode /opt/homebrew/opt/ode)

find_library(ODE_LIBRARY NAMES ode
             HINTS ENV ODE_DIR $ENV{HOMEBREW_PREFIX}/opt/ode /opt/homebrew/opt/ode)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ODE DEFAULT_MSG ODE_INCLUDE_DIR ODE_LIBRARY)

set(ODE_INCLUDE_DIRS ${ODE_INCLUDE_DIR})
set(ODE_LIBRARIES    ${ODE_LIBRARY})
mark_as_advanced(ODE_INCLUDE_DIR ODE_LIBRARY)
