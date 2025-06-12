cmake_minimum_required(VERSION 3.16)
project(vsg_ode_robot VERSION 1.0.0)

# ---------- Home-brew helper ------------------------------
execute_process(COMMAND brew --prefix
                OUTPUT_VARIABLE HOMEBREW_PREFIX
                OUTPUT_STRIP_TRAILING_WHITESPACE)
list(PREPEND  CMAKE_PREFIX_PATH "${HOMEBREW_PREFIX}" "${HOMEBREW_PREFIX}/opt/ode")
list(APPEND   CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
# ----------------------------------------------------------

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(vsg REQUIRED)
find_package(vsgXchange REQUIRED)

# First try Config mode (works if you later build ODE yourself)
find_package(ODE CONFIG QUIET)

# Fallback to the module we just added
if(NOT ODE_FOUND)
    find_package(ODE REQUIRED)      # uses cmake/FindODE.cmake
endif()

find_package(Threads REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${ODE_INCLUDE_DIRS})

add_executable(vsg_ode_robot
    src/main.cpp src/Robot.cpp src/PhysicsWorld.cpp
    src/RobotController.cpp src/Terrain.cpp src/Visualizer.cpp)

target_link_libraries(vsg_ode_robot
    vsg::vsg vsgXchange::vsgXchange
    ${ODE_LIBRARIES} Threads::Threads)

# ---- Shader bundle ------------------------------------------------------
set(SHADER_SRC "${CMAKE_CURRENT_SOURCE_DIR}/shaders")

# fall back to vsgExamples if user cloned it side-by-side
if(NOT EXISTS "${SHADER_SRC}" AND
   EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../vsgExamples/data/shaders")
    set(SHADER_SRC "${CMAKE_CURRENT_SOURCE_DIR}/../vsgExamples/data/shaders")
endif()

# final safety: only copy when the directory exists
if(EXISTS "${SHADER_SRC}")
    file(COPY "${SHADER_SRC}" DESTINATION "${CMAKE_BINARY_DIR}")
else()
    message(WARNING
      "No local shader bundle found – at run-time the app will rely on "
      "VSG_FILE_PATH pointing to e.g. vsgExamples/data")       # :contentReference[oaicite:1]{index=1}
endif()
# ------------------------------------------------------------------------

install(TARGETS vsg_ode_robot DESTINATION bin)
install(DIRECTORY shaders DESTINATION share/vsg_ode_robot OPTIONAL)