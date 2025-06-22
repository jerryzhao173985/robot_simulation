cmake_minimum_required(VERSION 3.16)
project(vsg_ode_robot VERSION 1.0.0)

# ---------- Vulkan SDK Configuration ------------------------------
set(VULKAN_SDK_DIR "$ENV{HOME}/VulkanSDK/1.4.313.1/macOS")
if(EXISTS "${VULKAN_SDK_DIR}")
    message(STATUS "Found Vulkan SDK at: ${VULKAN_SDK_DIR}")
    set(ENV{VULKAN_SDK} "${VULKAN_SDK_DIR}")
    list(PREPEND CMAKE_PREFIX_PATH "${VULKAN_SDK_DIR}")
    set(Vulkan_INCLUDE_DIR "${VULKAN_SDK_DIR}/include")
    set(Vulkan_LIBRARY "${VULKAN_SDK_DIR}/lib/libvulkan.dylib")
endif()

# ---------- Home-brew helper ------------------------------
execute_process(COMMAND brew --prefix
                OUTPUT_VARIABLE HOMEBREW_PREFIX
                OUTPUT_STRIP_TRAILING_WHITESPACE)
list(PREPEND  CMAKE_PREFIX_PATH "${HOMEBREW_PREFIX}" "${HOMEBREW_PREFIX}/opt/ode")
list(APPEND   CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# ---------- VSG Configuration ------------------------------
# Add VSG installation paths
list(PREPEND CMAKE_PREFIX_PATH "/usr/local")
set(vsg_DIR "/usr/local/lib/cmake/vsg")
set(vsgXchange_DIR "/usr/local/lib/cmake/vsgXchange")
# ----------------------------------------------------------

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Try to find VSG and vsgXchange
find_package(vsg QUIET)
find_package(vsgXchange QUIET)

# If VSG is not available, use OpenGL/GLFW fallback
if(NOT vsg_FOUND OR NOT vsgXchange_FOUND)
    message(STATUS "VSG not found, using OpenGL/GLFW fallback")
    set(USE_OPENGL_FALLBACK ON)  # Enable OpenGL fallback when VSG not available
    
    # Find OpenGL and GLFW
    find_package(OpenGL REQUIRED)
    find_package(glfw3 QUIET)
    
    # If GLFW not found via CMake, try homebrew
    if(NOT glfw3_FOUND)
        find_path(GLFW_INCLUDE_DIR NAMES GLFW/glfw3.h
                  HINTS ${HOMEBREW_PREFIX}/include /opt/homebrew/include)
        find_library(GLFW_LIBRARY NAMES glfw
                     HINTS ${HOMEBREW_PREFIX}/lib /opt/homebrew/lib)
        
        if(GLFW_INCLUDE_DIR AND GLFW_LIBRARY)
            set(glfw3_FOUND TRUE)
            add_library(glfw SHARED IMPORTED)
            set_target_properties(glfw PROPERTIES
                IMPORTED_LOCATION ${GLFW_LIBRARY}
                INTERFACE_INCLUDE_DIRECTORIES ${GLFW_INCLUDE_DIR})
        endif()
    endif()
    
    # Install GLFW if not found
    if(NOT glfw3_FOUND)
        message(STATUS "Installing GLFW via Homebrew...")
        execute_process(COMMAND brew install glfw)
        
        # Try finding again
        find_path(GLFW_INCLUDE_DIR NAMES GLFW/glfw3.h
                  HINTS ${HOMEBREW_PREFIX}/include /opt/homebrew/include)
        find_library(GLFW_LIBRARY NAMES glfw
                     HINTS ${HOMEBREW_PREFIX}/lib /opt/homebrew/lib)
                     
        if(GLFW_INCLUDE_DIR AND GLFW_LIBRARY)
            add_library(glfw SHARED IMPORTED)
            set_target_properties(glfw PROPERTIES
                IMPORTED_LOCATION ${GLFW_LIBRARY}
                INTERFACE_INCLUDE_DIRECTORIES ${GLFW_INCLUDE_DIR})
        endif()
    endif()
else()
    message(STATUS "VSG found! Using VSG/Vulkan backend")
    message(STATUS "vsg_FOUND: ${vsg_FOUND}")
    message(STATUS "vsgXchange_FOUND: ${vsgXchange_FOUND}")
endif()

# ---------- ODE Configuration -----------------------------------
# Auto-install ODE if not found
find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
    pkg_check_modules(ODE_PC QUIET ode)
endif()

# First try Config mode (works if ODE built with CMake)
find_package(ODE CONFIG QUIET)

# If not found, try our custom module
if(NOT ODE_FOUND AND NOT ODE_PC_FOUND)
    message(STATUS "ODE not found, attempting to install via Homebrew...")
    execute_process(
        COMMAND brew install ode
        RESULT_VARIABLE BREW_RESULT
        OUTPUT_QUIET ERROR_QUIET
    )
    
    if(BREW_RESULT EQUAL 0)
        message(STATUS "ODE installed successfully via Homebrew")
        # Refresh the search
        find_package(PkgConfig REQUIRED)
        pkg_check_modules(ODE_PC REQUIRED ode)
        
        # Set variables for compatibility
        set(ODE_FOUND TRUE)
        set(ODE_INCLUDE_DIRS ${ODE_PC_INCLUDE_DIRS})
        set(ODE_LIBRARIES ${ODE_PC_LIBRARIES})
        set(ODE_LIBRARY_DIRS ${ODE_PC_LIBRARY_DIRS})
    else()
        message(FATAL_ERROR "Failed to install ODE. Please install manually: brew install ode")
    endif()
endif()

# Fallback to our custom module if still not found
if(NOT ODE_FOUND)
    find_package(ODE REQUIRED)  # uses cmake/FindODE.cmake
endif()

# Print ODE configuration for debugging
message(STATUS "ODE Configuration:")
message(STATUS "  ODE_FOUND: ${ODE_FOUND}")
message(STATUS "  ODE_INCLUDE_DIRS: ${ODE_INCLUDE_DIRS}")
message(STATUS "  ODE_LIBRARIES: ${ODE_LIBRARIES}")
if(ODE_LIBRARY_DIRS)
    message(STATUS "  ODE_LIBRARY_DIRS: ${ODE_LIBRARY_DIRS}")
endif()
# ------------------------------------------------------------

find_package(Threads REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/noise ${ODE_INCLUDE_DIRS})

# Add library directories if specified
if(ODE_LIBRARY_DIRS)
    link_directories(${ODE_LIBRARY_DIRS})
endif()

add_executable(vsg_ode_robot
    src/main.cpp src/Robot.cpp src/PhysicsWorld.cpp
    src/RobotController.cpp src/Terrain.cpp src/Visualizer.cpp
    src/InputHandler.cpp src/DebugOutput.cpp
    src/Sensor.cpp src/Actuator.cpp src/NoiseManager.cpp
    noise/mac_drand48r.cpp)

# Add physics test executable (commented out - file doesn't exist)
# add_executable(test_physics
#     src/test_physics.cpp src/Robot.cpp src/PhysicsWorld.cpp
#     src/RobotController.cpp src/Terrain.cpp src/Visualizer.cpp)

# Link libraries based on available graphics backend
if(USE_OPENGL_FALLBACK)
    target_compile_definitions(vsg_ode_robot PRIVATE USE_OPENGL_FALLBACK)
    target_link_libraries(vsg_ode_robot
        ${ODE_LIBRARIES} 
        Threads::Threads
        OpenGL::GL 
        glfw)
    message(STATUS "Linking with OpenGL/GLFW fallback")
    message(STATUS "  ODE Libraries: ${ODE_LIBRARIES}")
else()
    target_link_libraries(vsg_ode_robot
        vsg::vsg 
        vsgXchange::vsgXchange
        ${ODE_LIBRARIES} 
        Threads::Threads)
    message(STATUS "Linking with VSG/Vulkan")
    message(STATUS "  VSG found: vsg::vsg, vsgXchange::vsgXchange")
    message(STATUS "  ODE Libraries: ${ODE_LIBRARIES}")
endif()

# Link test executable
if(USE_OPENGL_FALLBACK)
    # target_compile_definitions(test_physics PRIVATE USE_OPENGL_FALLBACK)
    # target_link_libraries(test_physics
    #     ${ODE_LIBRARIES} 
    #     Threads::Threads
    #     OpenGL::GL 
    #     glfw)
else()
    # target_link_libraries(test_physics
    #     vsg::vsg 
    #     vsgXchange::vsgXchange
    #     ${ODE_LIBRARIES} 
    #     Threads::Threads)
endif()

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
      "VSG_FILE_PATH pointing to e.g. vsgExamples/data")
endif()
# ------------------------------------------------------------------------

install(TARGETS vsg_ode_robot DESTINATION bin)
install(DIRECTORY shaders DESTINATION share/vsg_ode_robot OPTIONAL)