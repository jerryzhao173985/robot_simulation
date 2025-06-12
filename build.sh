#!/bin/bash

# VSG-ODE Robot Simulation Build Script

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}VSG-ODE Robot Simulation Build Script${NC}"
echo "======================================"

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check for required tools
echo -e "\n${YELLOW}Checking dependencies...${NC}"

if ! command_exists cmake; then
    echo -e "${RED}Error: CMake not found. Please install CMake 3.16 or higher.${NC}"
    exit 1
fi

if ! command_exists make; then
    echo -e "${RED}Error: Make not found. Please install build tools.${NC}"
    exit 1
fi

# Check for Vulkan
if ! pkg-config --exists vulkan 2>/dev/null && [ ! -d "$VULKAN_SDK" ]; then
    echo -e "${YELLOW}Warning: Vulkan SDK not found. Make sure it's installed and VULKAN_SDK is set.${NC}"
fi

# Create build directory
BUILD_DIR="build"
if [ -d "$BUILD_DIR" ]; then
    echo -e "${YELLOW}Build directory exists. Clean build? (y/n)${NC}"
    read -r CLEAN
    if [ "$CLEAN" = "y" ]; then
        rm -rf "$BUILD_DIR"
    fi
fi

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# Configure with CMake
echo -e "\n${YELLOW}Configuring with CMake...${NC}"

# --- Tell CMake & pkg-config where Home-brew lives ---------
BREW_PREFIX=$(brew --prefix)
export CMAKE_PREFIX_PATH="$BREW_PREFIX:$BREW_PREFIX/opt/ode:$CMAKE_PREFIX_PATH"
export PKG_CONFIG_PATH="$BREW_PREFIX/lib/pkgconfig:$PKG_CONFIG_PATH"   # :contentReference[oaicite:6]{index=6}
export ODE_DIR="$BREW_PREFIX/opt/ode"
# -----------------------------------------------------------

cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_OSX_ARCHITECTURES=arm64 \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  "$@"

# Build
echo -e "\n${YELLOW}Building...${NC}"
JOBS=$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)
make -j"$JOBS"

echo -e "\n${GREEN}Build complete!${NC}"
echo -e "Executable: ${BUILD_DIR}/vsg_ode_robot"
echo -e "\nTo run: cd ${BUILD_DIR} && ./vsg_ode_robot"