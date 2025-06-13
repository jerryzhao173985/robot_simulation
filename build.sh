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

# Check for Homebrew
if ! command_exists brew; then
    echo -e "${RED}Error: Homebrew not found. Please install Homebrew first.${NC}"
    echo "Visit https://brew.sh for installation instructions."
    exit 1
fi

# Check/install essential dependencies
echo -e "${YELLOW}Checking essential dependencies...${NC}"

# Check for pkg-config
if ! command_exists pkg-config; then
    echo -e "${YELLOW}Installing pkg-config...${NC}"
    brew install pkg-config
fi

# Check for ODE
if ! brew list ode >/dev/null 2>&1; then
    echo -e "${YELLOW}Installing ODE (Open Dynamics Engine)...${NC}"
    brew install ode
fi

echo -e "${GREEN}All essential dependencies are available.${NC}"

# Setup Vulkan SDK environment
VULKAN_SDK_DIR="$HOME/VulkanSDK/1.4.313.1/macOS"
if [ -d "$VULKAN_SDK_DIR" ]; then
    echo -e "${GREEN}Found Vulkan SDK at: $VULKAN_SDK_DIR${NC}"
    export VULKAN_SDK="$VULKAN_SDK_DIR"
    export VK_ICD_FILENAMES="$VULKAN_SDK_DIR/share/vulkan/icd.d/MoltenVK_icd.json"
    export VK_LAYER_PATH="$VULKAN_SDK_DIR/share/vulkan/explicit_layer.d"
    export PATH="$VULKAN_SDK_DIR/bin:$PATH"
    export DYLD_LIBRARY_PATH="$VULKAN_SDK_DIR/lib:$DYLD_LIBRARY_PATH"
    
    # Source the Vulkan setup script if it exists
    if [ -f "$HOME/VulkanSDK/1.4.313.1/setup-env.sh" ]; then
        source "$HOME/VulkanSDK/1.4.313.1/setup-env.sh"
    fi
    
    # Verify MoltenVK ICD file exists
    if [ -f "$VK_ICD_FILENAMES" ]; then
        echo -e "${GREEN}✓ MoltenVK ICD configured${NC}"
    else
        echo -e "${YELLOW}⚠ MoltenVK ICD file not found${NC}"
    fi
else
    echo -e "${YELLOW}Warning: Vulkan SDK not found at $VULKAN_SDK_DIR${NC}"
fi

# Check for VSG
if [ -d "/usr/local/lib/cmake/vsg" ]; then
    echo -e "${GREEN}Found VSG installation at /usr/local${NC}"
    export CMAKE_PREFIX_PATH="/usr/local:$CMAKE_PREFIX_PATH"
else
    echo -e "${YELLOW}Warning: VSG not found at /usr/local. Will use OpenGL fallback.${NC}"
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
export CMAKE_PREFIX_PATH="$BREW_PREFIX:$BREW_PREFIX/opt/ode:/usr/local:$CMAKE_PREFIX_PATH"
export PKG_CONFIG_PATH="$BREW_PREFIX/lib/pkgconfig:$PKG_CONFIG_PATH"
export ODE_DIR="$BREW_PREFIX/opt/ode"

# Set explicit paths for VSG
export vsg_DIR="/usr/local/lib/cmake/vsg"
export vsgXchange_DIR="/usr/local/lib/cmake/vsgXchange"
# -----------------------------------------------------------

cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_OSX_ARCHITECTURES=arm64 \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DCMAKE_PREFIX_PATH="/usr/local;$BREW_PREFIX;$BREW_PREFIX/opt/ode;$VULKAN_SDK_DIR" \
  -Dvsg_DIR="/usr/local/lib/cmake/vsg" \
  -DvsgXchange_DIR="/usr/local/lib/cmake/vsgXchange" \
  -DVulkan_INCLUDE_DIR="$VULKAN_SDK_DIR/include" \
  -DVulkan_LIBRARY="$VULKAN_SDK_DIR/lib/libvulkan.dylib" \
  "$@"

# Build
echo -e "\n${YELLOW}Building...${NC}"
JOBS=$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)
make -j"$JOBS"

BUILD_SUCCESS=$?

if [ $BUILD_SUCCESS -eq 0 ]; then
    echo -e "\n${GREEN}=== BUILD SUCCESSFUL! ===${NC}"
    echo -e "Executable: ${BUILD_DIR}/vsg_ode_robot"
    echo -e "\nTo run: cd ${BUILD_DIR} && ./vsg_ode_robot"
    
    # Check which graphics backend will be used
    if [ -f "./vsg_ode_robot" ]; then
        echo -e "\n${GREEN}Checking graphics backend...${NC}"
        
        # Check if VSG libraries are linked (VSG can be statically linked)
        # First check for dynamic linking
        if otool -L ./vsg_ode_robot | grep -q "libvsg"; then
            echo -e "${GREEN}✓ Using VSG/Vulkan backend (dynamically linked)${NC}"
        # Check for static linking by looking for VSG symbols
        elif nm ./vsg_ode_robot 2>/dev/null | grep -q "vsg::" || \
             strings ./vsg_ode_robot 2>/dev/null | grep -q "VSG_" || \
             otool -L ./vsg_ode_robot | grep -q "libvulkan"; then
            echo -e "${GREEN}✓ Using VSG/Vulkan backend (statically linked)${NC}"
        else
            echo -e "${YELLOW}⚠ Using OpenGL/GLFW fallback backend${NC}"
        fi
        
        # Check ODE linking
        if otool -L ./vsg_ode_robot | grep -q "libode"; then
            echo -e "${GREEN}✓ ODE physics engine linked${NC}"
        else
            echo -e "${RED}✗ ODE physics engine not found${NC}"
        fi
    fi
    
    echo -e "\n${GREEN}Ready to run the hexapod robot simulation!${NC}"
    
    # Create a run script with proper environment
    cat > ../run_robot.sh << 'EOF'
#!/bin/bash
# Auto-generated run script with proper Vulkan environment

# Setup Vulkan environment
export VULKAN_SDK="$HOME/VulkanSDK/1.4.313.1/macOS"
export VK_ICD_FILENAMES="$VULKAN_SDK/share/vulkan/icd.d/MoltenVK_icd.json"
export VK_LAYER_PATH="$VULKAN_SDK/share/vulkan/explicit_layer.d"  
export PATH="$VULKAN_SDK/bin:$PATH"
export DYLD_LIBRARY_PATH="$VULKAN_SDK/lib:$DYLD_LIBRARY_PATH"

cd build && ./vsg_ode_robot
EOF
    chmod +x ../run_robot.sh
    echo -e "${GREEN}Created run_robot.sh script with proper environment${NC}"
    echo -e "${YELLOW}Run the simulation with: ./run_robot.sh${NC}"
else
    echo -e "\n${RED}=== BUILD FAILED! ===${NC}"
    echo -e "${RED}Check the error messages above for details.${NC}"
    echo -e "\nCommon solutions:"
    echo -e "1. Make sure all dependencies are installed: brew install ode glfw"
    echo -e "2. For VSG issues, the simulation will fall back to OpenGL"
    echo -e "3. Check that Xcode Command Line Tools are installed: xcode-select --install"
    exit 1
fi