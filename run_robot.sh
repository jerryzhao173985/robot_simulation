#!/bin/bash
# Auto-generated run script with proper Vulkan environment

# Setup Vulkan environment
export VULKAN_SDK="$HOME/VulkanSDK/1.4.313.1/macOS"
export VK_ICD_FILENAMES="$VULKAN_SDK/share/vulkan/icd.d/MoltenVK_icd.json"
export VK_LAYER_PATH="$VULKAN_SDK/share/vulkan/explicit_layer.d"  
export PATH="$VULKAN_SDK/bin:$PATH"
export DYLD_LIBRARY_PATH="$VULKAN_SDK/lib:$DYLD_LIBRARY_PATH"

cd build && ./vsg_ode_robot
