#!/bin/bash
# Screenshot capture script for robot simulation debugging

echo "Starting robot simulation with screenshot capture..."
echo "Press Cmd+Shift+5 to capture a screenshot manually"
echo "The simulation will run for 5 seconds..."

# Run the simulation in background
cd build
./vsg_ode_robot &
SIM_PID=$!

# Wait for simulation to start
sleep 2

# Take screenshot using macOS screencapture
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
SCREENSHOT_PATH="../screenshots/robot_sim_${TIMESTAMP}.png"
mkdir -p ../screenshots

echo "Taking screenshot in 3 seconds..."
sleep 3

# Capture the entire screen
screencapture -x "$SCREENSHOT_PATH"
echo "Screenshot saved to: $SCREENSHOT_PATH"

# Let simulation run a bit more
sleep 5

# Kill the simulation
kill $SIM_PID 2>/dev/null

echo "Screenshot capture complete!"
echo "View the screenshot at: $SCREENSHOT_PATH"