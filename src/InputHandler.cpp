#include "InputHandler.h"

#ifndef USE_OPENGL_FALLBACK  // Only compile VSG version
#include "RobotController.h"
#include "Visualizer.h"
#include "DebugOutput.h"
#include "NoiseManager.h"
#include <iostream>
#include <iomanip>

InputHandler::InputHandler(RobotController* controller, Visualizer* vis)
    : robotController(controller), visualizer(vis)
{
}

void InputHandler::apply(vsg::KeyPressEvent& keyPress)
{
    auto key = keyPress.keyBase;
    
    // Minimal debug output - only for unknown keys
    if (key != vsg::KEY_w && key != vsg::KEY_a && key != vsg::KEY_s && key != vsg::KEY_d &&
        key != vsg::KEY_W && key != vsg::KEY_A && key != vsg::KEY_S && key != vsg::KEY_D &&
        key != vsg::KEY_Space && key != vsg::KEY_Escape && key != vsg::KEY_c && key != vsg::KEY_C &&
        key != vsg::KEY_v && key != vsg::KEY_V && key != vsg::KEY_n && key != vsg::KEY_N &&
        key != vsg::KEY_m && key != vsg::KEY_M) {
        std::cout << "[InputHandler] Unknown key: " << key << std::endl;
    }
    
    // Handle immediate key actions using correct VSG constants
    switch (key)
    {
        case vsg::KEY_Space:
            // Toggle control mode only on first press (not repeat)
            if (pressedKeys.find(key) == pressedKeys.end()) {
                manualControlActive = !manualControlActive;
                if (manualControlActive) {
                    robotController->setControlMode(RobotController::MANUAL);
                    std::cout << "\n\033[1;32m>>> MANUAL CONTROL ACTIVATED - Use WASD to move <<<\033[0m\n" << std::endl;
                } else {
                    robotController->setControlMode(RobotController::AUTONOMOUS);
                    currentMovement = vsg::vec3(0.0f, 0.0f, 0.0f);
                    currentRotation = 0.0f;
                    std::cout << "\n\033[1;36m>>> AUTONOMOUS MODE ACTIVATED <<<\033[0m\n" << std::endl;
                }
                keyPress.handled = true;
            }
            break;
            
        case vsg::KEY_w:
        case vsg::KEY_W:
        case vsg::KEY_s:
        case vsg::KEY_S:
        case vsg::KEY_a:
        case vsg::KEY_A:
        case vsg::KEY_d:
        case vsg::KEY_D:
            // Movement keys - update immediately
            if (manualControlActive) {
                keyPress.handled = true;
            }
            break;
            
        case vsg::KEY_c:
        case vsg::KEY_C:
            handleCameraSwitch();
            keyPress.handled = true;
            break;
            
        case vsg::KEY_v:
        case vsg::KEY_V:
            handleDebugToggle();
            keyPress.handled = true;
            break;
            
        case vsg::KEY_n:
        case vsg::KEY_N:
            handleNoiseIncrease();
            keyPress.handled = true;
            break;
            
        case vsg::KEY_m:
        case vsg::KEY_M:
            handleNoiseDecrease();
            keyPress.handled = true;
            break;
            
        case vsg::KEY_Escape:
            // Exit simulation
            std::cout << "\nExiting simulation..." << std::endl;
            visualizer->close();
            keyPress.handled = true;
            break;
    }
    
    // Add key to pressed set AFTER handling Space key
    pressedKeys.insert(key);
    keyPressTime[key] = std::chrono::steady_clock::now();
    
    // Update movement if manual control is active
    if (manualControlActive) {
        updateMovement();
    }
}

void InputHandler::apply(vsg::KeyReleaseEvent& keyRelease)
{
    auto key = keyRelease.keyBase;
    
    pressedKeys.erase(key);
    keyPressTime.erase(key);
    
    switch (key)
    {
        case vsg::KEY_Shift_L:
        case vsg::KEY_Shift_R:
            sprintMode = false;
            break;
    }
    
    // Update movement if manual control is active
    if (manualControlActive) {
        updateMovement();
    }
}

void InputHandler::apply(vsg::ButtonPressEvent& buttonPress)
{
    if (buttonPress.button == 1) { // Left mouse button
        mousePressed = true;
        lastMousePos = vsg::vec2(buttonPress.x, buttonPress.y);
    }
}

void InputHandler::apply(vsg::ButtonReleaseEvent& buttonRelease)
{
    if (buttonRelease.button == 1) {
        mousePressed = false;
    }
}

void InputHandler::apply(vsg::MoveEvent& moveEvent)
{
    if (mousePressed) {
        vsg::vec2 currentPos(moveEvent.x, moveEvent.y);
        vsg::vec2 delta = currentPos - lastMousePos;
        
        // Use mouse movement for camera control when not in follow mode
        // This will be handled by the camera controller
        
        lastMousePos = currentPos;
    }
}

void InputHandler::apply(vsg::ScrollWheelEvent& scrollWheel)
{
    // Zoom in/out for camera
    visualizer->adjustCameraDistance(scrollWheel.delta.y * -2.0f);
}

bool InputHandler::isKeyPressed(vsg::KeySymbol key) const
{
    return pressedKeys.find(key) != pressedKeys.end();
}

void InputHandler::updateMovement()
{
    if (!manualControlActive) {
        currentMovement = vsg::vec3(0.0f, 0.0f, 0.0f);
        currentRotation = 0.0f;
        return;
    }
    
    // Track key changes without debug output
    static std::set<vsg::KeySymbol> lastPressedKeys;
    bool keysChanged = (pressedKeys != lastPressedKeys);
    if (keysChanged) {
        lastPressedKeys = pressedKeys;
    }
    
    // Calculate movement based on pressed keys
    vsg::vec3 movement(0.0f, 0.0f, 0.0f);
    float rotation = 0.0f;
    
    // Forward/Backward - using correct VSG constants
    if (isKeyPressed(vsg::KEY_w) || isKeyPressed(vsg::KEY_W)) {
        movement.x += 1.0f;
    }
    if (isKeyPressed(vsg::KEY_s) || isKeyPressed(vsg::KEY_S)) {
        movement.x -= 1.0f;
    }
    
    // Left/Right turning - using correct VSG constants
    if (isKeyPressed(vsg::KEY_a) || isKeyPressed(vsg::KEY_A)) {
        rotation += 1.0f;
    }
    if (isKeyPressed(vsg::KEY_d) || isKeyPressed(vsg::KEY_D)) {
        rotation -= 1.0f;
    }
    
    // Apply speed modifiers
    float speed = baseSpeed;
    if (sprintMode) {
        speed *= sprintMultiplier;
    }
    
    // Normalize horizontal movement
    if (movement.x != 0.0f || movement.y != 0.0f) {
        float mag = std::sqrt(movement.x * movement.x + movement.y * movement.y);
        movement.x = (movement.x / mag) * speed;
        movement.y = (movement.y / mag) * speed;
    }
    movement.z *= verticalSpeed;
    
    currentMovement = movement;
    currentRotation = rotation * rotationSpeed;
    
    // Send movement commands to robot controller
    if (robotController) {
        robotController->setManualVelocity(currentMovement);
        robotController->setManualRotation(currentRotation);
    }
}

void InputHandler::handleCameraSwitch()
{
    static int cameraMode = 0;
    cameraMode = (cameraMode + 1) % 6;
    
    switch (cameraMode) {
        case 0:
            visualizer->setCameraMode(Visualizer::CameraMode::FOLLOW);
            break;
        case 1:
            visualizer->setCameraMode(Visualizer::CameraMode::FREE);
            break;
        case 2:
            visualizer->setCameraMode(Visualizer::CameraMode::ORBIT);
            break;
        case 3:
            visualizer->setCameraMode(Visualizer::CameraMode::TOP);
            break;
        case 4:
            visualizer->setCameraMode(Visualizer::CameraMode::FRONT);
            break;
        case 5:
            visualizer->setCameraMode(Visualizer::CameraMode::SIDE);
            break;
    }
}

void InputHandler::handleDebugToggle()
{
    // Cycle through debug levels
    Debug::Level currentLevel = Debug::currentLevel;
    Debug::Level newLevel;
    
    switch (currentLevel) {
        case Debug::NONE:
            newLevel = Debug::ERROR;
            break;
        case Debug::ERROR:
            newLevel = Debug::WARNING;
            break;
        case Debug::WARNING:
            newLevel = Debug::INFO;
            break;
        case Debug::INFO:
            newLevel = Debug::VERBOSE;
            break;
        case Debug::VERBOSE:
            newLevel = Debug::ALL;
            break;
        case Debug::ALL:
        default:
            newLevel = Debug::NONE;
            break;
    }
    
    Debug::setLevel(newLevel);
}

void InputHandler::handleNoiseIncrease()
{
    NoiseManager::getInstance().increaseNoise();
    float currentLevel = NoiseManager::getInstance().getNoiseLevel();
    std::cout << "\033[1;33mNoise level increased to: " << std::fixed << std::setprecision(1) << currentLevel << "\033[0m" << std::endl;
}

void InputHandler::handleNoiseDecrease()
{
    NoiseManager::getInstance().decreaseNoise();
    float currentLevel = NoiseManager::getInstance().getNoiseLevel();
    std::cout << "\033[1;33mNoise level decreased to: " << std::fixed << std::setprecision(1) << currentLevel << "\033[0m" << std::endl;
}

#endif // USE_OPENGL_FALLBACK
