#include "InputHandler.h"
#include "RobotController.h"
#include "Visualizer.h"
#include "DebugOutput.h"
#include <iostream>

InputHandler::InputHandler(RobotController* controller, Visualizer* vis)
    : robotController(controller), visualizer(vis)
{
}

void InputHandler::apply(vsg::KeyPressEvent& keyPress)
{
    auto key = keyPress.keyBase;
    
    // Debug output for all key presses - using correct VSG enum names
    std::cout << "[DEBUG] Key pressed: " << key << " (";
    switch(key) {
        case vsg::KEY_w: std::cout << "w"; break;
        case vsg::KEY_a: std::cout << "a"; break;
        case vsg::KEY_s: std::cout << "s"; break;
        case vsg::KEY_d: std::cout << "d"; break;
        case vsg::KEY_W: std::cout << "W"; break;
        case vsg::KEY_A: std::cout << "A"; break;
        case vsg::KEY_S: std::cout << "S"; break;
        case vsg::KEY_D: std::cout << "D"; break;
        case vsg::KEY_Space: std::cout << "Space"; break;
        case vsg::KEY_Escape: std::cout << "Escape"; break;
        case vsg::KEY_c: std::cout << "c"; break;
        case vsg::KEY_C: std::cout << "C"; break;
        case vsg::KEY_v: std::cout << "v"; break;
        case vsg::KEY_V: std::cout << "V"; break;
        default: std::cout << "Unknown"; break;
    }
    std::cout << ")" << std::endl;
    
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
                std::cout << "[DEBUG] Movement key detected, updating movement" << std::endl;
            }
            break;
            
        case vsg::KEY_c:
        case vsg::KEY_C:
            handleCameraSwitch();
            break;
            
        case vsg::KEY_v:
        case vsg::KEY_V:
            handleDebugToggle();
            break;
            
        case vsg::KEY_Escape:
            // Exit simulation
            std::cout << "\nExiting simulation..." << std::endl;
            visualizer->close();
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
    
    // Debug output
    std::cout << "[DEBUG] Key released: " << key << std::endl;
    
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
    
    // Debug: show which keys are currently pressed
    std::cout << "[DEBUG] updateMovement called. Pressed keys: ";
    for (auto key : pressedKeys) {
        std::cout << key << " ";
    }
    std::cout << std::endl;
    
    // Calculate movement based on pressed keys
    vsg::vec3 movement(0.0f, 0.0f, 0.0f);
    float rotation = 0.0f;
    
    // Forward/Backward - using correct VSG constants
    if (isKeyPressed(vsg::KEY_w) || isKeyPressed(vsg::KEY_W)) {
        movement.x += 1.0f;
        std::cout << "[DEBUG] W pressed - moving forward" << std::endl;
    }
    if (isKeyPressed(vsg::KEY_s) || isKeyPressed(vsg::KEY_S)) {
        movement.x -= 1.0f;
        std::cout << "[DEBUG] S pressed - moving backward" << std::endl;
    }
    
    // Left/Right turning - using correct VSG constants
    if (isKeyPressed(vsg::KEY_a) || isKeyPressed(vsg::KEY_A)) {
        rotation += 1.0f;
        std::cout << "[DEBUG] A pressed - turning left" << std::endl;
    }
    if (isKeyPressed(vsg::KEY_d) || isKeyPressed(vsg::KEY_D)) {
        rotation -= 1.0f;
        std::cout << "[DEBUG] D pressed - turning right" << std::endl;
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
    
    // Debug output
    std::cout << "[DEBUG] Movement: (" << movement.x << ", " << movement.y << ", " << movement.z << ") Rotation: " << rotation << std::endl;
    
    // Send movement commands to robot controller
    if (robotController) {
        robotController->setManualVelocity(currentMovement);
        robotController->setManualRotation(currentRotation);
        std::cout << "[DEBUG] Sent to controller - Velocity: (" << currentMovement.x << ", " << currentMovement.y << ", " << currentMovement.z << ") Rotation: " << currentRotation << std::endl;
    }
}

void InputHandler::handleCameraSwitch()
{
    static int cameraMode = 0;
    cameraMode = (cameraMode + 1) % 6;
    
    switch (cameraMode) {
        case 0:
            visualizer->setCameraMode(Visualizer::CameraMode::FOLLOW);
            std::cout << "Camera mode: FOLLOW" << std::endl;
            break;
        case 1:
            visualizer->setCameraMode(Visualizer::CameraMode::FREE);
            std::cout << "Camera mode: FREE" << std::endl;
            break;
        case 2:
            visualizer->setCameraMode(Visualizer::CameraMode::ORBIT);
            std::cout << "Camera mode: ORBIT" << std::endl;
            break;
        case 3:
            visualizer->setCameraMode(Visualizer::CameraMode::TOP);
            std::cout << "Camera mode: TOP" << std::endl;
            break;
        case 4:
            visualizer->setCameraMode(Visualizer::CameraMode::FRONT);
            std::cout << "Camera mode: FRONT" << std::endl;
            break;
        case 5:
            visualizer->setCameraMode(Visualizer::CameraMode::SIDE);
            std::cout << "Camera mode: SIDE" << std::endl;
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
            std::cout << "\n[DEBUG] Level: ERROR - Only errors shown" << std::endl;
            break;
        case Debug::ERROR:
            newLevel = Debug::WARNING;
            std::cout << "\n[DEBUG] Level: WARNING - Errors and warnings shown" << std::endl;
            break;
        case Debug::WARNING:
            newLevel = Debug::INFO;
            std::cout << "\n[DEBUG] Level: INFO - Important information shown" << std::endl;
            break;
        case Debug::INFO:
            newLevel = Debug::VERBOSE;
            std::cout << "\n[DEBUG] Level: VERBOSE - Detailed debug output" << std::endl;
            break;
        case Debug::VERBOSE:
            newLevel = Debug::ALL;
            std::cout << "\n[DEBUG] Level: ALL - All debug output including physics" << std::endl;
            break;
        case Debug::ALL:
        default:
            newLevel = Debug::NONE;
            std::cout << "\n[DEBUG] Level: NONE - Debug output disabled" << std::endl;
            break;
    }
    
    Debug::setLevel(newLevel);
}