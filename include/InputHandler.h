#pragma once

#ifndef USE_OPENGL_FALLBACK

#include <vsg/all.h>
#include <map>
#include <set>
#include <chrono>

class RobotController;
class Visualizer;

class InputHandler : public vsg::Inherit<vsg::Visitor, InputHandler>
{
public:
    InputHandler(RobotController* controller, Visualizer* vis);

    // Event handling
    void apply(vsg::KeyPressEvent& keyPress) override;
    void apply(vsg::KeyReleaseEvent& keyRelease) override;
    void apply(vsg::ButtonPressEvent& buttonPress) override;
    void apply(vsg::ButtonReleaseEvent& buttonRelease) override;
    void apply(vsg::MoveEvent& moveEvent) override;
    void apply(vsg::ScrollWheelEvent& scrollWheel) override;

    // State queries
    bool isKeyPressed(vsg::KeySymbol key) const;
    bool isManualControlActive() const { return manualControlActive; }
    
    // Movement control
    void updateMovement();
    vsg::vec3 getMovementVector() const { return currentMovement; }
    float getRotationSpeed() const { return currentRotation; }

    // UI toggles
    bool shouldShowStats() const { return showStats; }
    bool shouldShowHelp() const { return showHelp; }
    bool shouldShowDebug() const { return showDebug; }

private:
    RobotController* robotController;
    Visualizer* visualizer;
    
    // Key state tracking
    std::set<vsg::KeySymbol> pressedKeys;
    std::map<vsg::KeySymbol, std::chrono::steady_clock::time_point> keyPressTime;
    
    // Control states
    bool manualControlActive = false;
    bool sprintMode = false;
    
    // Movement state
    vsg::vec3 currentMovement{0.0f, 0.0f, 0.0f};
    float currentRotation = 0.0f;
    
    // Display toggles
    bool showStats = true;
    bool showHelp = false;
    bool showDebug = false;
    bool showShadows = true;
    bool showSSAO = false;
    
    // Mouse state
    vsg::vec2 lastMousePos;
    bool mousePressed = false;
    
    // Movement parameters
    const float baseSpeed = 2.0f;
    const float sprintMultiplier = 2.0f;
    const float rotationSpeed = 1.0f;
    const float verticalSpeed = 1.5f;
    
    // Key repeat prevention
    std::chrono::milliseconds keyRepeatDelay{100};
    
    void handleControlToggle(vsg::KeySymbol key);
    void handleCameraSwitch();
    void handleDisplayToggle(vsg::KeySymbol key);
    void handleDebugToggle();
    void handleNoiseIncrease();
    void handleNoiseDecrease();
};

#else // USE_OPENGL_FALLBACK

// Dummy input handler for OpenGL fallback
#include "FallbackTypes.h"
#include <memory>

class RobotController;
class Visualizer;

class InputHandler {
public:
    InputHandler(RobotController*, Visualizer*) {}
    static std::unique_ptr<InputHandler> create(RobotController* c, Visualizer* v) {
        return std::make_unique<InputHandler>(c, v);
    }
    void updateMovement() {}
    vec3 getMovementVector() const { return vec3(); }
    float getRotationSpeed() const { return 0; }
    bool shouldShowStats() const { return false; }
    bool shouldShowHelp() const { return false; }
    bool shouldShowDebug() const { return false; }
    bool isManualControlActive() const { return false; }
};

#endif // USE_OPENGL_FALLBACK