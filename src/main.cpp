#include <iostream>
#include <memory>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <unistd.h>
#include <sstream>
#include <iomanip>

#include "Robot.h"
#include "PhysicsWorld.h"
#include "RobotController.h"
#include "Terrain.h"
#include "Visualizer.h"
#include "InputHandler.h"
#include "DebugOutput.h"
#include "NoiseManager.h"
#include "TestEventHandler.h"
#include "ComprehensiveEventDebugger.h"

#ifndef USE_OPENGL_FALLBACK
#include <vsg/all.h>
#include <vsgXchange/all.h>
#endif

#include "PositionUtils.h"

// Global flag for graceful shutdown
std::atomic<bool> g_shouldExit{false};

void signalHandler(int signal) {
    std::cout << "\nReceived signal " << signal << ", shutting down gracefully..." << std::endl;
    g_shouldExit = true;
}

class RobotSimulation {
public:
    RobotSimulation() {
        // Initialize ODE
        dInitODE2(0);
        dAllocateODEDataForThread(dAllocateMaskAll);
    }

    ~RobotSimulation() {
        // Ensure all ODE-dependent objects are destroyed before closing ODE
        terrain.reset();
        controller.reset();
        robot.reset();
        physicsWorld.reset();
        dCloseODE();
    }

    bool initialize() {
        // Create visualizer with smaller window size
        visualizer = std::make_unique<Visualizer>(800, 600);
        if (!visualizer->initialize()) {
            std::cerr << "Failed to initialize visualizer" << std::endl;
            return false;
        }

#ifdef USE_OPENGL_FALLBACK
        // For OpenGL fallback, create simple scene root
        sceneRoot = ref_ptr<Group>(new Group());
#else
        // Create scene root
        sceneRoot = vsg::Group::create();
#endif
        visualizer->setSceneRoot(sceneRoot);
        visualizer->enableVSync(false);

        // Setup lighting
        setupLighting();

        // Create physics world
        physicsWorld = std::make_unique<PhysicsWorld>();
        physicsWorld->setGravity(vsg_vec3(0.0f, 0.0f, -9.81f)); // Full gravity - rely on good physics params
        physicsWorld->setGroundFriction(10.0f);  // Extremely high friction to prevent sliding
        physicsWorld->setGroundBounce(0.0f);     // No bounce at all
        physicsWorld->enableAdaptiveStepping(true);

        // Create terrain
        terrain = std::make_unique<Terrain>(physicsWorld.get(), sceneRoot);
        Terrain::TerrainParams terrainParams;
        terrainParams.size = 50.0f;
        terrainParams.resolution = 128;
        terrainParams.maxHeight = 3.0f;
        terrainParams.roughness = 0.4f;
        terrain->generate(Terrain::ROUGH, terrainParams);

        // Create robot
        robot = std::make_unique<Robot>(physicsWorld->getWorld(), physicsWorld->getSpace(), sceneRoot);
        robot->setBodyColor(vsg_vec4(0.2f, 0.3f, 0.8f, 1.0f));
        robot->setMetallic(0.7f);
        robot->setRoughness(0.3f);
        
        // Set physics world for contact sensors
        robot->setPhysicsWorld(physicsWorld.get());
        
        // Initialize NoiseManager with number of sensors and actuators
        NoiseManager::getInstance().init(
            robot->getSensorNames().size(),  // Number of sensors
            robot->getActuatorNames().size() // Number of actuators
        );
        
        // Add robot visual model to scene root
        robot->addToScene(sceneRoot);
        std::cout << "Robot physics and visual model initialized" << std::endl;

        // Create robot controller
        controller = std::make_unique<RobotController>(robot.get(), physicsWorld.get());
        controller->setControlMode(RobotController::AUTONOMOUS);  // Start in autonomous mode
        controller->enableObstacleAvoidance(true);
        controller->enableDynamicStability(true);
        
        std::cout << "RobotController created and configured:" << std::endl;
        std::cout << "  - Control Mode: AUTONOMOUS" << std::endl;
        std::cout << "  - Obstacle Avoidance: ENABLED" << std::endl;
        std::cout << "  - Dynamic Stability: ENABLED" << std::endl;
        
        // Create input handler
#ifndef USE_OPENGL_FALLBACK
        // Add comprehensive event debugger (disabled for performance)
        // auto eventDebugger = ComprehensiveEventDebugger::create();
        // visualizer->addEventHandler(eventDebugger);
        // std::cout << "Added ComprehensiveEventDebugger to visualizer" << std::endl;
        
        // Add test handler to verify events (disabled for performance)
        // auto testHandler = TestEventHandler::create();
        // visualizer->addEventHandler(testHandler);
        // std::cout << "Added TestEventHandler to visualizer" << std::endl;
        
        auto vsgInputHandler = InputHandler::create(controller.get(), visualizer.get());
        visualizer->addEventHandler(vsgInputHandler);
        std::cout << "Added InputHandler to visualizer" << std::endl;
        
        // Store raw pointer for access (VSG manages lifetime)
        inputHandlerPtr = vsgInputHandler.get();
        
        // Store debugger for summary
        // eventDebuggerPtr = eventDebugger.get();
        eventDebuggerPtr = nullptr;
#else
        inputHandler = std::make_unique<InputHandler>(controller.get(), visualizer.get());
        inputHandlerPtr = inputHandler.get();
#endif

        // Add some obstacles
        addObstacles();
        

        // Setup camera to look at robot initial position
        float robotInitialZ = 1.0f; // From Robot::createBody()
        visualizer->setCameraPosition(vsg_vec3(5.0f, 5.0f, robotInitialZ + 2.0f));
        visualizer->setCameraTarget(vsg_vec3(0.0f, 0.0f, robotInitialZ));
        visualizer->enableCameraFollow(false);
        
        std::cout << "Camera looking at robot initial position Z=" << robotInitialZ << std::endl;

        // Enable visual effects
        visualizer->enablePostProcessing(true);
        visualizer->enableShadows(true);
        visualizer->enableSSAO(true);
        visualizer->setAmbientLight(vsg_vec3(0.3f, 0.3f, 0.4f));
        
        // Try to add a skybox for better environment
        visualizer->addSkybox("vsgExamples/data/textures/skybox.ktx");
        
        // Add environment lighting
        visualizer->setEnvironmentLighting(0.2f, vsg_vec3(0.5f, -0.5f, -1.0f));

        return true;
    }

    void run() {
        auto lastTime = std::chrono::high_resolution_clock::now();
        double accumulator = 0.0;
        const double fixedTimeStep = 1.0 / 60.0; // 60 Hz physics for better performance

        // Navigation goal disabled for visual testing
        // RobotController::NavigationGoal goal;
        // goal.position = makePosition(10.0f, 0.0f, 10.0f);
        // goal.speed = 1.0f;
        // goal.tolerance = 0.5f;
        // controller->setNavigationGoal(goal);

        std::cout << "Starting main render loop..." << std::endl;
        std::cout << "\n=== CONTROLS ===" << std::endl;
        std::cout << "Space    - Toggle manual/autonomous control" << std::endl;
        std::cout << "WASD     - Move robot (manual mode only)" << std::endl;
        std::cout << "C        - Cycle camera modes" << std::endl;
        std::cout << "V        - Cycle debug verbosity" << std::endl;
        std::cout << "N/M      - Increase/Decrease noise level" << std::endl;
        std::cout << "ESC      - Exit simulation" << std::endl;
        std::cout << "================\n" << std::endl;
        
        std::cout << "INITIAL STATE: " << (controller->getControlMode() == RobotController::AUTONOMOUS ? "AUTONOMOUS" : "MANUAL") << " mode" << std::endl;
        
        // Give window a moment to initialize
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        int frame = 0;
        while (!visualizer->shouldClose() && !g_shouldExit) { // Run until user closes
            auto currentTime = std::chrono::high_resolution_clock::now();
            double deltaTime = std::chrono::duration<double>(currentTime - lastTime).count();
            lastTime = currentTime;

            // Cap delta time to prevent spiral of death
            deltaTime = std::min(deltaTime, 0.1);
            accumulator += deltaTime;

            // Fixed timestep physics update
            while (accumulator >= fixedTimeStep) {
                // Update physics
                physicsWorld->step(fixedTimeStep);

                // Update robot controller
                controller->update(fixedTimeStep);

                // Update robot
                robot->update(fixedTimeStep);
                
                // Log critical info when noise is active
                static int physicsUpdateCounter = 0;
                if (++physicsUpdateCounter % 600 == 0 && NoiseManager::getInstance().getNoiseLevel() > 0.0f) {
                    vsg_vec3 pos = robot->getPosition();
                    vsg_vec3 vel = robot->getVelocity();
                    std::cout << "\n[PHYSICS UPDATE] Time=" << std::fixed << std::setprecision(1) 
                             << physicsUpdateCounter * fixedTimeStep 
                             << "s, Pos=(" << pos.x << "," << pos.y << "," << pos.z 
                             << "), Vel=" << vsg::length(vel) << "m/s" << std::endl;
                }

                accumulator -= fixedTimeStep;
            }
            
            // Ground clamping - do this once per frame, not per physics step
            const dReal* bodyPos = dBodyGetPosition(robot->getBody());
            const float minBodyHeight = 0.7f;  // Increased height to ensure body doesn't touch ground
            
            // Simple height clamping with buffer zone
            if (bodyPos[2] < minBodyHeight) {
                dBodySetPosition(robot->getBody(), bodyPos[0], bodyPos[1], minBodyHeight);
                const dReal* vel = dBodyGetLinearVel(robot->getBody());
                if (vel[2] < 0) {
                    dBodySetLinearVel(robot->getBody(), vel[0], vel[1], 0.0f);
                }
            }

            robot->updateLegPositions();

            // Update camera to follow robot only if stable
            vsg_vec3 robotPos = robot->getPosition();
            
            // Don't follow robot if it's flipped or too low/high
            if (robot->isStable() && robotPos.z > 0.2f && robotPos.z < 3.0f) {
                visualizer->enableCameraFollow(true, robotPos);
            } else {
                // Switch to free camera if robot is unstable
                visualizer->enableCameraFollow(false);
                if (visualizer->getCameraMode() == Visualizer::CameraMode::FOLLOW) {
                    visualizer->setCameraMode(Visualizer::CameraMode::FREE);
                    DEBUG_WARNING("Robot unstable - switching to free camera");
                }
            }
            
            // Update robot visual model to match physics body
            vsg_quat robotOrient = robot->getOrientation();
            visualizer->updateRobotTransform(robotPos, robotOrient);
            
            // Check if robot reached goal and set new one (disabled for visual testing)
            // if (controller->hasReachedGoal()) {
            //     setRandomNavigationGoal();
            // }

            // Handle keyboard input
            handleInput();

            // Render
            visualizer->render();

            // Display stats
            if (frameCount % 60 == 0) {
                displayStats();
            }
            frameCount++;
            frame++;
            
        }
        
        std::cout << "Rendered " << frame << " frames before exit" << std::endl;
    }

private:
    void setupLighting() {
        // Main directional light (sun)
        vsg_vec3 sunDir(-1.0f, -2.0f, -1.0f);
        sunDir = vsg::normalize(sunDir);
        visualizer->addDirectionalLight(
            sunDir,
            vsg_vec3(1.0f, 0.95f, 0.8f),
            1.0f
        );

        // Fill light
        vsg_vec3 fillDir(1.0f, -0.5f, 0.5f);
        fillDir = vsg::normalize(fillDir);
        visualizer->addDirectionalLight(
            fillDir,
            vsg_vec3(0.4f, 0.4f, 0.5f),
            0.3f
        );

        // Point lights for atmosphere
        visualizer->addPointLight(
            vsg_vec3(5.0f, 10.0f, 5.0f),
            vsg_vec3(1.0f, 0.8f, 0.6f),
            0.5f,
            20.0f
        );
    }

    void addObstacles() {
        // Add various obstacles
        for (int i = 0; i < 10; ++i) {
            float x = (float)(rand() % 40 - 20);
            float z = (float)(rand() % 40 - 20);
            float height = terrain->getHeightAt(x, z) + 0.5f;
            float size = 0.5f + (float)(rand() % 20) / 10.0f;

            physicsWorld->createBox(
                makePosition(x, height, z),
                vsg_vec3(size, size, size),
                10.0f
            );
        }

        // Add ramps and stairs
        terrain->addRamp(
            makePosition(-10.0f, 0.0f, -10.0f),
            makePosition(-5.0f, 2.0f, -10.0f),
            3.0f
        );
    }
    

    void setRandomNavigationGoal() {
        RobotController::NavigationGoal goal;
        {
            float gx = (float)(rand() % 40 - 20);
            float gy = (float)(rand() % 40 - 20);
            goal.position = makePosition(gx, 0.0f, gy);
        }
        goal.speed = 0.5f + (float)(rand() % 10) / 10.0f;
        goal.tolerance = 0.5f;
        controller->setNavigationGoal(goal);

        std::cout << "New navigation goal: " << goal.position.x << ", " << goal.position.y << std::endl;
    }

    void handleInput() {
        // Input is now handled by the InputHandler class
        // Update movement if manual control is active
        if (inputHandlerPtr && inputHandlerPtr->isManualControlActive()) {
            inputHandlerPtr->updateMovement();
        } else {
            // Automated mode switching DISABLED for debugging
            // static int modeCounter = 0;
            // modeCounter++;
            // 
            // if (modeCounter % 600 == 0) {  // Every 10 seconds at 60 FPS
            //     static int currentMode = 0;
            //     currentMode = (currentMode + 1) % 3;
            //     
            //     switch (currentMode) {
            //         case 0:
            //             controller->setControlMode(RobotController::AUTONOMOUS);
            //             std::cout << "Switched to AUTONOMOUS mode" << std::endl;
            //             break;
            //         case 1:
            //             controller->setControlMode(RobotController::LEARNING);
            //             controller->startLearning();
            //             std::cout << "Switched to LEARNING mode" << std::endl;
            //             break;
            //         case 2:
            //             controller->setControlMode(RobotController::DEMONSTRATION);
            //             controller->recordPattern("demo_pattern");
            //             std::cout << "Switched to DEMONSTRATION mode" << std::endl;
            //             break;
            //     }
            // }
        }
    }

    void displayStats() {
        static int frameCount = 0;
        frameCount++;
        
        // Get robot info
        vsg_vec3 pos = robot->getPosition();
        vsg_vec3 vel = robot->getVelocity();
        bool stable = robot->isStable();
        float efficiency = controller->getEfficiency();
        
        // Update visualizer's stats text with robot info
        std::stringstream robotStats;
        robotStats << "Robot Status:\n";
        robotStats << "Position: (" << std::fixed << std::setprecision(2) 
                   << pos.x << ", " << pos.y << ", " << pos.z << ")\n";
        robotStats << "Velocity: (" << std::fixed << std::setprecision(2)
                   << vel.x << ", " << vel.y << ", " << vel.z << ")\n";
        robotStats << "Stable: " << (stable ? "Yes" : "No") << "\n";
        robotStats << "Efficiency: " << std::fixed << std::setprecision(1) 
                   << efficiency * 100 << "%\n";
        robotStats << "Control Mode: ";
        switch (controller->getControlMode()) {
            case RobotController::MANUAL: robotStats << "Manual"; break;
            case RobotController::AUTONOMOUS: robotStats << "Autonomous"; break;
            case RobotController::LEARNING: robotStats << "Learning"; break;
            case RobotController::DEMONSTRATION: robotStats << "Demo"; break;
            default: robotStats << "Unknown"; break;
        }
        
        // Update visualizer text with new system
        static auto lastTime = std::chrono::high_resolution_clock::now();
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration<double>(currentTime - lastTime).count();
        
        // Calculate FPS
        static double fps = 60.0;
        static int fpsFrameCount = 0;
        fpsFrameCount++;
        if (duration >= 1.0) {
            fps = fpsFrameCount / duration;
            fpsFrameCount = 0;
            lastTime = currentTime;
        }
        
        // Get control mode string
        std::string controlMode;
        switch (controller->getControlMode()) {
            case RobotController::MANUAL: 
                controlMode = inputHandlerPtr && inputHandlerPtr->isManualControlActive() ? "MANUAL (Active)" : "MANUAL (Inactive)"; 
                break;
            case RobotController::AUTONOMOUS: controlMode = "AUTONOMOUS"; break;
            case RobotController::LEARNING: controlMode = "LEARNING"; break;
            case RobotController::DEMONSTRATION: controlMode = "DEMONSTRATION"; break;
            default: controlMode = "UNKNOWN"; break;
        }
        
        // Get movement commands if manual control is active
        vsg_vec3 moveCmd(0.0f, 0.0f, 0.0f);
        float rotCmd = 0.0f;
        if (inputHandlerPtr && inputHandlerPtr->isManualControlActive()) {
            moveCmd = inputHandlerPtr->getMovementVector();
            rotCmd = inputHandlerPtr->getRotationSpeed();
        }
        
        // Get sensor status
        auto sensorStatus = controller->getSensorStatus();
        
        // Update visualizer text overlay
        visualizer->updateStatsText(
            fps, 
            1000.0 / fps,  // Frame time in ms
            pos, 
            vel, 
            stable, 
            controlMode,
            moveCmd,
            rotCmd,
            sensorStatus.footContacts,
            sensorStatus.angularVelocity,
            sensorStatus.averageJointVelocity
        );
        
        // Update controls visibility
        if (inputHandlerPtr) {
            visualizer->updateControlsText(inputHandlerPtr->shouldShowHelp());
            visualizer->setTextVisible(inputHandlerPtr->shouldShowStats(), inputHandlerPtr->shouldShowHelp());
        }
        
        // Remove console status output - now displayed in overlay
        
        // Only warn about critical issues
        if (pos.z < 0.1f) {
            DEBUG_WARNING("Robot body very low at Z=" + std::to_string(pos.z));
        }
    }

private:
    std::unique_ptr<Visualizer> visualizer;
    std::unique_ptr<PhysicsWorld> physicsWorld;
    std::unique_ptr<Robot> robot;
    std::unique_ptr<RobotController> controller;
    std::unique_ptr<Terrain> terrain;
    
#ifdef USE_OPENGL_FALLBACK
    std::unique_ptr<InputHandler> inputHandler;
#endif
    InputHandler* inputHandlerPtr = nullptr;
    ComprehensiveEventDebugger* eventDebuggerPtr = nullptr;
    
#ifdef USE_OPENGL_FALLBACK
    ref_ptr<Group> sceneRoot;
#else
    vsg::ref_ptr<vsg::Group> sceneRoot;
#endif
    
    int frameCount = 0;
};

int main(int argc, char** argv) {
    try {
        // Register signal handler for graceful shutdown
        std::signal(SIGINT, signalHandler);
        std::signal(SIGTERM, signalHandler);
        
        std::cout << "Robot simulation started!" << std::endl;
        std::cout << "\n=== SIMPLIFIED CONTROLS ===" << std::endl;
        std::cout << "Space    - Toggle manual/autonomous control" << std::endl;
        std::cout << "WASD     - Move robot (manual mode only)" << std::endl;
        std::cout << "C        - Cycle camera modes" << std::endl;
        std::cout << "V        - Cycle debug verbosity" << std::endl;
        std::cout << "ESC      - Exit simulation" << std::endl;
        std::cout << "===========================\n" << std::endl;
        
#ifdef USE_OPENGL_FALLBACK
        std::cout << "Running with OpenGL/GLFW fallback renderer." << std::endl;
#else
        std::cout << "Running with VulkanSceneGraph renderer." << std::endl;
#endif
        
        {
            // Scope the simulation for controlled destruction order
            RobotSimulation simulation;
            
            if (!simulation.initialize()) {
                std::cerr << "Failed to initialize simulation" << std::endl;
                return -1;
            }
            
            simulation.run();
            
            std::cout << "\nSimulation completed successfully!" << std::endl;
        } // simulation destructor called here in controlled manner

        std::cout << "Cleanup complete." << std::endl;
        
#ifndef USE_OPENGL_FALLBACK
    } catch (const vsg::Exception& e) {
        std::cerr << "VSG Error: " << e.message << std::endl;
        return -1;
#endif
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}