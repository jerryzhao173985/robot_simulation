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
#include "EnhancedRobotController.h"
#include "Terrain.h"
#include "Visualizer.h"
#include "InputHandler.h"
#include "DebugOutput.h"
#include "NoiseManager.h"
// Event handlers only available with VSG

#ifndef USE_OPENGL_FALLBACK
#include <vsg/all.h>
#include <vsgXchange/all.h>
// Event handlers only available with VSG
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
        enhancedController.reset();
        controller.reset();
        robot.reset();
        physicsWorld.reset();
        dCloseODE();
    }

    bool initialize() {
        // Create visualizer with smaller window size for better performance
        visualizer = std::make_unique<Visualizer>(1024, 768);
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
        visualizer->enableVSync(true);  // Enable VSync for smoother rendering

        // Setup lighting
        setupLighting();

        // Create physics world
        physicsWorld = std::make_unique<PhysicsWorld>();
        physicsWorld->setGravity(vsg_vec3(0.0f, 0.0f, -9.81f)); // Standard gravity
        physicsWorld->setGroundFriction(1.0f);   // Normal friction
        physicsWorld->setGroundBounce(0.0f);     // No bounce
        physicsWorld->enableAdaptiveStepping(false); // Use fixed timestep for stability

        // Create terrain
        terrain = std::make_unique<Terrain>(physicsWorld.get(), sceneRoot);
        Terrain::TerrainParams terrainParams;
        terrainParams.size = 50.0f;
        terrainParams.resolution = 128;
        terrainParams.maxHeight = 3.0f;
        terrainParams.roughness = 0.4f;
        // Use flat terrain for testing ground collision
        terrain->generate(Terrain::FLAT, terrainParams);

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
        
        // Connect robot legs to visualizer's robot transform
#ifndef USE_OPENGL_FALLBACK
        auto vizRobotTransform = visualizer->getRobotTransform();
        if (vizRobotTransform) {
            robot->setVisualParent(vizRobotTransform);
            std::cout << "Connected robot legs to visualizer's transform" << std::endl;
            // Recompile scene to include dynamic robot geometry
            visualizer->compileScene();
            std::cout << "Recompiled scene with robot geometry" << std::endl;
        } else {
            // Fallback: add robot's own transform to scene
            robot->addToScene(sceneRoot);
            visualizer->compileScene();
        }
#else
        robot->addToScene(sceneRoot);
#endif
        
        // Create robot controller - enhanced version with advanced algorithms
        enhancedController = std::make_unique<EnhancedRobotController>(robot.get(), physicsWorld.get());
        
        // Configure enhanced parameters
        EnhancedRobotController::EnhancedParams enhancedParams;
        enhancedParams.useInverseKinematics = true;
        enhancedParams.adaptiveGait = true;
        enhancedParams.terrainAdaptation = true;
        enhancedParams.dynamicStability = true;
        enhancedParams.enablePathPlanning = true;
        enhancedParams.enableAdvancedGait = true;
        enhancedParams.enableLearning = false;  // Disable learning initially
        enhancedController->setEnhancedParams(enhancedParams);
        
        // Use base controller interface for compatibility
        controller = enhancedController.get();
        
        controller->setControlMode(RobotController::AUTONOMOUS);  // Start in autonomous mode
        controller->enableObstacleAvoidance(true);
        controller->enableDynamicStability(true);
        
        std::cout << "✅ Enhanced Robot Controller initialized with advanced algorithms!" << std::endl;
        
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
        // Input handler added
        
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
        
        // Add a test box that should fall and hit the ground
        physicsWorld->createBox(
            vsg_vec3(0.0f, 2.0f, 2.0f),  // Position above ground
            vsg_vec3(0.5f, 0.5f, 0.5f),   // Size
            1.0f                          // Mass
        );
        

        // Setup camera to look at robot initial position in Orbit mode
        float robotInitialZ = 0.5f; // From Robot::createBody() - proper standing height
        visualizer->setCameraPosition(vsg_vec3(5.0f, 5.0f, robotInitialZ + 2.0f));
        visualizer->setCameraTarget(vsg_vec3(0.0f, 0.0f, robotInitialZ));
        visualizer->setCameraMode(Visualizer::CameraMode::ORBIT);  // Set default to Orbit mode
        visualizer->enableCameraFollow(false);
        
        // Camera positioned

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
        const double fixedTimeStep = 1.0 / 120.0; // 120 Hz physics for stable simulation

        // Set initial navigation goal to test leg movement
        RobotController::NavigationGoal goal;
        goal.position = makePosition(5.0f, 0.0f, 5.0f);
        goal.speed = 0.5f;  // Slower speed for better observation
        goal.tolerance = 0.5f;
        controller->setNavigationGoal(goal);

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
        int physicsUpdates = 0;
        auto loopStart = std::chrono::high_resolution_clock::now();
        
        while (!visualizer->shouldClose() && !g_shouldExit) { // Run until user closes
            auto currentTime = std::chrono::high_resolution_clock::now();
            double deltaTime = std::chrono::duration<double>(currentTime - lastTime).count();
            lastTime = currentTime;
            
            // Debug: Log frame timing every 10 frames
            if (frame % 10 == 0 && frame > 0) {
                double totalTime = std::chrono::duration<double>(currentTime - loopStart).count();
                std::cout << "[PERF] Frame " << frame << ": " 
                          << frame/totalTime << " FPS, "
                          << physicsUpdates/totalTime << " physics Hz, "
                          << "deltaTime=" << deltaTime << "s" << std::endl;
            }

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
                physicsUpdates++;
            }
            
            // Let physics handle ground contact naturally
            // No artificial intervention

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
            
            // Check if robot reached goal and set new one
            if (controller->hasReachedGoal()) {
                setRandomNavigationGoal();
                std::cout << "Robot reached goal, setting new random navigation target" << std::endl;
            }

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
    std::unique_ptr<EnhancedRobotController> enhancedController;
    std::unique_ptr<Terrain> terrain;
    
#ifdef USE_OPENGL_FALLBACK
    std::unique_ptr<InputHandler> inputHandler;
#endif
    InputHandler* inputHandlerPtr = nullptr;
#ifndef USE_OPENGL_FALLBACK
    // ComprehensiveEventDebugger* eventDebuggerPtr = nullptr;
    void* eventDebuggerPtr = nullptr;  // Disabled for now
#else
    void* eventDebuggerPtr = nullptr;
#endif
    
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