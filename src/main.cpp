#include <iostream>
#include <memory>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <unistd.h>

#include "Robot.h"
#include "PhysicsWorld.h"
#include "RobotController.h"
#include "Terrain.h"
#include "Visualizer.h"

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
        // Create visualizer
        visualizer = std::make_unique<Visualizer>(1920, 1080);
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
        
        // Add robot visual model to scene root
        robot->addToScene(sceneRoot);
        std::cout << "Robot physics and visual model initialized" << std::endl;

        // Create robot controller
        controller = std::make_unique<RobotController>(robot.get(), physicsWorld.get());
        controller->setControlMode(RobotController::MANUAL);  // Manual control for stability
        controller->enableObstacleAvoidance(true);
        controller->enableDynamicStability(true);

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
        const double fixedTimeStep = 1.0 / 240.0; // 240 Hz physics for better stability

        // Navigation goal disabled for visual testing
        // RobotController::NavigationGoal goal;
        // goal.position = makePosition(10.0f, 0.0f, 10.0f);
        // goal.speed = 1.0f;
        // goal.tolerance = 0.5f;
        // controller->setNavigationGoal(goal);

        std::cout << "Starting main render loop..." << std::endl;
        
        // Give window a moment to initialize
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        int frame = 0;
        while (!visualizer->shouldClose() && !g_shouldExit && frame < 300) { // Run for ~5 seconds at 60fps
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
                
                // Ground clamping - prevent robot from sinking below ground
                const dReal* bodyPos = dBodyGetPosition(robot->getBody());
                // Minimum height calculated from leg geometry: body must be ~0.61m above ground for feet to touch
                const float minBodyHeight = 0.61f;  // Proper height for feet to reach ground
                
                // More aggressive clamping - check if robot is sinking and stop it
                static float lastBodyZ = 1.0f;
                static int sinkingFrames = 0;
                
                if (bodyPos[2] < lastBodyZ - 0.001f) {  // Robot is sinking
                    sinkingFrames++;
                    if (sinkingFrames > 5 && bodyPos[2] < 0.7f) {  // Persistent sinking below 0.7m
                        // Force stop vertical motion
                        dBodySetPosition(robot->getBody(), bodyPos[0], bodyPos[1], std::max((float)bodyPos[2], minBodyHeight));
                        dBodySetLinearVel(robot->getBody(), 0, 0, 0);  // Stop all motion
                        dBodySetAngularVel(robot->getBody(), 0, 0, 0);
                        
                        // Also stop all leg segments
                        auto footGeoms = robot->getFootGeoms();
                        for (auto footGeom : footGeoms) {
                            dBodyID footBody = dGeomGetBody(footGeom);
                            if (footBody) {
                                const dReal* footVel = dBodyGetLinearVel(footBody);
                                dBodySetLinearVel(footBody, footVel[0], footVel[1], 0.0f);
                            }
                        }
                    }
                } else {
                    sinkingFrames = 0;
                }
                lastBodyZ = bodyPos[2];
                
                // Hard clamp at minimum height
                if (bodyPos[2] < minBodyHeight) {
                    dBodySetPosition(robot->getBody(), bodyPos[0], bodyPos[1], minBodyHeight);
                    const dReal* vel = dBodyGetLinearVel(robot->getBody());
                    if (vel[2] < 0) {
                        dBodySetLinearVel(robot->getBody(), vel[0], vel[1], 0.0f);
                    }
                }
                
                // Clamp feet to ground - foot boxes should stay above ground
                auto footGeoms = robot->getFootGeoms();
                for (auto footGeom : footGeoms) {
                    dBodyID footBody = dGeomGetBody(footGeom);
                    if (footBody) {
                        const dReal* footPos = dBodyGetPosition(footBody);
                        // Foot boxes are large (8x leg radius = 0.32m), so center should be at least 0.16m above ground
                        const float minFootHeight = 0.16f;
                        if (footPos[2] < minFootHeight) {
                            dBodySetPosition(footBody, footPos[0], footPos[1], minFootHeight);
                            const dReal* footVel = dBodyGetLinearVel(footBody);
                            if (footVel[2] < 0) {
                                dBodySetLinearVel(footBody, footVel[0], footVel[1], 0.0f);
                            }
                        }
                    }
                }

                accumulator -= fixedTimeStep;
            }

            robot->updateLegPositions();

            // Update camera to follow robot
            vsg_vec3 robotPos = robot->getPosition();
            visualizer->enableCameraFollow(true, robotPos);
            
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
        // Check if manual control is enabled
        if (visualizer->isManualControlEnabled()) {
            // Get manual control velocity from visualizer
            vsg_vec3 vel = visualizer->getManualControlVelocity();
            
            // Apply manual control to robot through controller
            controller->setControlMode(RobotController::MANUAL);
            
            // Convert to control commands for the robot
            if (vel.x != 0 || vel.y != 0 || vel.z != 0) {
                // Apply velocities to robot body
                const dReal* currentVel = dBodyGetLinearVel(robot->getBody());
                dBodySetLinearVel(robot->getBody(), 
                    vel.x,  // Forward/backward
                    vel.y,  // Left/right
                    currentVel[2] + vel.z * 0.1f  // Up/down (dampened)
                );
            }
        } else {
            // Automated mode switching
            static int modeCounter = 0;
            modeCounter++;
            
            if (modeCounter % 600 == 0) {  // Every 10 seconds at 60 FPS
                static int currentMode = 0;
                currentMode = (currentMode + 1) % 3;
                
                switch (currentMode) {
                    case 0:
                        controller->setControlMode(RobotController::AUTONOMOUS);
                        std::cout << "Switched to AUTONOMOUS mode" << std::endl;
                        break;
                    case 1:
                        controller->setControlMode(RobotController::LEARNING);
                        controller->startLearning();
                        std::cout << "Switched to LEARNING mode" << std::endl;
                        break;
                    case 2:
                        controller->setControlMode(RobotController::DEMONSTRATION);
                        controller->recordPattern("demo_pattern");
                        std::cout << "Switched to DEMONSTRATION mode" << std::endl;
                        break;
                }
            }
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
        
        // Send stats to visualizer (it will append its own FPS info)
        visualizer->updateTextOverlay(robotStats.str(), "");
        
        // Display every 60 frames (1 second at 60 FPS)
        if (frameCount % 60 == 0) {
            std::cout << "\n=== Robot Status ===" << std::endl;
            std::cout << "Position: (" << pos.x << ", " << pos.y << ", " << pos.z << ")" << std::endl;
            std::cout << "Velocity: (" << vel.x << ", " << vel.y << ", " << vel.z << ")" << std::endl;
            std::cout << "Stable: " << (stable ? "Yes" : "No") << std::endl;
            
            // Check if ground clamping is active
            if (pos.z < 0.65f) {
                std::cout << "*** GROUND CLAMPING ACTIVE - Robot at Z=" << pos.z << " ***" << std::endl;
            }
            std::cout << "Efficiency: " << efficiency * 100 << "%" << std::endl;
        }
        
        // Always check for very low positions
        if (pos.z < 0.1f) {
            std::cout << "WARNING: Robot body Z=" << pos.z << " (very low!)" << std::endl;
        }
    }

private:
    std::unique_ptr<Visualizer> visualizer;
    std::unique_ptr<PhysicsWorld> physicsWorld;
    std::unique_ptr<Robot> robot;
    std::unique_ptr<RobotController> controller;
    std::unique_ptr<Terrain> terrain;
    
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
        std::cout << "\n=== CONTROLS ===" << std::endl;
        std::cout << "WASD     - Move robot (when manual control is on)" << std::endl;
        std::cout << "Q/E      - Move up/down" << std::endl;
        std::cout << "Space    - Toggle manual control" << std::endl;
        std::cout << "C        - Cycle camera modes" << std::endl;
        std::cout << "H        - Toggle help overlay" << std::endl;
        std::cout << "F1       - Toggle stats display" << std::endl;
        std::cout << "F2       - Toggle shadows" << std::endl;
        std::cout << "F3       - Toggle SSAO" << std::endl;
        std::cout << "Mouse    - Free camera control (drag to rotate)" << std::endl;
        std::cout << "Ctrl+C   - Graceful shutdown" << std::endl;
        std::cout << "================\n" << std::endl;
        
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