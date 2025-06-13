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
        physicsWorld->setGravity(vsg_vec3(0.0f, 0.0f, -9.81f));
        physicsWorld->setGroundFriction(1.2f);
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
        controller->setControlMode(RobotController::AUTONOMOUS);
        controller->enableObstacleAvoidance(true);
        controller->enableDynamicStability(true);

        // Add some obstacles
        addObstacles();
        

        // Setup camera
        visualizer->setCameraPosition(vsg_vec3(20.0f, 20.0f, 20.0f));
        visualizer->setCameraTarget(vsg_vec3(0.0f, 0.0f, 0.0f));
        visualizer->enableCameraFollow(false);

        // Enable visual effects
        visualizer->enablePostProcessing(true);
        visualizer->enableShadows(true);
        visualizer->enableSSAO(true);
        visualizer->setAmbientLight(vsg_vec3(0.3f, 0.3f, 0.4f));

        return true;
    }

    void run() {
        auto lastTime = std::chrono::high_resolution_clock::now();
        double accumulator = 0.0;
        const double fixedTimeStep = 1.0 / 60.0; // 60 Hz physics

        // Set initial navigation goal
        RobotController::NavigationGoal goal;
        goal.position = makePosition(10.0f, 0.0f, 10.0f);
        goal.speed = 1.0f;
        goal.tolerance = 0.5f;
        controller->setNavigationGoal(goal);

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

                accumulator -= fixedTimeStep;
            }

            robot->updateLegPositions();

            // Update camera to follow robot
            vsg_vec3 robotPos = robot->getPosition();
            visualizer->enableCameraFollow(true, robotPos);
            
            // Update robot visual model to match physics body
            vsg_quat robotOrient = robot->getOrientation();
            visualizer->updateRobotTransform(robotPos, robotOrient);
            
            // Check if robot reached goal and set new one
            if (controller->hasReachedGoal()) {
                setRandomNavigationGoal();
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
        // This would be connected to actual input handling
        // For now, we can switch control modes with simulated input
        
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

    void displayStats() {
        std::cout << "\n=== Robot Status ===" << std::endl;
        vsg_vec3 pos = robot->getPosition();
        vsg_vec3 vel = robot->getVelocity();
        std::cout << "Position: (" << pos.x << ", " << pos.y << ", " << pos.z << ")" << std::endl;
        std::cout << "Velocity: (" << vel.x << ", " << vel.y << ", " << vel.z << ")" << std::endl;
        std::cout << "Stable: " << (robot->isStable() ? "Yes" : "No") << std::endl;
        std::cout << "Energy: " << robot->getEnergyConsumption() << " W" << std::endl;
        std::cout << "Stability: " << controller->getStability() * 100 << "%" << std::endl;
        std::cout << "Efficiency: " << controller->getEfficiency() * 100 << "%" << std::endl;
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
        std::cout << "The robot will autonomously navigate the terrain." << std::endl;
        std::cout << "Control modes will cycle automatically." << std::endl;
        std::cout << "Press Ctrl+C for graceful shutdown." << std::endl;
        
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