#include <iostream>
#include <memory>
#include <chrono>
#include <thread>

#include "Robot.h"
#include "PhysicsWorld.h"
#include "RobotController.h"
#include "Terrain.h"
#include "Visualizer.h"

#include <vsg/all.h>
#include <vsgXchange/all.h>

class RobotSimulation {
public:
    RobotSimulation() {
        // Initialize ODE
        dInitODE2(0);
        dAllocateODEDataForThread(dAllocateMaskAll);
    }

    ~RobotSimulation() {
        dCloseODE();
    }

    bool initialize() {
        // Create visualizer
        visualizer = std::make_unique<Visualizer>(1920, 1080);
        if (!visualizer->initialize()) {
            std::cerr << "Failed to initialize visualizer" << std::endl;
            return false;
        }

        // Create scene root
        sceneRoot = vsg::Group::create();
        visualizer->setSceneRoot(sceneRoot);

        // Setup lighting
        setupLighting();

        // Create physics world
        physicsWorld = std::make_unique<PhysicsWorld>();
        physicsWorld->setGravity(vsg::vec3(0.0f, -9.81f, 0.0f));
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
        robot->setBodyColor(vsg::vec4(0.2f, 0.3f, 0.8f, 1.0f));
        robot->setMetallic(0.7f);
        robot->setRoughness(0.3f);

        // Create robot controller
        controller = std::make_unique<RobotController>(robot.get());
        controller->setControlMode(RobotController::AUTONOMOUS);
        controller->enableObstacleAvoidance(true);
        controller->enableDynamicStability(true);

        // Add some obstacles
        addObstacles();

        // Setup camera
        visualizer->setCameraPosition(vsg::vec3(10.0f, 8.0f, 10.0f));
        visualizer->setCameraTarget(vsg::vec3(0.0f, 0.0f, 0.0f));
        visualizer->enableCameraFollow(true);

        // Enable visual effects
        visualizer->enablePostProcessing(true);
        visualizer->enableShadows(true);
        visualizer->enableSSAO(true);
        visualizer->setAmbientLight(vsg::vec3(0.3f, 0.3f, 0.4f));

        return true;
    }

    void run() {
        auto lastTime = std::chrono::high_resolution_clock::now();
        double accumulator = 0.0;
        const double fixedTimeStep = 1.0 / 60.0; // 60 Hz physics

        // Set initial navigation goal
        RobotController::NavigationGoal goal;
        goal.position = vsg::vec3(10.0f, 0.0f, 10.0f);
        goal.speed = 1.0f;
        goal.tolerance = 0.5f;
        controller->setNavigationGoal(goal);

        while (!visualizer->shouldClose()) {
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

            // Update camera to follow robot
            vsg::vec3 robotPos = robot->getPosition();
            visualizer->enableCameraFollow(true, robotPos);

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
        }
    }

private:
    void setupLighting() {
        // Main directional light (sun)
        visualizer->addDirectionalLight(
            vsg::normalize(vsg::vec3(-1.0f, -2.0f, -1.0f)),
            vsg::vec3(1.0f, 0.95f, 0.8f),
            1.0f
        );

        // Fill light
        visualizer->addDirectionalLight(
            vsg::normalize(vsg::vec3(1.0f, -0.5f, 0.5f)),
            vsg::vec3(0.4f, 0.4f, 0.5f),
            0.3f
        );

        // Point lights for atmosphere
        visualizer->addPointLight(
            vsg::vec3(5.0f, 10.0f, 5.0f),
            vsg::vec3(1.0f, 0.8f, 0.6f),
            0.5f,
            20.0f
        );
    }

    void addObstacles() {
        // Add various obstacles
        for (int i = 0; i < 10; ++i) {
            float x = (float)(rand() % 40 - 20);
            float z = (float)(rand() % 40 - 20);
            float y = terrain->getHeightAt(x, z) + 0.5f;
            float size = 0.5f + (float)(rand() % 20) / 10.0f;

            physicsWorld->createBox(
                vsg::vec3(x, y, z),
                vsg::vec3(size, size, size),
                10.0f
            );

            // Add visual representation
            auto transform = vsg::MatrixTransform::create();
            transform->matrix = vsg::translate(x, y, z);
            
            auto box = vsg::Box::create();
            box->min = vsg::vec3(-size/2, -size/2, -size/2);
            box->max = vsg::vec3(size/2, size/2, size/2);
            
            auto node = vsg::Builder::create()->createBox(vsg::GeometryInfo{box});
            transform->addChild(node);
            sceneRoot->addChild(transform);
        }

        // Add ramps and stairs
        terrain->addRamp(
            vsg::vec3(-10.0f, 0.0f, -10.0f),
            vsg::vec3(-5.0f, 2.0f, -10.0f),
            3.0f
        );
    }

    void setRandomNavigationGoal() {
        RobotController::NavigationGoal goal;
        goal.position = vsg::vec3(
            (float)(rand() % 40 - 20),
            0.0f,
            (float)(rand() % 40 - 20)
        );
        goal.speed = 0.5f + (float)(rand() % 10) / 10.0f;
        goal.tolerance = 0.5f;
        controller->setNavigationGoal(goal);

        std::cout << "New navigation goal: " << goal.position.x << ", " << goal.position.z << std::endl;
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
        std::cout << "Position: " << robot->getPosition() << std::endl;
        std::cout << "Velocity: " << robot->getVelocity() << std::endl;
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
    vsg::ref_ptr<vsg::Group> sceneRoot;
    
    int frameCount = 0;
};

int main(int argc, char** argv) {
    try {
        RobotSimulation simulation;
        
        if (!simulation.initialize()) {
            std::cerr << "Failed to initialize simulation" << std::endl;
            return -1;
        }
        
        std::cout << "Robot simulation started!" << std::endl;
        std::cout << "The robot will autonomously navigate the terrain." << std::endl;
        std::cout << "Control modes will cycle automatically." << std::endl;
        
        simulation.run();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}