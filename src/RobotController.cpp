#include "RobotController.h"
#include "Robot.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <fstream>

RobotController::RobotController(Robot* robot) 
    : robot(robot), controlMode(MANUAL), currentPathIndex(0) {
    
    // Initialize control input
    currentInput.targetVelocity = vsg_vec3();
    currentInput.targetAngularVelocity = 0.0f;
    currentInput.jump = false;
    currentInput.crouch = false;
    currentInput.gaitType = 0;
    
    // Initialize navigation goal
    currentGoal.position = vsg_vec3();
    currentGoal.speed = 1.0f;
    currentGoal.tolerance = 0.5f;
    
    // Initialize PID controllers
    pitchController = {1.0f, 0.1f, 0.05f, 0.0f, 0.0f};
    rollController = {1.0f, 0.1f, 0.05f, 0.0f, 0.0f};
    yawController = {0.5f, 0.1f, 0.02f, 0.0f, 0.0f};
    heightController = {2.0f, 0.2f, 0.1f, 0.0f, 0.0f};
    
    // Initialize obstacle detection
    obstacleDirection = vsg_vec3();
}

RobotController::~RobotController() {
    if (isLearning) {
        stopLearning();
    }
    if (isRecording) {
        stopRecording();
    }
}

float RobotController::PIDController::update(float error, float dt) {
    integral += error * dt;
    float derivative = (error - previousError) / dt;
    previousError = error;
    
    return kp * error + ki * integral + kd * derivative;
}

void RobotController::update(double deltaTime) {
    // Process sensor data
    auto sensorReadings = robot->getSensorReadings();
    processSensorData(sensorReadings);
    
    // Update based on control mode
    switch (controlMode) {
        case MANUAL:
            updateManualControl(deltaTime);
            break;
        case AUTONOMOUS:
            updateAutonomousControl(deltaTime);
            break;
        case LEARNING:
            updateLearningControl(deltaTime);
            break;
        case DEMONSTRATION:
            updateDemonstrationControl(deltaTime);
            break;
    }
    
    // Always maintain balance if enabled
    if (dynamicStabilityEnabled) {
        maintainBalance();
    }
    
    // Update performance metrics
    updatePerformanceMetrics(deltaTime);
}

void RobotController::updateManualControl(double deltaTime) {
    // Apply direct control input
    std::vector<float> motorCommands;
    
    // Convert input to motor commands
    vsg::vec3 velocity = currentInput.targetVelocity;
    float angularVel = currentInput.targetAngularVelocity;
    
    // Simple mapping for demonstration
    motorCommands.push_back(velocity.x);
    motorCommands.push_back(velocity.z);
    motorCommands.push_back(angularVel);
    
    robot->applyControl(motorCommands);
}

void RobotController::updateAutonomousControl(double deltaTime) {
    // Navigate to goal
    if (!navigationPath.empty()) {
        followPath(deltaTime);
    } else if (hasNavigationGoal()) {
        planPath();
    }
    
    // Avoid obstacles
    if (obstacleAvoidanceEnabled) {
        avoidObstacles();
    }
    
    // Optimize gait
    optimizeGait();
    
    // Adapt to terrain
    adaptToTerrain();
}

void RobotController::updateLearningControl(double deltaTime) {
    // Collect training data
    if (isLearning) {
        auto sensorData = robot->getSensorReadings();
        std::vector<float> actions;
        
        // Simple learning: try random actions and evaluate
        for (int i = 0; i < 3; ++i) {
            actions.push_back((rand() % 200 - 100) / 100.0f);
        }
        
        // Store experience
        trainingData.push_back({sensorData, actions});
        
        // Apply actions
        robot->applyControl(actions);
        
        // Evaluate performance
        float reward = evaluatePerformance();
        
        // Simple adaptation
        if (reward > 0) {
            adaptationRate *= 1.01f;
        } else {
            adaptationRate *= 0.99f;
        }
    }
}

void RobotController::updateDemonstrationControl(double deltaTime) {
    // Execute recorded pattern
    if (!currentRecordingName.empty() && savedPatterns.count(currentRecordingName)) {
        auto& pattern = savedPatterns[currentRecordingName];
        
        // Find appropriate command based on time
        double patternTime = 0.0;
        for (const auto& [time, input] : pattern) {
            if (time <= patternTime) {
                currentInput = input;
            }
            patternTime += deltaTime;
        }
    }
    
    // Record current actions if recording
    if (isRecording) {
        recordedInputs.push_back({simulationTime, currentInput});
    }
    
    updateManualControl(deltaTime);
}

void RobotController::setNavigationGoal(const NavigationGoal& goal) {
    currentGoal = goal;
    navigationPath.clear();
    currentPathIndex = 0;
}

void RobotController::clearNavigationPath() {
    navigationPath.clear();
    currentPathIndex = 0;
}

bool RobotController::hasReachedGoal() const {
    if (!hasNavigationGoal()) return true;
    
    vsg::vec3 currentPos = robot->getPosition();
    float distance = vsg::length(currentPos - currentGoal.position);
    
    return distance < currentGoal.tolerance;
}

void RobotController::planPath() {
    // Simple A* or RRT* path planning would go here
    // For now, use straight line with waypoints
    
    vsg::vec3 start = robot->getPosition();
    vsg::vec3 end = currentGoal.position;
    
    // Create waypoints
    int numWaypoints = 10;
    for (int i = 0; i <= numWaypoints; ++i) {
        float t = i / (float)numWaypoints;
        vsg::vec3 waypoint = start * (1.0f - t) + end * t;
        
        // Add height offset to account for terrain
        waypoint.y += 0.5f;
        
        navigationPath.push_back(waypoint);
    }
    
    if (pathSmoothingEnabled) {
        smoothPath();
    }
}

void RobotController::followPath(double deltaTime) {
    if (currentPathIndex >= navigationPath.size()) {
        navigationPath.clear();
        return;
    }
    
    vsg::vec3 currentPos = robot->getPosition();
    vsg::vec3 targetPos = navigationPath[currentPathIndex];
    
    // Check if reached current waypoint
    float distanceToWaypoint = vsg::length(targetPos - currentPos);
    if (distanceToWaypoint < 0.5f) {
        currentPathIndex++;
        if (currentPathIndex >= navigationPath.size()) {
            return;
        }
        targetPos = navigationPath[currentPathIndex];
    }
    
    // Calculate direction to target
    vsg::vec3 direction = vsg::normalize(targetPos - currentPos);
    
    // Set control input
    currentInput.targetVelocity = direction * currentGoal.speed;
    
    // Calculate turn angle
    float targetAngle = atan2(direction.z, direction.x);
    vsg::quat currentOrientation = robot->getOrientation();
    vsg::vec3 euler = eulerFromQuat(currentOrientation);
    float angleDiff = targetAngle - euler.y;
    
    // Normalize angle difference
    while (angleDiff > M_PI) angleDiff -= 2 * M_PI;
    while (angleDiff < -M_PI) angleDiff += 2 * M_PI;
    
    currentInput.targetAngularVelocity = angleDiff * 2.0f;
    
    // Apply control
    updateManualControl(deltaTime);
}

void RobotController::avoidObstacles() {
    if (vsg::length(obstacleDirection) > 0.1f && obstacleDistance < 3.0f) {
        // Simple obstacle avoidance
        vsg::vec3 avoidanceDirection = vsg::vec3(-obstacleDirection.z, 0, obstacleDirection.x);
        
        // Blend avoidance with current velocity
        float avoidanceStrength = 1.0f - (obstacleDistance / 3.0f);
        currentInput.targetVelocity = vsg::normalize(
            currentInput.targetVelocity * (1.0f - avoidanceStrength) + 
            avoidanceDirection * avoidanceStrength
        ) * vsg::length(currentInput.targetVelocity);
    }
}

void RobotController::maintainBalance() {
    vsg::quat orientation = robot->getOrientation();
    vsg::vec3 euler = eulerFromQuat(orientation);
    
    // PID control for stability
    float pitchCorrection = pitchController.update(-euler.x, 0.016f);
    float rollCorrection = rollController.update(-euler.z, 0.016f);
    
    // Apply corrections through motor commands
    std::vector<float> stabilizationCommands;
    stabilizationCommands.push_back(pitchCorrection);
    stabilizationCommands.push_back(rollCorrection);
    
    // Blend with current commands
    robot->applyControl(stabilizationCommands);
}

void RobotController::optimizeGait() {
    // Adjust gait parameters based on speed and terrain
    float speed = vsg::length(robot->getVelocity());
    
    if (speed < 0.5f) {
        currentInput.gaitType = 0; // Walk
    } else if (speed < 2.0f) {
        currentInput.gaitType = 1; // Trot
    } else {
        currentInput.gaitType = 2; // Gallop
    }
    
    // Adjust based on stability
    if (stabilityScore < 0.7f) {
        currentInput.gaitType = 0; // Safer gait
    }
}

void RobotController::adaptToTerrain() {
    // Analyze sensor data for terrain type
    // Adjust control parameters accordingly
    
    // Simple terrain detection based on foot contact patterns
    auto sensorData = robot->getSensorReadings();
    
    // Count ground contacts
    int groundContacts = 0;
    for (size_t i = sensorData.size() - 6; i < sensorData.size(); ++i) {
        if (sensorData[i] > 0.5f) groundContacts++;
    }
    
    // Adjust aggressiveness based on terrain difficulty
    if (groundContacts < 3) {
        // Difficult terrain - be more careful
        aggressiveness = std::max(0.2f, aggressiveness - 0.01f);
    } else if (groundContacts >= 5) {
        // Easy terrain - can be more aggressive
        aggressiveness = std::min(0.8f, aggressiveness + 0.01f);
    }
}

void RobotController::processSensorData(const std::vector<float>& sensorReadings) {
    // Clear previous obstacles
    detectedObstacles.clear();
    
    // Reset obstacle direction
    obstacleDirection = vsg_vec3();
    obstacleDistance = std::numeric_limits<float>::max();
    
    // Simple obstacle detection based on proximity sensors
    if (sensorReadings.size() >= 6) {
        for (size_t i = 0; i < 6; ++i) {
            if (sensorReadings[i] < 2.0f) { // Obstacle detected within 2 units
                float angle = (float)i * M_PI / 3.0f; // 60-degree intervals
                obstacleDirection = vsg_vec3(cos(angle), 0, sin(angle));
                obstacleDistance = sensorReadings[i];
                
                // Add to detected obstacles list
                detectedObstacles.push_back(obstacleDirection * sensorReadings[i]);
            }
        }
    }
}

bool RobotController::isBalanced() const {
    return robot->isStable() && stabilityScore > 0.8f;
}

bool RobotController::isMoving() const {
    return vsg::length(robot->getVelocity()) > 0.1f;
}

void RobotController::startLearning() {
    isLearning = true;
    trainingData.clear();
    std::cout << "Started learning mode" << std::endl;
}

void RobotController::stopLearning() {
    isLearning = false;
    std::cout << "Stopped learning mode. Collected " << trainingData.size() << " samples." << std::endl;
}

void RobotController::saveLearnedBehavior(const std::string& filename) {
    std::ofstream file(filename, std::ios::binary);
    if (file.is_open()) {
        // Save training data
        size_t dataSize = trainingData.size();
        file.write(reinterpret_cast<const char*>(&dataSize), sizeof(dataSize));
        
        for (const auto& [sensors, actions] : trainingData) {
            size_t sensorSize = sensors.size();
            size_t actionSize = actions.size();
            
            file.write(reinterpret_cast<const char*>(&sensorSize), sizeof(sensorSize));
            file.write(reinterpret_cast<const char*>(sensors.data()), sensorSize * sizeof(float));
            
            file.write(reinterpret_cast<const char*>(&actionSize), sizeof(actionSize));
            file.write(reinterpret_cast<const char*>(actions.data()), actionSize * sizeof(float));
        }
        
        file.close();
        std::cout << "Saved learned behavior to " << filename << std::endl;
    }
}

void RobotController::loadLearnedBehavior(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    if (file.is_open()) {
        trainingData.clear();
        
        size_t dataSize;
        file.read(reinterpret_cast<char*>(&dataSize), sizeof(dataSize));
        
        for (size_t i = 0; i < dataSize; ++i) {
            size_t sensorSize, actionSize;
            
            file.read(reinterpret_cast<char*>(&sensorSize), sizeof(sensorSize));
            std::vector<float> sensors(sensorSize);
            file.read(reinterpret_cast<char*>(sensors.data()), sensorSize * sizeof(float));
            
            file.read(reinterpret_cast<char*>(&actionSize), sizeof(actionSize));
            std::vector<float> actions(actionSize);
            file.read(reinterpret_cast<char*>(actions.data()), actionSize * sizeof(float));
            
            trainingData.push_back({sensors, actions});
        }
        
        file.close();
        std::cout << "Loaded learned behavior from " << filename << std::endl;
    }
}

void RobotController::executePattern(const std::string& patternName) {
    if (savedPatterns.count(patternName)) {
        currentRecordingName = patternName;
        controlMode = DEMONSTRATION;
        std::cout << "Executing pattern: " << patternName << std::endl;
    }
}

void RobotController::recordPattern(const std::string& patternName) {
    isRecording = true;
    currentRecordingName = patternName;
    recordedInputs.clear();
    std::cout << "Recording pattern: " << patternName << std::endl;
}

void RobotController::stopRecording() {
    if (isRecording) {
        isRecording = false;
        savedPatterns[currentRecordingName] = recordedInputs;
        std::cout << "Saved pattern: " << currentRecordingName << " with " 
                  << recordedInputs.size() << " frames" << std::endl;
    }
}

void RobotController::smoothPath() {
    if (navigationPath.size() < 3) return;
    
    std::vector<vsg::vec3> smoothedPath;
    smoothedPath.push_back(navigationPath[0]);
    
    for (size_t i = 1; i < navigationPath.size() - 1; ++i) {
        vsg::vec3 prev = navigationPath[i - 1];
        vsg::vec3 curr = navigationPath[i];
        vsg::vec3 next = navigationPath[i + 1];
        
        vsg::vec3 smoothed = (prev + curr * 2.0f + next) * 0.25f;
        smoothedPath.push_back(smoothed);
    }
    
    smoothedPath.push_back(navigationPath.back());
    navigationPath = smoothedPath;
}

void RobotController::updatePerformanceMetrics(double deltaTime) {
    // Update stability score
    if (robot->isStable()) {
        stabilityScore = std::min(1.0f, stabilityScore + 0.01f);
    } else {
        stabilityScore = std::max(0.0f, stabilityScore - 0.05f);
    }
    
    // Update efficiency score
    float energyUsage = robot->getEnergyConsumption();
    float speed = vsg::length(robot->getVelocity());
    
    if (speed > 0.1f) {
        float efficiency = speed / (energyUsage + 1.0f);
        efficiencyScore = efficiencyScore * 0.95f + efficiency * 0.05f;
    }
    
    simulationTime += deltaTime;
}

float RobotController::evaluatePerformance() {
    float reward = 0.0f;
    
    // Reward for stability
    reward += stabilityScore * 10.0f;
    
    // Reward for efficiency
    reward += efficiencyScore * 5.0f;
    
    // Reward for progress toward goal
    if (hasNavigationGoal()) {
        float distanceToGoal = vsg::length(robot->getPosition() - currentGoal.position);
        reward += (100.0f - distanceToGoal) * 0.1f;
    }
    
    // Penalty for collisions
    if (obstacleDistance < 0.5f) {
        reward -= 10.0f;
    }
    
    return reward;
}

bool RobotController::hasNavigationGoal() const {
    return currentGoal.tolerance > 0.0f;
}

vsg::vec3 RobotController::eulerFromQuat(const vsg::quat& q) const {
    vsg::vec3 euler;
    
    // Roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    euler.x = std::atan2(sinr_cosp, cosr_cosp);
    
    // Pitch (y-axis rotation)  
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        euler.y = std::copysign(M_PI / 2, sinp);
    else
        euler.y = std::asin(sinp);
    
    // Yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    euler.z = std::atan2(siny_cosp, cosy_cosp);
    
    return euler;
}