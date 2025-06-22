#include "RobotController.h"
#include "Robot.h"
#include "DebugOutput.h"
#include "Sensor.h"
#include "NoiseManager.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <fstream>
#include "PositionUtils.h"

RobotController::RobotController(Robot* robot, PhysicsWorld* physicsWorld)
    : robot(robot)
    , physicsWorld(physicsWorld)
    , controlMode(MANUAL)
    , currentPathIndex(0) {
    
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
        maintainBalance(static_cast<float>(deltaTime));
    }
    
    // Update performance metrics
    updatePerformanceMetrics(deltaTime);
}

void RobotController::updateManualControl(double deltaTime) {
    // Enhanced debug logging with noise information
    static int debugLogCounter = 0;
    debugLogCounter++;
    
    // Log every second (60 frames) when noise is active
    float noiseLevel = NoiseManager::getInstance().getNoiseLevel();
    bool shouldLogDebug = (debugLogCounter == 1) || // First frame
                         (debugLogCounter % 60 == 0) || // Every second
                         (noiseLevel > 0.0f && debugLogCounter % 30 == 0); // Every 0.5s with noise
    
    if (shouldLogDebug) {
        // Compact debug output - only essential info
        int contactCount = 0;
        int stableContactCount = filteredSensorData.getStableContactCount();
        
        // Count contacts silently
        for (int i = 0; i < 6; ++i) {
            std::string sensorName = "foot_contact_" + std::to_string(i);
            if (auto* contact = robot->getSensor(sensorName)) {
                auto reading = contact->getReading();
                if (reading.valid && reading.values.size() > 0 && reading.values[0] > 0.5f) {
                    contactCount++;
                }
            }
        }
        
        // Get robot position for monitoring sinking
        vsg_vec3 robotPos = robot->getPosition();
        
        // Only show critical info
        std::cout << "[" << std::fixed << std::setprecision(1) << simulationTime << "s] "
                  << "Contacts: " << contactCount << "/6, "
                  << "Height: " << std::setprecision(2) << robotPos.z << "m";
        
        if (noiseLevel > 0.0f) {
            std::cout << ", Noise: " << std::setprecision(1) << noiseLevel;
        }
        
        // Warn if robot is sinking
        if (robotPos.z < 0.3f) {
            std::cout << " [WARNING: SINKING!]";
        }
        
        std::cout << std::endl;
    }
    
    // Apply manual control from keyboard input
    if (manualVelocity.x != 0.0f || manualVelocity.y != 0.0f || manualVelocity.z != 0.0f || manualRotation != 0.0f) {
        // Convert world-space velocity to robot-local space
        vsg_quat robotOrient = robot->getOrientation();
        vsg_vec3 forward = robotOrient * vsg_vec3(1, 0, 0);
        vsg_vec3 right = robotOrient * vsg_vec3(0, 1, 0);
        vsg_vec3 up = vsg_vec3(0, 0, 1);
        
        // Calculate target velocity in world space
        vsg_vec3 targetVel = forward * manualVelocity.x + right * manualVelocity.y + up * manualVelocity.z;
        
        // Robot velocity is controlled through motor commands, not direct velocity setting
        // The targetVel and manualRotation are used to generate appropriate motor commands below
        
        // Generate motor commands for hexapod gait
        std::vector<float> motorCommands(18, 0.0f); // 6 legs * 3 joints
        
        // If noise is enabled but robot is not moving, set all commands to zero
        // This allows the noise in actuators to be visible
        if (NoiseManager::getInstance().getNoiseLevel() > 0.0f && 
            std::abs(manualVelocity.x) < 0.01f && 
            std::abs(manualVelocity.y) < 0.01f && 
            std::abs(manualRotation) < 0.01f) {
            // Let noise do the work - commands stay at zero
            if (shouldLogDebug) {
                std::cout << "\nNoise-only mode: All motor commands set to 0 to show pure noise effect" << std::endl;
            }
            robot->applyControl(motorCommands);
            return;
        }
        
        // Tripod gait with proper turning control
        float gaitSpeed = 3.0f; // Gait cycle frequency
        float phase = fmod(simulationTime * gaitSpeed, 2.0 * M_PI);
        
        // Calculate stride length based on velocity
        float strideLength = manualVelocity.x * 0.8f;
        float turnRadius = (manualRotation != 0.0f) ? 2.0f / std::abs(manualRotation) : 1000.0f;
        
        for (int leg = 0; leg < 6; ++leg) {
            // Tripod groups: legs 0,2,4 vs 1,3,5
            bool isGroupA = (leg % 2 == 0);
            float legPhase = isGroupA ? phase : phase + M_PI;
            
            // Determine which side of robot (left=0,1,2  right=3,4,5)
            bool isLeftSide = (leg < 3);
            int legPair = leg % 3; // Front=0, Middle=1, Rear=2
            
            // Calculate differential stride for turning
            float turnFactor = 1.0f;
            if (manualRotation != 0.0f) {
                // Inner legs move slower, outer legs move faster during turn
                if ((manualRotation > 0 && isLeftSide) || (manualRotation < 0 && !isLeftSide)) {
                    turnFactor = 1.0f - std::abs(manualRotation) * 0.5f; // Inner side
                } else {
                    turnFactor = 1.0f + std::abs(manualRotation) * 0.5f; // Outer side
                }
            }
            
            // Hip joint - horizontal leg swing for forward motion and turning
            float hipSwing = strideLength * turnFactor * sin(legPhase);
            
            // Add rotation component for turning
            if (manualRotation != 0.0f) {
                float rotComponent = manualRotation * (isLeftSide ? -1.0f : 1.0f) * 0.5f;
                hipSwing += rotComponent * cos(legPhase);
            }
            
            motorCommands[leg * 3 + 0] = hipSwing;
            
            // Knee joint - vertical lift during swing phase
            bool inSwingPhase = sin(legPhase) > 0;
            if (inSwingPhase) {
                // Lift leg during swing
                motorCommands[leg * 3 + 1] = 0.4f + 0.2f * sin(legPhase);
            } else {
                // Push down during stance
                motorCommands[leg * 3 + 1] = -0.2f;
            }
            
            // Ankle joint - maintain foot parallel to ground
            motorCommands[leg * 3 + 2] = -motorCommands[leg * 3 + 1] * 0.7f;
        }
        
        robot->applyControl(motorCommands);
    } else {
        // No input - maintain stance
        // Zero velocity is achieved by not applying motor commands
        std::vector<float> zeroCommands(18, 0.0f);
        robot->applyControl(zeroCommands);
    }
    
    // Always maintain balance
    maintainBalance(deltaTime);
}

void RobotController::updateAutonomousControl(double deltaTime) {
    // Navigate to goal
    if (!navigationPath.empty()) {
        followPath(deltaTime);
    } else if (hasNavigationGoal()) {
        planPath();
    } else {
        // No goal - maintain stance
        std::vector<float> zeroCommands(18, 0.0f);
        robot->applyControl(zeroCommands);
    }
    
    // Avoid obstacles
    if (obstacleAvoidanceEnabled && !navigationPath.empty()) {
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

void RobotController::maintainBalance(float deltaTime) {
    if (!dynamicStabilityEnabled || !physicsWorld) return;

    // Update filtered sensor data from IMU
    if (auto* imu = robot->getSensor("body_imu")) {
        auto reading = imu->getReading();
        if (reading.valid && reading.values.size() >= 10) {
            vsg_quat orientation(reading.values[0], reading.values[1], 
                               reading.values[2], reading.values[3]);
            vsg_vec3 angularVel(reading.values[4], reading.values[5], reading.values[6]);
            vsg_vec3 linearAccel(reading.values[7], reading.values[8], reading.values[9]);
            
            filteredSensorData.updateIMU(orientation, angularVel, linearAccel);
        }
    }
    
    // Update contact sensor history
    std::array<bool, 6> currentContacts{};
    int contactIndex = 0;
    for (const auto& name : robot->getSensorNames()) {
        if (name.find("foot") != std::string::npos && name.find("contact") != std::string::npos) {
            if (auto* sensor = robot->getSensor(name)) {
                auto reading = sensor->getReading();
                if (reading.valid && reading.values.size() > 0 && contactIndex < 6) {
                    currentContacts[contactIndex] = reading.values[0] > 0.5f;
                    contactIndex++;
                }
            }
        }
    }
    filteredSensorData.updateContacts(currentContacts);
    
    // Use filtered orientation for PID control
    vsg::vec3 euler = eulerFromQuat(filteredSensorData.filteredOrientation);
    
    // Adjust PID gains based on noise level
    float noiseLevel = NoiseManager::getInstance().getNoiseLevel();
    float gainReduction = 1.0f - (noiseLevel * 0.5f); // Reduce gains by up to 50% at max noise
    
    // Apply gain reduction to PID controllers
    float adjustedPitchKp = pitchController.kp * gainReduction;
    float adjustedRollKp = rollController.kp * gainReduction;
    
    // Calculate corrections with adjusted gains
    float pitchError = -euler.x;
    float rollError = -euler.z;
    float pitchCorr = adjustedPitchKp * pitchError + pitchController.ki * pitchController.integral + 
                      pitchController.kd * (pitchError - pitchController.previousError) / deltaTime;
    float rollCorr = adjustedRollKp * rollError + rollController.ki * rollController.integral + 
                     rollController.kd * (rollError - rollController.previousError) / deltaTime;
    
    // Update PID state
    pitchController.previousError = pitchError;
    rollController.previousError = rollError;
    pitchController.integral += pitchError * deltaTime;
    rollController.integral += rollError * deltaTime;

    // Scale corrective forces for body weight support
    float balanceForceScale = 50.0f * gainReduction; // Also reduce force scale with noise

    // Only apply forces if we have stable ground contact (3+ feet)
    int stableContacts = filteredSensorData.getStableContactCount();
    if (stableContacts >= 3) {
        // Apply contact-based support forces on each foot
        auto footGeoms = robot->getFootGeoms();
        int footIndex = 0;
        for (auto footGeom : footGeoms) {
            if (footIndex < 6 && filteredSensorData.isFootStableContact(footIndex)) {
                auto contacts = physicsWorld->getContactPoints(footGeom);
                for (auto& cp : contacts) {
                    float forceGain = (pitchCorr + rollCorr) * balanceForceScale;
                    dBodyAddForceAtPos(robot->getBody(),
                                      cp.normal.x * forceGain,
                                      cp.normal.y * forceGain,
                                      cp.normal.z * forceGain,
                                      cp.position.x, cp.position.y, cp.position.z);
                }
            }
            footIndex++;
        }
    }

    // Enhanced debug logging for balance control
    static int dbgCounter = 0;
    if (++dbgCounter % 60 == 0 && noiseLevel > 0.0f) {
        std::cout << "\n--- BALANCE CONTROL DEBUG ---" << std::endl;
        std::cout << "Euler Angles: roll=" << std::fixed << std::setprecision(1) 
                  << euler.x * 180.0f/M_PI << "°, pitch=" << euler.y * 180.0f/M_PI << "°" << std::endl;
        std::cout << "PID Errors: pitch=" << pitchError << ", roll=" << rollError << std::endl;
        std::cout << "PID Corrections: pitch=" << pitchCorr << ", roll=" << rollCorr << std::endl;
        std::cout << "Stable Contacts: " << stableContacts << "/6 (minimum 3 required)" << std::endl;
        std::cout << "Gain Reduction: " << (1.0f - gainReduction) * 100 << "% (due to noise)" << std::endl;
        std::cout << "Balance Force Scale: " << balanceForceScale << " (original: 50.0)" << std::endl;
        
        if (stableContacts < 3) {
            std::cout << "WARNING: Not enough stable contacts for balance control!" << std::endl;
        }
    }
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
    
    // Terrain adaptation: use body-ground contact count to gauge support stability
    if (!physicsWorld) return;
    dGeomID bodyGeom = robot->getBodyGeom();
    auto contacts = physicsWorld->getContactPoints(bodyGeom);
    int numContacts = static_cast<int>(contacts.size());
    // Few contacts -> uneven support, reduce aggressiveness
    if (numContacts < 2) {
        aggressiveness = std::max(0.2f, aggressiveness - 0.01f);
    }
    // Many contacts -> stable support, can increase aggressiveness
    else if (numContacts >= 4) {
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
                if (sensorReadings[i] < 2.0f) {
                    if (sensorReadings[i] < obstacleDistance) {
                        float angle = i * M_PI / 3.0f;
                        obstacleDirection = makePosition(std::cos(angle), 0.0f, std::sin(angle));
                        obstacleDistance = sensorReadings[i];
                    }
                    detectedObstacles.push_back(obstacleDirection * sensorReadings[i]);
                }
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

RobotController::SensorStatus RobotController::getSensorStatus() const {
    SensorStatus status;
    
    // Get IMU data
    if (auto* imu = robot->getSensor("body_imu")) {
        auto reading = imu->getReading();
        if (reading.valid && reading.values.size() >= 10) {
            status.angularVelocity = vsg::vec3(reading.values[4], reading.values[5], reading.values[6]);
            status.imuValid = true;
        }
    }
    
    // Count foot contacts
    for (int i = 0; i < 6; ++i) {
        std::string sensorName = "foot_contact_" + std::to_string(i);
        if (auto* contact = robot->getSensor(sensorName)) {
            auto reading = contact->getReading();
            if (reading.valid && reading.values.size() > 0 && reading.values[0] > 0.5f) {
                status.footContacts++;
            }
        }
    }
    
    // Calculate average joint velocity
    float totalVel = 0.0f;
    int jointCount = 0;
    for (const auto& name : robot->getSensorNames()) {
        if (name.find("_vel") != std::string::npos) {
            if (auto* sensor = robot->getSensor(name)) {
                auto reading = sensor->getReading();
                if (reading.valid && reading.values.size() > 0) {
                    totalVel += std::abs(reading.values[0]);
                    jointCount++;
                }
            }
        }
    }
    
    if (jointCount > 0) {
        status.averageJointVelocity = totalVel / jointCount;
    }
    
    return status;
}

// FilteredSensorData implementation
void RobotController::FilteredSensorData::updateIMU(const vsg_quat& newOrientation, 
                                                   const vsg_vec3& newAngularVel, 
                                                   const vsg_vec3& newLinearAccel) {
    // Apply exponential moving average filter
    // For angular velocity
    filteredAngularVel.x = angularVelAlpha * newAngularVel.x + (1.0f - angularVelAlpha) * filteredAngularVel.x;
    filteredAngularVel.y = angularVelAlpha * newAngularVel.y + (1.0f - angularVelAlpha) * filteredAngularVel.y;
    filteredAngularVel.z = angularVelAlpha * newAngularVel.z + (1.0f - angularVelAlpha) * filteredAngularVel.z;
    
    // For orientation (using spherical linear interpolation for quaternions)
    // Simple approximation for small changes
    filteredOrientation.x = orientationAlpha * newOrientation.x + (1.0f - orientationAlpha) * filteredOrientation.x;
    filteredOrientation.y = orientationAlpha * newOrientation.y + (1.0f - orientationAlpha) * filteredOrientation.y;
    filteredOrientation.z = orientationAlpha * newOrientation.z + (1.0f - orientationAlpha) * filteredOrientation.z;
    filteredOrientation.w = orientationAlpha * newOrientation.w + (1.0f - orientationAlpha) * filteredOrientation.w;
    
    // Normalize quaternion
    float mag = std::sqrt(filteredOrientation.x * filteredOrientation.x + 
                         filteredOrientation.y * filteredOrientation.y + 
                         filteredOrientation.z * filteredOrientation.z + 
                         filteredOrientation.w * filteredOrientation.w);
    if (mag > 0.0f) {
        filteredOrientation.x /= mag;
        filteredOrientation.y /= mag;
        filteredOrientation.z /= mag;
        filteredOrientation.w /= mag;
    }
    
    // For linear acceleration
    filteredLinearAccel.x = linearAccelAlpha * newLinearAccel.x + (1.0f - linearAccelAlpha) * filteredLinearAccel.x;
    filteredLinearAccel.y = linearAccelAlpha * newLinearAccel.y + (1.0f - linearAccelAlpha) * filteredLinearAccel.y;
    filteredLinearAccel.z = linearAccelAlpha * newLinearAccel.z + (1.0f - linearAccelAlpha) * filteredLinearAccel.z;
}

void RobotController::FilteredSensorData::updateContacts(const std::array<bool, 6>& contacts) {
    // Store in circular buffer
    contactHistory[contactHistoryIndex] = contacts;
    contactHistoryIndex = (contactHistoryIndex + 1) % CONTACT_HISTORY_SIZE;
}

int RobotController::FilteredSensorData::getStableContactCount() const {
    int stableCount = 0;
    
    // For each foot, check if it has been in contact for majority of history
    for (int foot = 0; foot < 6; ++foot) {
        if (isFootStableContact(foot)) {
            stableCount++;
        }
    }
    
    return stableCount;
}

bool RobotController::FilteredSensorData::isFootStableContact(int footIndex) const {
    if (footIndex < 0 || footIndex >= 6) return false;
    
    // Count how many times this foot has been in contact in history
    int contactCount = 0;
    for (int i = 0; i < CONTACT_HISTORY_SIZE; ++i) {
        if (contactHistory[i][footIndex]) {
            contactCount++;
        }
    }
    
    // Require at least 3 out of 5 frames for stable contact
    return contactCount >= 3;
}