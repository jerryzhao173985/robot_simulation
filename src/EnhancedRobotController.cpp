#include "EnhancedRobotController.h"
#include "Robot.h"
#include "PhysicsWorld.h"
#include "DebugOutput.h"
#include "PositionUtils.h"
#include <iostream>
#include <algorithm>
#include <cmath>

// ============================================================================
// ENHANCED ROBOT CONTROLLER IMPLEMENTATION
// ============================================================================

EnhancedRobotController::EnhancedRobotController(Robot* robot, PhysicsWorld* physicsWorld)
    : RobotController(robot, physicsWorld)
    , currentPathIndex_(0)
    , lastUpdateTime_(0) {
    
    // Initialize all control algorithm instances
    stateMachine_ = std::make_unique<ControlAlgorithms::StateMachine<ControlState, ControlEvent>>(ControlState::IDLE);
    setupStateMachine();
    
    // Initialize leg kinematics for each leg
    initializeLegKinematics();
    
    // Initialize gait generator
    ControlAlgorithms::GaitGenerator::GaitParams gaitParams;
    gaitParams.type = ControlAlgorithms::GaitGenerator::TRIPOD_GAIT;
    gaitParams.stepHeight = 0.1f;
    gaitParams.strideLength = 0.3f;
    gaitParams.cyclePeriod = 2.0f;
    gaitParams.dutyCycle = 0.6f;
    gaitParams.bodyVelocity = vsg_vec3(0, 0, 0);
    gaitParams.angularVelocity = 0;
    
    gaitGenerator_ = std::make_unique<ControlAlgorithms::GaitGenerator>(gaitParams);
    
    // Initialize terrain adapter
    ControlAlgorithms::TerrainAdapter::AdaptationParams terrainParams;
    terrainParams.minGroundClearance = 0.05f;
    terrainParams.maxStepHeight = 0.2f;
    terrainParams.stabilityThreshold = 0.3f;  // 30 degrees
    terrainParams.adaptationRate = 0.1f;
    terrainParams.enableProbing = true;
    
    terrainAdapter_ = std::make_unique<ControlAlgorithms::TerrainAdapter>(terrainParams);
    
    // Initialize path planner
    ControlAlgorithms::PathPlanner::PlanningParams planningParams;
    planningParams.stepSize = 0.1f;
    planningParams.goalTolerance = 0.2f;
    planningParams.maxPlanningTime = 5.0f;
    planningParams.maxNodes = 10000;
    planningParams.optimizePath = true;
    
    pathPlanner_ = std::make_unique<ControlAlgorithms::PathPlanner>(planningParams);
    
    // Initialize obstacle avoidance
    ControlAlgorithms::ObstacleAvoidance::VFHParams vfhParams;
    vfhParams.histogramSectors = 72;  // 5-degree sectors
    vfhParams.threshold = 0.5f;
    vfhParams.robotRadius = 0.3f;
    vfhParams.lookAheadDistance = 1.0f;
    vfhParams.safetyMargin = 0.2f;
    
    obstacleAvoidance_ = std::make_unique<ControlAlgorithms::ObstacleAvoidance>(vfhParams);
    
    // Initialize reinforcement learning controller
    ControlAlgorithms::RLController::RLParams rlParams;
    rlParams.learningRate = 0.01f;
    rlParams.discountFactor = 0.95f;
    rlParams.explorationRate = 0.1f;
    rlParams.explorationDecay = 0.995f;
    rlParams.stateSize = 20;  // Sensor inputs + robot state
    rlParams.actionSize = 18;  // 6 legs * 3 joints
    rlParams.useContinuous = false;
    
    rlController_ = std::make_unique<ControlAlgorithms::RLController>(rlParams);
    
    // Initialize joint controllers
    initializeControllers();
    
    // Load default parameters
    loadDefaultParameters();
    
    // Initialize foot positions
    updateFootPositions();
    
    std::cout << "Enhanced Robot Controller initialized with advanced algorithms" << std::endl;
}

EnhancedRobotController::~EnhancedRobotController() {
    // Cleanup is handled automatically by unique_ptr destructors
}

void EnhancedRobotController::setupStateMachine() {
    // Define state transitions
    stateMachine_->addTransition(ControlState::IDLE, ControlEvent::START_WALKING, ControlState::WALKING);
    stateMachine_->addTransition(ControlState::WALKING, ControlEvent::STOP_WALKING, ControlState::STANDING);
    stateMachine_->addTransition(ControlState::WALKING, ControlEvent::START_TURNING, ControlState::TURNING);
    stateMachine_->addTransition(ControlState::TURNING, ControlEvent::STOP_TURNING, ControlState::WALKING);
    stateMachine_->addTransition(ControlState::STANDING, ControlEvent::LOSE_BALANCE, ControlState::BALANCING);
    stateMachine_->addTransition(ControlState::BALANCING, ControlEvent::REGAIN_BALANCE, ControlState::STANDING);
    stateMachine_->addTransition(ControlState::WALKING, ControlEvent::OBSTACLE_DETECTED, ControlState::BALANCING);
    
    // Set entry actions
    stateMachine_->setEntryAction(ControlState::IDLE, [this]() { onEnterIdle(); });
    stateMachine_->setEntryAction(ControlState::STANDING, [this]() { onEnterStanding(); });
    stateMachine_->setEntryAction(ControlState::WALKING, [this]() { onEnterWalking(); });
    stateMachine_->setEntryAction(ControlState::TURNING, [this]() { onEnterTurning(); });
    stateMachine_->setEntryAction(ControlState::BALANCING, [this]() { onEnterBalancing(); });
    stateMachine_->setEntryAction(ControlState::CLIMBING, [this]() { onEnterClimbing(); });
    stateMachine_->setEntryAction(ControlState::LEARNING, [this]() { onEnterLearning(); });
    stateMachine_->setEntryAction(ControlState::ERROR_RECOVERY, [this]() { onEnterErrorRecovery(); });
    
    // Set state update actions
    stateMachine_->setStateAction(ControlState::IDLE, [this](float dt) { updateIdle(dt); });
    stateMachine_->setStateAction(ControlState::STANDING, [this](float dt) { updateStanding(dt); });
    stateMachine_->setStateAction(ControlState::WALKING, [this](float dt) { updateWalking(dt); });
    stateMachine_->setStateAction(ControlState::TURNING, [this](float dt) { updateTurning(dt); });
    stateMachine_->setStateAction(ControlState::BALANCING, [this](float dt) { updateBalancing(dt); });
    stateMachine_->setStateAction(ControlState::CLIMBING, [this](float dt) { updateClimbing(dt); });
    stateMachine_->setStateAction(ControlState::LEARNING, [this](float dt) { updateLearning(dt); });
    stateMachine_->setStateAction(ControlState::ERROR_RECOVERY, [this](float dt) { updateErrorRecovery(dt); });
}

void EnhancedRobotController::initializeLegKinematics() {
    // Robot leg parameters based on the existing robot configuration
    ControlAlgorithms::LegKinematics::LegParams legParams;
    legParams.coxaLength = 0.25f;
    legParams.femurLength = 0.35f;
    legParams.tibiaLength = 0.4f;
    
    // Leg attachment points (matching the robot's hexapod layout)
    std::array<vsg_vec3, 6> attachmentPoints = {
        makePosition(0.4f, 0.0f, 0.3f),   // Right front
        makePosition(0.0f, 0.0f, 0.4f),   // Right middle  
        makePosition(-0.4f, 0.0f, 0.3f),  // Right rear
        makePosition(-0.4f, 0.0f, -0.3f), // Left rear
        makePosition(0.0f, 0.0f, -0.4f),  // Left middle
        makePosition(0.4f, 0.0f, -0.3f)   // Left front
    };
    
    for (int i = 0; i < 6; ++i) {
        legParams.baseOffset = attachmentPoints[i];
        legKinematics_[i] = std::make_unique<ControlAlgorithms::LegKinematics>(legParams);
    }
}

void EnhancedRobotController::initializeControllers() {
    // Initialize joint PID controllers for each leg
    for (int leg = 0; leg < 6; ++leg) {
        for (int joint = 0; joint < 3; ++joint) {
            // Different gains for different joints
            if (joint == 0) {  // Coxa (hip) - needs stronger control
                jointControllers_[leg][joint] = std::make_unique<ControlAlgorithms::PIDController>(5.0f, 0.1f, 0.2f);
            } else if (joint == 1) {  // Femur (thigh) - medium control
                jointControllers_[leg][joint] = std::make_unique<ControlAlgorithms::PIDController>(3.0f, 0.05f, 0.1f);
            } else {  // Tibia (shin) - lighter control
                jointControllers_[leg][joint] = std::make_unique<ControlAlgorithms::PIDController>(2.0f, 0.02f, 0.05f);
            }
            
            jointControllers_[leg][joint]->setIntegralLimit(10.0f);
            jointControllers_[leg][joint]->setDerivativeFilter(0.1f);
        }
    }
    
    // Initialize body stability cascade controllers (Roll, Pitch, Yaw)
    bodyStabilityControllers_[0] = std::make_unique<ControlAlgorithms::CascadePIDController>(
        2.0f, 0.1f, 0.5f,  // Outer loop (position)
        10.0f, 0.5f, 1.0f  // Inner loop (velocity)
    );
    bodyStabilityControllers_[1] = std::make_unique<ControlAlgorithms::CascadePIDController>(
        2.0f, 0.1f, 0.5f,  // Outer loop (position)
        10.0f, 0.5f, 1.0f  // Inner loop (velocity)
    );
    bodyStabilityControllers_[2] = std::make_unique<ControlAlgorithms::CascadePIDController>(
        1.0f, 0.05f, 0.2f, // Outer loop (position) - yaw less aggressive
        5.0f, 0.2f, 0.5f   // Inner loop (velocity)
    );
}

void EnhancedRobotController::loadDefaultParameters() {
    enhancedParams_.useInverseKinematics = true;
    enhancedParams_.adaptiveGait = true;
    enhancedParams_.terrainAdaptation = true;
    enhancedParams_.dynamicStability = true;
    enhancedParams_.enablePathPlanning = true;
    enhancedParams_.planningStepSize = 0.1f;
    enhancedParams_.goalTolerance = 0.2f;
    enhancedParams_.enableLearning = false;
    enhancedParams_.learningRate = 0.01f;
    enhancedParams_.explorationRate = 0.1f;
    enhancedParams_.enableAdvancedGait = true;
    enhancedParams_.enablePredictiveControl = false;
}

void EnhancedRobotController::update(double deltaTime) {
    // Call base class update first
    RobotController::update(deltaTime);
    
    // Update enhanced control systems
    updateEnhancedControl(deltaTime);
    
    // Update state machine
    stateMachine_->update(static_cast<float>(deltaTime));
    
    // Update performance metrics
    updatePerformanceMetrics(deltaTime);
    
    // Check system health
    if (!checkSystemHealth()) {
        processControlEvent(ControlEvent::ERROR_OCCURRED);
    }
    
    lastUpdateTime_ = deltaTime;
}

void EnhancedRobotController::updateEnhancedControl(double deltaTime) {
    // Update foot positions from robot sensors
    updateFootPositions();
    
    // Update gait generator
    if (enhancedParams_.enableAdvancedGait) {
        updateGaitControl(deltaTime);
    }
    
    // Update terrain adaptation
    if (enhancedParams_.terrainAdaptation) {
        updateTerrainAdaptation(deltaTime);
    }
    
    // Update kinematics control
    if (enhancedParams_.useInverseKinematics) {
        updateKinematicsControl(deltaTime);
    }
    
    // Update advanced balance control
    if (enhancedParams_.dynamicStability) {
        updateAdvancedBalance(deltaTime);
    }
    
    // Update path following
    if (enhancedParams_.enablePathPlanning && !currentPath_.empty()) {
        updatePathFollowing(deltaTime);
    }
    
    // Update obstacle avoidance
    updateObstacleAvoidance(deltaTime);
    
    // Update learning system
    if (enhancedParams_.enableLearning) {
        updateLearningSystem(deltaTime);
    }
}

void EnhancedRobotController::updateKinematicsControl(double deltaTime) {
    std::vector<float> jointCommands(18, 0.0f);  // 6 legs * 3 joints
    
    // Get foot trajectories from gait generator
    auto footTrajectories = gaitGenerator_->getAllFootTrajectories();
    
    // Apply inverse kinematics for each leg
    for (int leg = 0; leg < 6; ++leg) {
        vsg_vec3 footTarget = footTrajectories[leg].position;
        
        // Apply terrain adaptation adjustments
        auto heightAdjustments = terrainAdapter_->getFootHeightAdjustments();
        footTarget.z += heightAdjustments[leg];
        
        // Store foot target for debugging
        footTargets_[leg] = footTarget;
        
        // Apply kinematics to compute joint commands
        applyKinematicsToJoints(leg, footTarget, jointCommands);
    }
    
    // Apply the computed commands
    robot->applyControl(jointCommands);
}

void EnhancedRobotController::applyKinematicsToJoints(int legIndex, const vsg_vec3& footTarget, std::vector<float>& commands) {
    if (legIndex < 0 || legIndex >= 6) return;
    
    // Compute inverse kinematics
    ControlAlgorithms::LegKinematics::JointAngles targetAngles;
    if (legKinematics_[legIndex]->inverseKinematics(footTarget, targetAngles)) {
        // Get current joint positions (would need actual sensor data)
        // For now, use simple position control
        
        int baseIndex = legIndex * 3;
        commands[baseIndex + 0] = targetAngles.coxa;
        commands[baseIndex + 1] = targetAngles.femur; 
        commands[baseIndex + 2] = targetAngles.tibia;
        
        // Apply PID control for smoother motion
        // This would require actual joint position feedback from sensors
        // For demonstration, we'll use the computed angles directly
    } else {
        // If inverse kinematics fails, use safe default positions
        int baseIndex = legIndex * 3;
        commands[baseIndex + 0] = 0.0f;  // Coxa neutral
        commands[baseIndex + 1] = -0.3f; // Femur down
        commands[baseIndex + 2] = 0.6f;  // Tibia up (to reach ground)
    }
}

void EnhancedRobotController::updateGaitControl(double deltaTime) {
    // Update gait generator
    gaitGenerator_->update(static_cast<float>(deltaTime));
    
    // Adapt gait based on current conditions
    if (enhancedParams_.adaptiveGait) {
        // Analyze current robot state and terrain
        bool shouldModifyGait = terrainAdapter_->shouldModifyGait();
        float recommendedSpeed = terrainAdapter_->getRecommendedSpeed();
        
        // Adjust gait parameters based on conditions
        ControlAlgorithms::GaitGenerator::GaitParams currentParams;
        
        if (shouldModifyGait) {
            // Switch to more stable gait
            currentParams.type = ControlAlgorithms::GaitGenerator::WAVE_GAIT;
            currentParams.stepHeight = 0.08f;  // Lower steps
            currentParams.cyclePeriod = 3.0f;  // Slower
        } else {
            // Use normal gait
            currentParams.type = ControlAlgorithms::GaitGenerator::TRIPOD_GAIT;
            currentParams.stepHeight = 0.1f;
            currentParams.cyclePeriod = 2.0f;
        }
        
        // Apply speed adjustment
        currentParams.cyclePeriod /= recommendedSpeed;
        
        gaitGenerator_->setGaitParams(currentParams);
    }
}

void EnhancedRobotController::updateTerrainAdaptation(double deltaTime) {
    // Update terrain adapter with current foot positions
    terrainAdapter_->update(currentFootPositions_, physicsWorld);
    
    // Get adapted body orientation
    vsg_vec3 adaptedOrientation = terrainAdapter_->getAdaptedBodyOrientation();
    
    // Apply body orientation adaptation (would integrate with body control)
    // This would modify the body pose based on terrain analysis
}

void EnhancedRobotController::updateAdvancedBalance(double deltaTime) {
    // Get current robot orientation and angular velocity
    vsg_quat currentOrientation = robot->getOrientation();
    vsg_vec3 currentEuler = ControlAlgorithms::Utils::eulerFromQuaternion(currentOrientation);
    
    // Target orientation (level by default, or adapted from terrain)
    vsg_vec3 targetEuler = terrainAdapter_->getAdaptedBodyOrientation();
    
    // Compute orientation errors
    float rollError = targetEuler.x - currentEuler.x;
    float pitchError = targetEuler.y - currentEuler.y; 
    float yawError = targetEuler.z - currentEuler.z;
    
    // Get angular velocities (would need actual sensor data)
    vsg_vec3 angularVel = vsg_vec3(0, 0, 0);  // Placeholder
    
    // Apply cascade control for body stability
    float rollCorrection = bodyStabilityControllers_[0]->update(0, rollError, angularVel.x, deltaTime);
    float pitchCorrection = bodyStabilityControllers_[1]->update(0, pitchError, angularVel.y, deltaTime);
    float yawCorrection = bodyStabilityControllers_[2]->update(0, yawError, angularVel.z, deltaTime);
    
    // Apply corrections through leg adjustments or direct body forces
    // This would integrate with the main control system
}

void EnhancedRobotController::updatePathFollowing(double deltaTime) {
    if (currentPath_.empty() || currentPathIndex_ >= currentPath_.size()) {
        return;
    }
    
    vsg_vec3 robotPos = robot->getPosition();
    vsg_vec3 targetPos = currentPath_[currentPathIndex_];
    
    float distanceToTarget = ControlAlgorithms::Utils::length(targetPos - robotPos);
    
    // Check if reached current waypoint
    if (distanceToTarget < enhancedParams_.goalTolerance) {
        currentPathIndex_++;
        if (currentPathIndex_ >= currentPath_.size()) {
            std::cout << "Path following completed!" << std::endl;
            return;
        }
        targetPos = currentPath_[currentPathIndex_];
    }
    
    // Update gait generator with target direction
    vsg_vec3 direction = ControlAlgorithms::Utils::normalize(targetPos - robotPos);
    
    ControlAlgorithms::GaitGenerator::GaitParams params;
    params.bodyVelocity = direction * 0.5f;  // Move toward target
    params.angularVelocity = 0;
    
    gaitGenerator_->setGaitParams(params);
}

void EnhancedRobotController::updateObstacleAvoidance(double deltaTime) {
    // Get detected obstacles from sensors
    std::vector<vsg_vec3> obstacles;
    
    // Process sensor data to extract obstacle positions
    auto sensorReadings = robot->getSensorReadings();
    // Convert sensor readings to obstacle positions (simplified)
    for (size_t i = 0; i < std::min(sensorReadings.size(), size_t(6)); ++i) {
        if (sensorReadings[i] < 2.0f) {  // Obstacle detected
            float angle = i * M_PI / 3.0f;
            vsg_vec3 obstaclePos = robot->getPosition() + 
                vsg_vec3(std::cos(angle) * sensorReadings[i], std::sin(angle) * sensorReadings[i], 0);
            obstacles.push_back(obstaclePos);
        }
    }
    
    if (!obstacles.empty()) {
        vsg_vec3 robotPos = robot->getPosition();
        vsg_vec3 goalDirection = vsg_vec3(1, 0, 0);  // Default forward direction
        
        // If following a path, use path direction
        if (!currentPath_.empty() && currentPathIndex_ < currentPath_.size()) {
            goalDirection = ControlAlgorithms::Utils::normalize(currentPath_[currentPathIndex_] - robotPos);
        }
        
        // Compute avoidance vector
        vsg_vec3 avoidanceVector = obstacleAvoidance_->computeAvoidanceVector(robotPos, goalDirection, obstacles);
        
        // Apply avoidance to gait parameters
        ControlAlgorithms::GaitGenerator::GaitParams params;
        params.bodyVelocity = avoidanceVector * 0.3f;  // Slower when avoiding
        gaitGenerator_->setGaitParams(params);
    }
}

void EnhancedRobotController::updateLearningSystem(double deltaTime) {
    if (!enhancedParams_.enableLearning) return;
    
    // Extract current state
    std::vector<float> currentState = extractStateVector();
    
    // Get action from RL controller
    auto action = rlController_->selectAction(currentState);
    
    // Apply learning action
    applyLearningAction(action.commands);
    
    // Compute reward based on performance
    float reward = computeLearningReward();
    
    // Update policy with experience
    // (This would store the previous state-action pair and update with current state and reward)
    rlController_->updatePolicy(currentState, action, reward, currentState);
}

void EnhancedRobotController::updateFootPositions() {
    // Update current foot positions based on forward kinematics
    // This would normally use actual joint sensor data
    for (int leg = 0; leg < 6; ++leg) {
        // Get current joint angles (placeholder - would use real sensor data)
        ControlAlgorithms::LegKinematics::JointAngles currentAngles;
        currentAngles.coxa = 0.0f;   // Would read from joint sensors
        currentAngles.femur = -0.3f;
        currentAngles.tibia = 0.6f;
        
        // Compute foot position using forward kinematics
        currentFootPositions_[leg] = legKinematics_[leg]->forwardKinematics(currentAngles);
    }
}

// State machine implementations
void EnhancedRobotController::onEnterIdle() {
    std::cout << "Robot entering IDLE state" << std::endl;
    // Reset all controllers
    for (int leg = 0; leg < 6; ++leg) {
        for (int joint = 0; joint < 3; ++joint) {
            jointControllers_[leg][joint]->reset();
        }
    }
}

void EnhancedRobotController::onEnterStanding() {
    std::cout << "Robot entering STANDING state" << std::endl;
    // Set standing pose targets
    for (int leg = 0; leg < 6; ++leg) {
        footTargets_[leg] = vsg_vec3(0, 0, -0.3f);  // Default standing height
    }
}

void EnhancedRobotController::onEnterWalking() {
    std::cout << "Robot entering WALKING state" << std::endl;
    // Start gait generator
    ControlAlgorithms::GaitGenerator::GaitParams walkParams;
    walkParams.type = ControlAlgorithms::GaitGenerator::TRIPOD_GAIT;
    walkParams.cyclePeriod = 2.0f;
    gaitGenerator_->setGaitParams(walkParams);
}

void EnhancedRobotController::onEnterTurning() {
    std::cout << "Robot entering TURNING state" << std::endl;
    // Set turning gait parameters
}

void EnhancedRobotController::onEnterBalancing() {
    std::cout << "Robot entering BALANCING state" << std::endl;
    // Reset balance controllers
    for (auto& controller : bodyStabilityControllers_) {
        controller->reset();
    }
}

void EnhancedRobotController::onEnterClimbing() {
    std::cout << "Robot entering CLIMBING state" << std::endl;
}

void EnhancedRobotController::onEnterLearning() {
    std::cout << "Robot entering LEARNING state" << std::endl;
}

void EnhancedRobotController::onEnterErrorRecovery() {
    std::cout << "Robot entering ERROR_RECOVERY state" << std::endl;
}

// State update implementations
void EnhancedRobotController::updateIdle(float dt) {
    // Minimal activity in idle state
}

void EnhancedRobotController::updateStanding(float dt) {
    // Maintain standing pose with active balance
}

void EnhancedRobotController::updateWalking(float dt) {
    // Walking control is handled in main update loop
}

void EnhancedRobotController::updateTurning(float dt) {
    // Turning control
}

void EnhancedRobotController::updateBalancing(float dt) {
    // Active balance recovery
}

void EnhancedRobotController::updateClimbing(float dt) {
    // Specialized climbing behavior
}

void EnhancedRobotController::updateLearning(float dt) {
    // Learning-specific updates
}

void EnhancedRobotController::updateErrorRecovery(float dt) {
    // Error recovery procedures
}

// Public interface methods
void EnhancedRobotController::setGaitType(ControlAlgorithms::GaitGenerator::GaitType type) {
    ControlAlgorithms::GaitGenerator::GaitParams params;
    params.type = type;
    gaitGenerator_->setGaitParams(params);
}

void EnhancedRobotController::setGaitParameters(float stepHeight, float strideLength, float cyclePeriod) {
    ControlAlgorithms::GaitGenerator::GaitParams params;
    params.stepHeight = stepHeight;
    params.strideLength = strideLength;
    params.cyclePeriod = cyclePeriod;
    gaitGenerator_->setGaitParams(params);
}

bool EnhancedRobotController::planPathToGoal(const vsg_vec3& goal) {
    vsg_vec3 start = robot->getPosition();
    currentPath_ = pathPlanner_->planPath(start, goal);
    currentPathIndex_ = 0;
    
    if (!currentPath_.empty()) {
        std::cout << "Path planned with " << currentPath_.size() << " waypoints" << std::endl;
        return true;
    }
    
    std::cout << "Path planning failed" << std::endl;
    return false;
}

void EnhancedRobotController::addObstacle(const vsg_vec3& position) {
    pathPlanner_->addObstacle(position);
}

void EnhancedRobotController::clearObstacles() {
    pathPlanner_->setObstacles({});
}

EnhancedRobotController::ControlState EnhancedRobotController::getCurrentControlState() const {
    return stateMachine_->getCurrentState();
}

void EnhancedRobotController::processControlEvent(ControlEvent event) {
    stateMachine_->processEvent(event);
}

std::vector<float> EnhancedRobotController::extractStateVector() const {
    std::vector<float> state;
    
    // Robot position and orientation
    vsg_vec3 pos = robot->getPosition();
    vsg_quat orient = robot->getOrientation();
    vsg_vec3 vel = robot->getVelocity();
    
    state.push_back(pos.x);
    state.push_back(pos.y);
    state.push_back(pos.z);
    state.push_back(orient.x);
    state.push_back(orient.y);
    state.push_back(orient.z);
    state.push_back(orient.w);
    state.push_back(vel.x);
    state.push_back(vel.y);
    state.push_back(vel.z);
    
    // Add sensor readings
    auto sensorReadings = robot->getSensorReadings();
    for (size_t i = 0; i < std::min(sensorReadings.size(), size_t(10)); ++i) {
        state.push_back(sensorReadings[i]);
    }
    
    return state;
}

void EnhancedRobotController::applyLearningAction(const std::vector<float>& action) {
    if (action.size() >= 18) {
        robot->applyControl(action);
    }
}

float EnhancedRobotController::computeLearningReward() const {
    float reward = 0;
    
    // Reward for stability
    if (robot->isStable()) {
        reward += 10.0f;
    }
    
    // Reward for forward motion
    vsg_vec3 velocity = robot->getVelocity();
    reward += velocity.x * 5.0f;  // Forward velocity
    
    // Penalty for excessive energy use
    float energy = robot->getEnergyConsumption();
    reward -= energy * 0.1f;
    
    return reward;
}

void EnhancedRobotController::updatePerformanceMetrics(double deltaTime) {
    // Update stability index
    currentMetrics_.stabilityIndex = robot->isStable() ? 1.0f : 0.0f;
    
    // Update energy efficiency
    float energy = robot->getEnergyConsumption();
    float speed = ControlAlgorithms::Utils::length(robot->getVelocity());
    currentMetrics_.energyEfficiency = speed / (energy + 1.0f);
    
    // Update path accuracy (if following path)
    if (!currentPath_.empty() && currentPathIndex_ < currentPath_.size()) {
        vsg_vec3 robotPos = robot->getPosition();
        vsg_vec3 targetPos = currentPath_[currentPathIndex_];
        float distance = ControlAlgorithms::Utils::length(targetPos - robotPos);
        currentMetrics_.pathAccuracy = 1.0f / (distance + 1.0f);
    }
    
    // Other metrics would be computed similarly
}

bool EnhancedRobotController::checkSystemHealth() {
    // Check for critical failures
    vsg_vec3 pos = robot->getPosition();
    
    // Check if robot has fallen
    if (pos.z < 0.1f) {
        return false;
    }
    
    // Check if robot is stable
    if (!robot->isStable()) {
        static int unstableCount = 0;
        unstableCount++;
        if (unstableCount > 100) {  // Unstable for too long
            return false;
        }
    }
    
    return true;
}

EnhancedRobotController::PerformanceMetrics EnhancedRobotController::getPerformanceMetrics() const {
    return currentMetrics_;
}

std::array<vsg_vec3, 6> EnhancedRobotController::getDebugFootTargets() const {
    return footTargets_;
}