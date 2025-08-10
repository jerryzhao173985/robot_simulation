#pragma once

#include "RobotController.h"
#include "ControlAlgorithms.h"
#include "FallbackTypes.h"
#include <memory>
#include <array>

#ifdef USE_OPENGL_FALLBACK
    using vsg_vec3 = vec3;
    using vsg_vec4 = vec4;
    using vsg_quat = quat;
#else
#include <vsg/all.h>
    using vsg_vec3 = vsg::vec3;
    using vsg_vec4 = vsg::vec4;
    using vsg_quat = vsg::quat;
#endif

class Robot;
class PhysicsWorld;

// Enhanced Robot Controller with comprehensive control algorithm integration
class EnhancedRobotController : public RobotController {
public:
    // Control states for the state machine
    enum class ControlState {
        IDLE,
        STANDING,
        WALKING,
        TURNING,
        BALANCING,
        CLIMBING,
        LEARNING,
        ERROR_RECOVERY
    };
    
    // Control events for state transitions
    enum class ControlEvent {
        START_WALKING,
        STOP_WALKING,
        START_TURNING,
        STOP_TURNING,
        LOSE_BALANCE,
        REGAIN_BALANCE,
        OBSTACLE_DETECTED,
        OBSTACLE_CLEARED,
        ERROR_OCCURRED,
        ERROR_RESOLVED
    };
    
    struct EnhancedParams {
        // Kinematics parameters
        bool useInverseKinematics = true;
        bool adaptiveGait = true;
        bool terrainAdaptation = true;
        bool dynamicStability = true;
        
        // Path planning parameters
        bool enablePathPlanning = true;
        float planningStepSize = 0.1f;
        float goalTolerance = 0.2f;
        
        // Learning parameters
        bool enableLearning = false;
        float learningRate = 0.01f;
        float explorationRate = 0.1f;
        
        // Performance optimization
        bool enableAdvancedGait = true;
        bool enablePredictiveControl = false;
    };
    
    EnhancedRobotController(Robot* robot, PhysicsWorld* physicsWorld);
    ~EnhancedRobotController();
    
    // Override base update method with enhanced algorithms
    void update(double deltaTime) override;
    
    // Enhanced control modes
    void setEnhancedParams(const EnhancedParams& params) { enhancedParams_ = params; }
    const EnhancedParams& getEnhancedParams() const { return enhancedParams_; }
    
    // Kinematics control
    void setFootPosition(int legIndex, const vsg_vec3& position);
    vsg_vec3 getFootPosition(int legIndex) const;
    bool isLegConfigurationValid(int legIndex, const vsg_vec3& footPos) const;
    
    // Gait control
    void setGaitType(ControlAlgorithms::GaitGenerator::GaitType type);
    void setGaitParameters(float stepHeight, float strideLength, float cyclePeriod);
    void setCustomGait(const std::array<float, 6>& phaseOffsets);
    
    // Advanced navigation
    bool planPathToGoal(const vsg_vec3& goal);
    void followPlannedPath();
    void addObstacle(const vsg_vec3& position);
    void clearObstacles();
    
    // Learning interface
    void enableAdvancedLearning(bool enable);
    void saveAdvancedBehavior(const std::string& filename);
    void loadAdvancedBehavior(const std::string& filename);
    
    // State machine interface
    ControlState getCurrentControlState() const;
    void processControlEvent(ControlEvent event);
    
    // Performance monitoring
    struct PerformanceMetrics {
        float stabilityIndex;
        float energyEfficiency;
        float pathAccuracy;
        float adaptationSpeed;
        float learningProgress;
    };
    PerformanceMetrics getPerformanceMetrics() const;
    
    // Debug and visualization
    void setDebugVisualization(bool enable) { debugVisualization_ = enable; }
    std::vector<vsg_vec3> getDebugPath() const { return currentPath_; }
    std::array<vsg_vec3, 6> getDebugFootTargets() const;

private:
    // Control algorithm instances
    std::unique_ptr<ControlAlgorithms::StateMachine<ControlState, ControlEvent>> stateMachine_;
    std::array<std::unique_ptr<ControlAlgorithms::LegKinematics>, 6> legKinematics_;
    std::unique_ptr<ControlAlgorithms::GaitGenerator> gaitGenerator_;
    std::unique_ptr<ControlAlgorithms::TerrainAdapter> terrainAdapter_;
    std::unique_ptr<ControlAlgorithms::PathPlanner> pathPlanner_;
    std::unique_ptr<ControlAlgorithms::ObstacleAvoidance> obstacleAvoidance_;
    std::unique_ptr<ControlAlgorithms::RLController> rlController_;
    
    // Enhanced PID controllers for each joint
    std::array<std::array<std::unique_ptr<ControlAlgorithms::PIDController>, 3>, 6> jointControllers_;
    
    // Cascade controllers for body stability
    std::array<std::unique_ptr<ControlAlgorithms::CascadePIDController>, 3> bodyStabilityControllers_;
    
    // Control state and parameters
    EnhancedParams enhancedParams_;
    std::vector<vsg_vec3> currentPath_;
    size_t currentPathIndex_;
    
    // Foot targets and trajectories
    std::array<vsg_vec3, 6> footTargets_;
    std::array<vsg_vec3, 6> currentFootPositions_;
    
    // Performance tracking
    PerformanceMetrics currentMetrics_;
    double lastUpdateTime_;
    
    // Debug and visualization
    bool debugVisualization_ = false;
    
    // State machine actions
    void setupStateMachine();
    void onEnterIdle();
    void onEnterStanding();
    void onEnterWalking();
    void onEnterTurning();
    void onEnterBalancing();
    void onEnterClimbing();
    void onEnterLearning();
    void onEnterErrorRecovery();
    
    void updateIdle(float dt);
    void updateStanding(float dt);
    void updateWalking(float dt);
    void updateTurning(float dt);
    void updateBalancing(float dt);
    void updateClimbing(float dt);
    void updateLearning(float dt);
    void updateErrorRecovery(float dt);
    
    // Enhanced control methods
    void updateEnhancedControl(double deltaTime);
    void updateKinematicsControl(double deltaTime);
    void updateGaitControl(double deltaTime);
    void updateTerrainAdaptation(double deltaTime);
    void updateAdvancedBalance(double deltaTime);
    void updatePathFollowing(double deltaTime);
    void updateObstacleAvoidance(double deltaTime);
    void updateLearningSystem(double deltaTime);
    
    // Joint control
    void computeJointCommands(std::vector<float>& commands);
    void applyKinematicsToJoints(int legIndex, const vsg_vec3& footTarget, std::vector<float>& commands);
    
    // Utility methods
    void updateFootPositions();
    void updatePerformanceMetrics(double deltaTime);
    bool checkSystemHealth();
    void handleEmergencyStop();
    
    // Configuration methods
    void initializeLegKinematics();
    void initializeControllers();
    void loadDefaultParameters();
    
    // Learning integration
    std::vector<float> extractStateVector() const;
    void applyLearningAction(const std::vector<float>& action);
    float computeLearningReward() const;
};

// Specialized controller for different robot behaviors
class SpecializedBehaviorController {
public:
    enum class BehaviorType {
        EXPLORATION,
        SEARCH_AND_RESCUE,
        SURVEILLANCE,
        TRANSPORTATION,
        INSPECTION
    };
    
    struct BehaviorParams {
        BehaviorType type;
        float priority;
        float timeLimit;
        std::map<std::string, float> customParams;
    };
    
    SpecializedBehaviorController(EnhancedRobotController* controller);
    
    void executeBehavior(const BehaviorParams& params);
    void stopCurrentBehavior();
    bool isBehaviorActive() const { return behaviorActive_; }
    
    BehaviorType getCurrentBehavior() const { return currentBehavior_.type; }
    float getBehaviorProgress() const;

private:
    EnhancedRobotController* controller_;
    BehaviorParams currentBehavior_;
    bool behaviorActive_;
    double behaviorStartTime_;
    
    void executeExploration(const BehaviorParams& params, double elapsed);
    void executeSearchAndRescue(const BehaviorParams& params, double elapsed);
    void executeSurveillance(const BehaviorParams& params, double elapsed);
    void executeTransportation(const BehaviorParams& params, double elapsed);
    void executeInspection(const BehaviorParams& params, double elapsed);
};

// Performance optimizer for continuous improvement
class PerformanceOptimizer {
public:
    struct OptimizationParams {
        float energyWeight = 0.3f;
        float stabilityWeight = 0.4f;
        float speedWeight = 0.3f;
        int optimizationPeriod = 1000; // Update frequency in control cycles
        bool enableRealTimeOptimization = true;
    };
    
    PerformanceOptimizer(EnhancedRobotController* controller);
    
    void setParams(const OptimizationParams& params) { params_ = params; }
    void update(double deltaTime);
    
    float getOptimizationScore() const { return currentScore_; }
    std::vector<float> getOptimalParameters() const { return optimalParams_; }

private:
    EnhancedRobotController* controller_;
    OptimizationParams params_;
    
    std::vector<float> currentParams_;
    std::vector<float> optimalParams_;
    float currentScore_;
    float bestScore_;
    
    int optimizationCounter_;
    std::vector<std::vector<float>> parameterHistory_;
    std::vector<float> scoreHistory_;
    
    void optimizeParameters();
    float evaluatePerformance();
    void updateParameterSearch();
};