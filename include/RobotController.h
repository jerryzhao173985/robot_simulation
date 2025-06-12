#pragma once

#include <vsg/all.h>
#include <vector>
#include <memory>
#include <functional>
#include <map>

class Robot;

class RobotController {
public:
    enum ControlMode {
        MANUAL,
        AUTONOMOUS,
        LEARNING,
        DEMONSTRATION
    };

    struct ControlInput {
        vsg::vec3 targetVelocity;
        float targetAngularVelocity;
        bool jump;
        bool crouch;
        int gaitType; // 0: walk, 1: trot, 2: gallop
    };

    struct NavigationGoal {
        vsg::vec3 position;
        vsg::quat orientation;
        float speed;
        float tolerance;
    };

    RobotController(Robot* robot);
    ~RobotController();

    void update(double deltaTime);
    void setControlMode(ControlMode mode) { controlMode = mode; }
    void setControlInput(const ControlInput& input) { currentInput = input; }
    
    // Navigation
    void setNavigationGoal(const NavigationGoal& goal);
    void clearNavigationPath();
    bool hasReachedGoal() const;
    
    // Advanced control features
    void enableObstacleAvoidance(bool enable) { obstacleAvoidanceEnabled = enable; }
    void enablePathSmoothing(bool enable) { pathSmoothingEnabled = enable; }
    void enableDynamicStability(bool enable) { dynamicStabilityEnabled = enable; }
    void setAggressiveness(float value) { aggressiveness = std::clamp(value, 0.0f, 1.0f); }
    
    // Learning and adaptation
    void startLearning();
    void stopLearning();
    void saveLearnedBehavior(const std::string& filename);
    void loadLearnedBehavior(const std::string& filename);
    
    // Behavior patterns
    void executePattern(const std::string& patternName);
    void recordPattern(const std::string& patternName);
    void stopRecording();
    
    // Sensor processing
    void processSensorData(const std::vector<float>& sensorReadings);
    vsg::vec3 getPerceivedObstacleDirection() const { return obstacleDirection; }
    
    // State queries
    bool isBalanced() const;
    bool isMoving() const;
    float getStability() const { return stabilityScore; }
    float getEfficiency() const { return efficiencyScore; }

private:
    void updateManualControl(double deltaTime);
    void updateAutonomousControl(double deltaTime);
    void updateLearningControl(double deltaTime);
    void updateDemonstrationControl(double deltaTime);
    
    void planPath();
    void followPath(double deltaTime);
    void avoidObstacles();
    void maintainBalance();
    void optimizeGait();
    void adaptToTerrain();
    void smoothPath();
    void updatePerformanceMetrics(double deltaTime);
    float evaluatePerformance();
    bool hasNavigationGoal() const;
    vsg::vec3 eulerFromQuat(const vsg::quat& q) const;
    
    // PID controllers for stability
    struct PIDController {
        float kp, ki, kd;
        float integral;
        float previousError;
        float update(float error, float dt);
    };
    
    Robot* robot;
    ControlMode controlMode;
    ControlInput currentInput;
    NavigationGoal currentGoal;
    
    // Path planning
    std::vector<vsg::vec3> navigationPath;
    size_t currentPathIndex;
    float pathFollowingSpeed;
    
    // Control parameters
    float aggressiveness = 0.5f;
    float reactionTime = 0.1f;
    float maxAcceleration = 5.0f;
    float maxAngularAcceleration = 3.0f;
    
    // Feature flags
    bool obstacleAvoidanceEnabled = true;
    bool pathSmoothingEnabled = true;
    bool dynamicStabilityEnabled = true;
    
    // Sensor processing
    vsg::vec3 obstacleDirection;
    float obstacleDistance;
    std::vector<vsg::vec3> detectedObstacles;
    
    // Stability control
    PIDController pitchController;
    PIDController rollController;
    PIDController yawController;
    PIDController heightController;
    
    // Performance metrics
    float stabilityScore = 1.0f;
    float efficiencyScore = 1.0f;
    float adaptationRate = 0.1f;
    double simulationTime = 0.0;
    
    // Learning system
    bool isLearning = false;
    std::vector<std::pair<std::vector<float>, std::vector<float>>> trainingData;
    
    // Behavior recording
    bool isRecording = false;
    std::string currentRecordingName;
    std::vector<std::pair<double, ControlInput>> recordedInputs;
    std::map<std::string, std::vector<std::pair<double, ControlInput>>> savedPatterns;
};