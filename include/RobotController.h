#pragma once

#include "FallbackTypes.h"

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

#include <vector>
#include <memory>
#include "PhysicsWorld.h"
#include <functional>
#include <map>
#include <algorithm>
#include <array>

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
        vsg_vec3 targetVelocity;
        float targetAngularVelocity;
        bool jump;
        bool crouch;
        int gaitType; // 0: walk, 1: trot, 2: gallop
    };

    struct NavigationGoal {
        vsg_vec3 position;
        vsg_quat orientation;
        float speed;
        float tolerance;
    };

    RobotController(Robot* robot, PhysicsWorld* physicsWorld);
    ~RobotController();

    void update(double deltaTime);
    void setControlMode(ControlMode mode) { controlMode = mode; }
    ControlMode getControlMode() const { return controlMode; }
    void setControlInput(const ControlInput& input) { currentInput = input; }
    
    // Manual control
    void setManualVelocity(const vsg_vec3& velocity) { manualVelocity = velocity; }
    void setManualRotation(float rotation) { manualRotation = rotation; }
    
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
    vsg_vec3 getPerceivedObstacleDirection() const { return obstacleDirection; }
    
    // State queries
    bool isBalanced() const;
    bool isMoving() const;
    float getStability() const { return stabilityScore; }
    float getEfficiency() const { return efficiencyScore; }
    
    // Sensor status for display
    struct SensorStatus {
        int footContacts = 0;
        vsg_vec3 angularVelocity;
        float averageJointVelocity = 0.0f;
        bool imuValid = false;
    };
    SensorStatus getSensorStatus() const;

private:
    void updateManualControl(double deltaTime);
    void updateAutonomousControl(double deltaTime);
    void updateLearningControl(double deltaTime);
    void updateDemonstrationControl(double deltaTime);
    
    void planPath();
    void followPath(double deltaTime);
    void avoidObstacles();
    void maintainBalance(float deltaTime);
    void optimizeGait();
    void adaptToTerrain();
    void smoothPath();
    void updatePerformanceMetrics(double deltaTime);
    float evaluatePerformance();
    bool hasNavigationGoal() const;
    vsg_vec3 eulerFromQuat(const vsg_quat& q) const;
    
    // PID controllers for stability
    struct PIDController {
        float kp, ki, kd;
        float integral;
        float previousError;
        float update(float error, float dt);
    };
    
    Robot* robot;
    PhysicsWorld* physicsWorld;
    ControlMode controlMode;
    ControlInput currentInput;
    NavigationGoal currentGoal;
    
    // Path planning
    std::vector<vsg_vec3> navigationPath;
    size_t currentPathIndex;
    float pathFollowingSpeed;
    
    // Control parameters
    float aggressiveness = 0.5f;
    float reactionTime = 0.1f;
    float maxAcceleration = 5.0f;
    float maxAngularAcceleration = 3.0f;
    
    // Feature flags
    bool obstacleAvoidanceEnabled = true;
    
    // Manual control
    vsg_vec3 manualVelocity{0.0f, 0.0f, 0.0f};
    float manualRotation = 0.0f;
    bool pathSmoothingEnabled = true;
    bool dynamicStabilityEnabled = true;
    
    // Sensor processing
    vsg_vec3 obstacleDirection;
    float obstacleDistance;
    std::vector<vsg_vec3> detectedObstacles;
    
    // Noise filtering for sensors
    struct FilteredSensorData {
        // Exponential moving average filter state
        vsg_vec3 filteredAngularVel{0.0f, 0.0f, 0.0f};
        vsg_quat filteredOrientation{0.0f, 0.0f, 0.0f, 1.0f};
        vsg_vec3 filteredLinearAccel{0.0f, 0.0f, 0.0f};
        
        // Contact sensor voting buffer (last N frames)
        static constexpr int CONTACT_HISTORY_SIZE = 5;
        std::array<std::array<bool, 6>, CONTACT_HISTORY_SIZE> contactHistory{};
        int contactHistoryIndex = 0;
        
        // EMA filter coefficients (0-1, higher = more weight on new data)
        float angularVelAlpha = 0.3f;
        float orientationAlpha = 0.5f;
        float linearAccelAlpha = 0.2f;
        
        // Methods for filtering
        void updateIMU(const vsg_quat& newOrientation, const vsg_vec3& newAngularVel, const vsg_vec3& newLinearAccel);
        void updateContacts(const std::array<bool, 6>& contacts);
        int getStableContactCount() const;
        bool isFootStableContact(int footIndex) const;
    } filteredSensorData;
    
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