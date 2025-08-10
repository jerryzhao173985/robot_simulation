#pragma once

#include "FallbackTypes.h"
#include <vector>
#include <array>
#include <memory>
#include <functional>
#include <queue>
#include <unordered_map>
#include <cmath>

#ifdef USE_OPENGL_FALLBACK
    using vsg_vec3 = vec3;
    using vsg_vec4 = vec4;
    using vsg_quat = quat;
#else
    // Check if we actually have VSG available
    #if __has_include(<vsg/all.h>)
        #include <vsg/all.h>
        using vsg_vec3 = vsg::vec3;
        using vsg_vec4 = vsg::vec4;
        using vsg_quat = vsg::quat;
    #else
        // Fallback to simple vector types for standalone use
        struct vsg_vec3 {
            float x, y, z;
            vsg_vec3() : x(0), y(0), z(0) {}
            vsg_vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
            vsg_vec3 operator+(const vsg_vec3& other) const { return vsg_vec3(x + other.x, y + other.y, z + other.z); }
            vsg_vec3 operator-(const vsg_vec3& other) const { return vsg_vec3(x - other.x, y - other.y, z - other.z); }
            vsg_vec3 operator*(float s) const { return vsg_vec3(x * s, y * s, z * s); }
        };
        struct vsg_vec4 {
            float x, y, z, w;
            vsg_vec4() : x(0), y(0), z(0), w(0) {}
            vsg_vec4(float x_, float y_, float z_, float w_) : x(x_), y(y_), z(z_), w(w_) {}
        };
        struct vsg_quat {
            float x, y, z, w;
            vsg_quat() : x(0), y(0), z(0), w(1) {}
            vsg_quat(float x_, float y_, float z_, float w_) : x(x_), y(y_), z(z_), w(w_) {}
            vsg_vec3 operator*(const vsg_vec3& v) const {
                // Simple quaternion-vector multiplication
                float qx = x, qy = y, qz = z, qw = w;
                float vx = v.x, vy = v.y, vz = v.z;
                return vsg_vec3(
                    vx * (1 - 2 * (qy * qy + qz * qz)) + vy * 2 * (qx * qy - qz * qw) + vz * 2 * (qx * qz + qy * qw),
                    vx * 2 * (qx * qy + qz * qw) + vy * (1 - 2 * (qx * qx + qz * qz)) + vz * 2 * (qy * qz - qx * qw),
                    vx * 2 * (qx * qz - qy * qw) + vy * 2 * (qy * qz + qx * qw) + vz * (1 - 2 * (qx * qx + qy * qy))
                );
            }
        };
    #endif
#endif

namespace ControlAlgorithms {

// ============================================================================
// BASIC CONTROL ALGORITHMS
// ============================================================================

// Enhanced PID Controller with anti-windup, derivative filtering, and feed-forward
class PIDController {
private:
    float kp_, ki_, kd_, kf_;  // Proportional, Integral, Derivative, Feed-forward gains
    float integral_;
    float previousError_;
    float integralMax_;  // Anti-windup limit
    float derivativeFilter_;  // Low-pass filter coefficient for derivative
    float filteredDerivative_;
    bool firstUpdate_;
    
public:
    PIDController(float kp = 1.0f, float ki = 0.0f, float kd = 0.0f, float kf = 0.0f);
    
    float update(float error, float dt, float feedforward = 0.0f);
    void setGains(float kp, float ki, float kd, float kf = 0.0f);
    void setIntegralLimit(float limit) { integralMax_ = limit; }
    void setDerivativeFilter(float alpha) { derivativeFilter_ = alpha; }
    void reset();
    
    float getProportionalTerm() const { return kp_ * previousError_; }
    float getIntegralTerm() const { return ki_ * integral_; }
    float getDerivativeTerm() const { return kd_ * filteredDerivative_; }
};

// Finite State Machine for robot control modes
template<typename StateType, typename EventType>
class StateMachine {
private:
    StateType currentState_;
    std::unordered_map<StateType, std::unordered_map<EventType, StateType>> transitions_;
    std::unordered_map<StateType, std::function<void()>> entryActions_;
    std::unordered_map<StateType, std::function<void()>> exitActions_;
    std::unordered_map<StateType, std::function<void(float)>> stateActions_;
    
public:
    StateMachine(StateType initialState) : currentState_(initialState) {}
    
    void addTransition(StateType from, EventType event, StateType to) {
        transitions_[from][event] = to;
    }
    
    void setEntryAction(StateType state, std::function<void()> action) {
        entryActions_[state] = action;
    }
    
    void setExitAction(StateType state, std::function<void()> action) {
        exitActions_[state] = action;
    }
    
    void setStateAction(StateType state, std::function<void(float)> action) {
        stateActions_[state] = action;
    }
    
    bool processEvent(EventType event) {
        if (transitions_.count(currentState_) && transitions_[currentState_].count(event)) {
            StateType nextState = transitions_[currentState_][event];
            
            // Execute exit action
            if (exitActions_.count(currentState_)) {
                exitActions_[currentState_]();
            }
            
            currentState_ = nextState;
            
            // Execute entry action
            if (entryActions_.count(currentState_)) {
                entryActions_[currentState_]();
            }
            
            return true;
        }
        return false;
    }
    
    void update(float dt) {
        if (stateActions_.count(currentState_)) {
            stateActions_[currentState_](dt);
        }
    }
    
    StateType getCurrentState() const { return currentState_; }
};

// Advanced cascade PID controller for complex systems
class CascadePIDController {
private:
    PIDController outerLoop_;
    PIDController innerLoop_;
    
public:
    CascadePIDController(float outerKp, float outerKi, float outerKd,
                        float innerKp, float innerKi, float innerKd);
    
    float update(float setpoint, float outerFeedback, float innerFeedback, float dt);
    void setOuterGains(float kp, float ki, float kd);
    void setInnerGains(float kp, float ki, float kd);
    void reset();
};

// ============================================================================
// INTERMEDIATE CONTROL ALGORITHMS  
// ============================================================================

// Forward and Inverse Kinematics for hexapod legs
class LegKinematics {
public:
    struct JointAngles {
        float coxa;   // Hip joint
        float femur;  // Thigh joint  
        float tibia;  // Shin joint
    };
    
    struct LegParams {
        float coxaLength;
        float femurLength; 
        float tibiaLength;
        vsg_vec3 baseOffset;  // Attachment point relative to body center
    };
    
private:
    LegParams params_;
    
public:
    LegKinematics(const LegParams& params) : params_(params) {}
    
    // Forward kinematics: joint angles -> foot position
    vsg_vec3 forwardKinematics(const JointAngles& angles) const;
    
    // Inverse kinematics: foot position -> joint angles
    bool inverseKinematics(const vsg_vec3& footPos, JointAngles& angles) const;
    
    // Jacobian for velocity/force transforms
    std::array<std::array<float, 3>, 3> getJacobian(const JointAngles& angles) const;
    
    // Workspace boundary check
    bool isReachable(const vsg_vec3& footPos) const;
    
    // Get foot velocity from joint velocities
    vsg_vec3 getFootVelocity(const JointAngles& angles, const JointAngles& angularVel) const;
};

// Gait Pattern Generator for hexapod locomotion
class GaitGenerator {
public:
    enum GaitType {
        WAVE_GAIT,      // Slow, stable - one leg at a time
        RIPPLE_GAIT,    // Medium speed - two legs alternating 
        TRIPOD_GAIT,    // Fast - two tripods alternating
        CUSTOM_GAIT     // User-defined pattern
    };
    
    struct FootTrajectory {
        vsg_vec3 position;
        vsg_vec3 velocity;
        bool inSwing;     // true if foot is in swing phase, false if stance
        float phaseTime;  // current phase time [0,1]
    };
    
    struct GaitParams {
        GaitType type;
        float stepHeight;
        float strideLength;
        float cyclePeriod;
        float dutyCycle;  // fraction of cycle in stance phase
        vsg_vec3 bodyVelocity;
        float angularVelocity;
    };
    
private:
    GaitParams params_;
    std::array<float, 6> legPhases_;  // Phase offset for each leg
    float currentTime_;
    
    // Foot trajectory generation
    vsg_vec3 generateSwingTrajectory(float phase, const vsg_vec3& start, const vsg_vec3& end, float height) const;
    vsg_vec3 generateStanceTrajectory(float phase, const vsg_vec3& start, float strideLength, float direction) const;
    
public:
    GaitGenerator(const GaitParams& params);
    
    void setGaitParams(const GaitParams& params);
    void update(float dt);
    
    FootTrajectory getFootTrajectory(int legIndex) const;
    std::array<FootTrajectory, 6> getAllFootTrajectories() const;
    
    void setCustomGait(const std::array<float, 6>& phaseOffsets);
    float getCurrentCycleTime() const { return currentTime_; }
};

// Terrain Adaptation Algorithm
class TerrainAdapter {
public:
    struct TerrainInfo {
        vsg_vec3 normal;
        float slope;
        float roughness;
        float hardness;
        bool isStable;
    };
    
    struct AdaptationParams {
        float minGroundClearance;
        float maxStepHeight;
        float stabilityThreshold;
        float adaptationRate;
        bool enableProbing;
    };
    
private:
    AdaptationParams params_;
    std::array<TerrainInfo, 6> footTerrainInfo_;  // Terrain at each foot
    vsg_vec3 bodyOrientation_;
    
    TerrainInfo analyzeLocalTerrain(const vsg_vec3& position, class PhysicsWorld* physics) const;
    void adjustBodyPose(const std::array<TerrainInfo, 6>& terrainData);
    
public:
    TerrainAdapter(const AdaptationParams& params) : params_(params) {}
    
    void update(const std::array<vsg_vec3, 6>& footPositions, class PhysicsWorld* physics);
    
    vsg_vec3 getAdaptedBodyOrientation() const { return bodyOrientation_; }
    std::array<float, 6> getFootHeightAdjustments() const;
    TerrainInfo getFootTerrainInfo(int legIndex) const;
    
    bool shouldModifyGait() const;
    float getRecommendedSpeed() const;
};

// ============================================================================
// ADVANCED CONTROL ALGORITHMS
// ============================================================================

// A* Path Planning Algorithm
class PathPlanner {
public:
    struct Node {
        vsg_vec3 position;
        float g_cost;  // Distance from start
        float h_cost;  // Heuristic distance to goal
        float f_cost() const { return g_cost + h_cost; }
        Node* parent;
        
        Node(const vsg_vec3& pos) : position(pos), g_cost(0), h_cost(0), parent(nullptr) {}
    };
    
    struct PlanningParams {
        float stepSize;
        float goalTolerance;
        float maxPlanningTime;
        int maxNodes;
        bool optimizePath;
    };
    
private:
    PlanningParams params_;
    std::vector<vsg_vec3> obstacles_;
    
    float heuristic(const vsg_vec3& a, const vsg_vec3& b) const;
    bool isCollisionFree(const vsg_vec3& from, const vsg_vec3& to) const;
    std::vector<Node*> getNeighbors(Node* node) const;
    std::vector<vsg_vec3> smoothPath(const std::vector<vsg_vec3>& path) const;
    
public:
    PathPlanner(const PlanningParams& params) : params_(params) {}
    
    void setObstacles(const std::vector<vsg_vec3>& obstacles) { obstacles_ = obstacles; }
    void addObstacle(const vsg_vec3& position) { obstacles_.push_back(position); }
    
    std::vector<vsg_vec3> planPath(const vsg_vec3& start, const vsg_vec3& goal);
    
    bool isPathValid(const std::vector<vsg_vec3>& path) const;
    float getPathLength(const std::vector<vsg_vec3>& path) const;
};

// Dynamic Movement Primitives for learning and adaptation
class DMP {
public:
    struct DMPParams {
        int numBasisFunctions;
        float alpha;  // Temporal scaling
        float beta;   // Damping
        float tau;    // Time constant
    };
    
    struct Trajectory {
        std::vector<vsg_vec3> positions;
        std::vector<vsg_vec3> velocities;
        std::vector<float> timestamps;
    };
    
private:
    DMPParams params_;
    std::vector<float> weights_;
    std::vector<float> centers_;
    std::vector<float> widths_;
    vsg_vec3 goal_;
    vsg_vec3 start_;
    
    float basisFunction(float s, int i) const;
    float canonicalSystem(float t) const;
    
public:
    DMP(const DMPParams& params);
    
    void learnFromTrajectory(const Trajectory& demo);
    void setGoal(const vsg_vec3& goal) { goal_ = goal; }
    void setStart(const vsg_vec3& start) { start_ = start; }
    
    vsg_vec3 getPosition(float t) const;
    vsg_vec3 getVelocity(float t) const;
    vsg_vec3 getAcceleration(float t) const;
    
    void adaptToNewGoal(const vsg_vec3& newGoal);
    Trajectory generateTrajectory(float duration, float dt) const;
};

// Reinforcement Learning Controller using Q-Learning
class RLController {
public:
    struct State {
        std::vector<float> features;
        size_t hash() const;
        bool operator==(const State& other) const;
    };
    
    struct Action {
        std::vector<float> commands;
        int discreteAction;  // For discrete action spaces
    };
    
    struct RLParams {
        float learningRate;
        float discountFactor;
        float explorationRate;
        float explorationDecay;
        int stateSize;
        int actionSize;
        bool useContinuous;  // Continuous vs discrete actions
    };
    
private:
    RLParams params_;
    std::unordered_map<size_t, std::unordered_map<int, float>> qTable_;
    std::vector<std::pair<State, Action>> experienceBuffer_;
    int maxExperienceSize_;
    
    int selectAction(const State& state);
    void updateQValue(const State& state, int action, float reward, const State& nextState);
    State extractState(const std::vector<float>& sensorData) const;
    
public:
    RLController(const RLParams& params);
    
    Action selectAction(const std::vector<float>& sensorData);
    void updatePolicy(const std::vector<float>& state, const Action& action, 
                     float reward, const std::vector<float>& nextState);
    
    void savePolicy(const std::string& filename) const;
    void loadPolicy(const std::string& filename);
    
    float getExplorationRate() const { return params_.explorationRate; }
    void setExplorationRate(float rate) { params_.explorationRate = rate; }
};

// Obstacle Avoidance using Vector Field Histogram (VFH)
class ObstacleAvoidance {
public:
    struct VFHParams {
        int histogramSectors;
        float threshold;
        float robotRadius;
        float lookAheadDistance;
        float safetyMargin;
    };
    
private:
    VFHParams params_;
    std::vector<float> polarHistogram_;
    
    void buildPolarHistogram(const std::vector<vsg_vec3>& obstacles, const vsg_vec3& robotPos);
    float findBestDirection(float preferredDirection) const;
    
public:
    ObstacleAvoidance(const VFHParams& params);
    
    vsg_vec3 computeAvoidanceVector(const vsg_vec3& robotPos, 
                                   const vsg_vec3& goalDirection,
                                   const std::vector<vsg_vec3>& obstacles);
    
    bool isDirectionSafe(float direction, const vsg_vec3& robotPos,
                        const std::vector<vsg_vec3>& obstacles) const;
    
    std::vector<float> getPolarHistogram() const { return polarHistogram_; }
};

// Model Predictive Controller for advanced trajectory following
class MPController {
public:
    struct MPCParams {
        int predictionHorizon;
        int controlHorizon;
        float dt;
        float positionWeight;
        float velocityWeight;
        float accelerationWeight;
        float inputWeight;
    };
    
    struct SystemModel {
        // Linear system model: x(k+1) = A*x(k) + B*u(k)
        std::vector<std::vector<float>> A;  // State transition matrix
        std::vector<std::vector<float>> B;  // Input matrix
        std::vector<std::vector<float>> C;  // Output matrix
    };
    
private:
    MPCParams params_;
    SystemModel model_;
    std::vector<float> currentState_;
    std::queue<std::vector<float>> controlSequence_;
    
    std::vector<float> optimizeControl(const std::vector<float>& reference);
    float costFunction(const std::vector<float>& predictedStates, 
                      const std::vector<float>& controlInputs,
                      const std::vector<float>& reference) const;
    
public:
    MPController(const MPCParams& params, const SystemModel& model);
    
    std::vector<float> computeControl(const std::vector<float>& currentState,
                                     const std::vector<float>& reference);
    
    void updateModel(const SystemModel& newModel) { model_ = newModel; }
    void setState(const std::vector<float>& state) { currentState_ = state; }
};

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

namespace Utils {
    // Math utilities
    float clamp(float value, float min, float max);
    float lerp(float a, float b, float t);
    float smoothStep(float edge0, float edge1, float x);
    
    // Vector operations
    float dot(const vsg_vec3& a, const vsg_vec3& b);
    vsg_vec3 cross(const vsg_vec3& a, const vsg_vec3& b);
    float length(const vsg_vec3& v);
    vsg_vec3 normalize(const vsg_vec3& v);
    
    // Quaternion operations
    vsg_quat slerp(const vsg_quat& a, const vsg_quat& b, float t);
    vsg_vec3 eulerFromQuaternion(const vsg_quat& q);
    vsg_quat quaternionFromEuler(const vsg_vec3& euler);
    
    // Filter utilities
    class LowPassFilter {
    private:
        float alpha_;
        float value_;
        bool initialized_;
        
    public:
        LowPassFilter(float alpha) : alpha_(alpha), value_(0), initialized_(false) {}
        
        float update(float input) {
            if (!initialized_) {
                value_ = input;
                initialized_ = true;
            } else {
                value_ = alpha_ * input + (1.0f - alpha_) * value_;
            }
            return value_;
        }
        
        void reset() { initialized_ = false; }
        float getValue() const { return value_; }
    };
    
    // Moving average filter
    class MovingAverageFilter {
    private:
        std::queue<float> values_;
        int windowSize_;
        float sum_;
        
    public:
        MovingAverageFilter(int windowSize) : windowSize_(windowSize), sum_(0) {}
        
        float update(float input) {
            values_.push(input);
            sum_ += input;
            
            if (values_.size() > windowSize_) {
                sum_ -= values_.front();
                values_.pop();
            }
            
            return sum_ / values_.size();
        }
        
        void reset() {
            while (!values_.empty()) values_.pop();
            sum_ = 0;
        }
    };
}

} // namespace ControlAlgorithms