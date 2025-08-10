#include "ControlAlgorithms.h"
#if __has_include("PhysicsWorld.h")
    #include "PhysicsWorld.h"
#endif
#include <algorithm>
#include <random>
#include <fstream>
#include <iostream>
#include <cmath>
#include <limits>
#include <queue>

namespace ControlAlgorithms {

// ============================================================================
// BASIC CONTROL ALGORITHMS IMPLEMENTATION
// ============================================================================

PIDController::PIDController(float kp, float ki, float kd, float kf)
    : kp_(kp), ki_(ki), kd_(kd), kf_(kf)
    , integral_(0), previousError_(0), integralMax_(1000.0f)
    , derivativeFilter_(0.1f), filteredDerivative_(0)
    , firstUpdate_(true) {}

float PIDController::update(float error, float dt, float feedforward) {
    if (dt <= 0) return 0;
    
    // Proportional term
    float proportional = kp_ * error;
    
    // Integral term with anti-windup
    integral_ += error * dt;
    integral_ = Utils::clamp(integral_, -integralMax_, integralMax_);
    float integralTerm = ki_ * integral_;
    
    // Derivative term with filtering
    float derivative = 0;
    if (!firstUpdate_) {
        derivative = (error - previousError_) / dt;
        // Low-pass filter for derivative
        filteredDerivative_ = derivativeFilter_ * derivative + (1.0f - derivativeFilter_) * filteredDerivative_;
    } else {
        filteredDerivative_ = 0;
        firstUpdate_ = false;
    }
    float derivativeTerm = kd_ * filteredDerivative_;
    
    // Feed-forward term
    float feedforwardTerm = kf_ * feedforward;
    
    previousError_ = error;
    
    return proportional + integralTerm - derivativeTerm + feedforwardTerm;
}

void PIDController::setGains(float kp, float ki, float kd, float kf) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    kf_ = kf;
}

void PIDController::reset() {
    integral_ = 0;
    previousError_ = 0;
    filteredDerivative_ = 0;
    firstUpdate_ = true;
}

// Template method implementations are now in the header file

// Cascade PID Implementation
CascadePIDController::CascadePIDController(float outerKp, float outerKi, float outerKd,
                                          float innerKp, float innerKi, float innerKd)
    : outerLoop_(outerKp, outerKi, outerKd)
    , innerLoop_(innerKp, innerKi, innerKd) {}

float CascadePIDController::update(float setpoint, float outerFeedback, float innerFeedback, float dt) {
    // Outer loop generates setpoint for inner loop
    float outerError = setpoint - outerFeedback;
    float innerSetpoint = outerLoop_.update(outerError, dt);
    
    // Inner loop tracks the outer loop's output
    float innerError = innerSetpoint - innerFeedback;
    return innerLoop_.update(innerError, dt);
}

void CascadePIDController::setOuterGains(float kp, float ki, float kd) {
    outerLoop_.setGains(kp, ki, kd);
}

void CascadePIDController::setInnerGains(float kp, float ki, float kd) {
    innerLoop_.setGains(kp, ki, kd);
}

void CascadePIDController::reset() {
    outerLoop_.reset();
    innerLoop_.reset();
}

// ============================================================================
// INTERMEDIATE CONTROL ALGORITHMS IMPLEMENTATION
// ============================================================================

// Leg Kinematics Implementation
vsg_vec3 LegKinematics::forwardKinematics(const JointAngles& angles) const {
    // Forward kinematics for 3-DOF leg
    float c1 = std::cos(angles.coxa);
    float s1 = std::sin(angles.coxa);
    float c2 = std::cos(angles.femur);
    float s2 = std::sin(angles.femur);
    float c23 = std::cos(angles.femur + angles.tibia);
    float s23 = std::sin(angles.femur + angles.tibia);
    
    // Position in leg coordinate frame
    float x_leg = params_.coxaLength + params_.femurLength * c2 + params_.tibiaLength * c23;
    float y_leg = 0;
    float z_leg = params_.femurLength * s2 + params_.tibiaLength * s23;
    
    // Transform to body coordinate frame
    vsg_vec3 footPos;
    footPos.x = params_.baseOffset.x + c1 * x_leg - s1 * y_leg;
    footPos.y = params_.baseOffset.y + s1 * x_leg + c1 * y_leg;
    footPos.z = params_.baseOffset.z + z_leg;
    
    return footPos;
}

bool LegKinematics::inverseKinematics(const vsg_vec3& footPos, JointAngles& angles) const {
    // Transform to leg coordinate frame
    vsg_vec3 localPos;
    localPos.x = footPos.x - params_.baseOffset.x;
    localPos.y = footPos.y - params_.baseOffset.y;
    localPos.z = footPos.z - params_.baseOffset.z;
    
    // Coxa angle (hip rotation)
    angles.coxa = std::atan2(localPos.y, localPos.x);
    
    // Distance from coxa joint to foot
    float r = std::sqrt(localPos.x * localPos.x + localPos.y * localPos.y) - params_.coxaLength;
    float h = localPos.z;
    float d = std::sqrt(r * r + h * h);
    
    // Check if position is reachable
    if (d > params_.femurLength + params_.tibiaLength || 
        d < std::abs(params_.femurLength - params_.tibiaLength)) {
        return false; // Unreachable
    }
    
    // Tibia angle using law of cosines
    float cosAngle = (params_.femurLength * params_.femurLength + params_.tibiaLength * params_.tibiaLength - d * d) /
                     (2.0f * params_.femurLength * params_.tibiaLength);
    cosAngle = Utils::clamp(cosAngle, -1.0f, 1.0f);
    angles.tibia = std::acos(cosAngle) - M_PI; // Negative for typical leg configuration
    
    // Femur angle
    float alpha = std::atan2(h, r);
    float beta = std::acos(Utils::clamp((params_.femurLength * params_.femurLength + d * d - params_.tibiaLength * params_.tibiaLength) /
                                       (2.0f * params_.femurLength * d), -1.0f, 1.0f));
    angles.femur = alpha + beta;
    
    return true;
}

std::array<std::array<float, 3>, 3> LegKinematics::getJacobian(const JointAngles& angles) const {
    std::array<std::array<float, 3>, 3> jacobian = {};
    
    float c1 = std::cos(angles.coxa);
    float s1 = std::sin(angles.coxa);
    float c2 = std::cos(angles.femur);
    float s2 = std::sin(angles.femur);
    float c23 = std::cos(angles.femur + angles.tibia);
    float s23 = std::sin(angles.femur + angles.tibia);
    
    float x_leg = params_.coxaLength + params_.femurLength * c2 + params_.tibiaLength * c23;
    float y_leg = 0;
    
    // ∂x/∂θ
    jacobian[0][0] = -s1 * x_leg - c1 * y_leg;  // ∂x/∂coxa
    jacobian[0][1] = c1 * (-params_.femurLength * s2 - params_.tibiaLength * s23);  // ∂x/∂femur
    jacobian[0][2] = -c1 * params_.tibiaLength * s23;  // ∂x/∂tibia
    
    // ∂y/∂θ
    jacobian[1][0] = c1 * x_leg - s1 * y_leg;   // ∂y/∂coxa
    jacobian[1][1] = s1 * (-params_.femurLength * s2 - params_.tibiaLength * s23);  // ∂y/∂femur
    jacobian[1][2] = -s1 * params_.tibiaLength * s23;  // ∂y/∂tibia
    
    // ∂z/∂θ
    jacobian[2][0] = 0;  // ∂z/∂coxa
    jacobian[2][1] = params_.femurLength * c2 + params_.tibiaLength * c23;  // ∂z/∂femur
    jacobian[2][2] = params_.tibiaLength * c23;  // ∂z/∂tibia
    
    return jacobian;
}

bool LegKinematics::isReachable(const vsg_vec3& footPos) const {
    vsg_vec3 localPos;
    localPos.x = footPos.x - params_.baseOffset.x;
    localPos.y = footPos.y - params_.baseOffset.y;
    localPos.z = footPos.z - params_.baseOffset.z;
    
    float r = std::sqrt(localPos.x * localPos.x + localPos.y * localPos.y) - params_.coxaLength;
    float h = localPos.z;
    float d = std::sqrt(r * r + h * h);
    
    return d <= params_.femurLength + params_.tibiaLength && 
           d >= std::abs(params_.femurLength - params_.tibiaLength);
}

vsg_vec3 LegKinematics::getFootVelocity(const JointAngles& angles, const JointAngles& angularVel) const {
    auto jacobian = getJacobian(angles);
    
    vsg_vec3 velocity;
    velocity.x = jacobian[0][0] * angularVel.coxa + jacobian[0][1] * angularVel.femur + jacobian[0][2] * angularVel.tibia;
    velocity.y = jacobian[1][0] * angularVel.coxa + jacobian[1][1] * angularVel.femur + jacobian[1][2] * angularVel.tibia;
    velocity.z = jacobian[2][0] * angularVel.coxa + jacobian[2][1] * angularVel.femur + jacobian[2][2] * angularVel.tibia;
    
    return velocity;
}

// Gait Generator Implementation
GaitGenerator::GaitGenerator(const GaitParams& params) : params_(params), currentTime_(0) {
    // Initialize leg phases based on gait type
    switch (params.type) {
        case WAVE_GAIT:
            for (int i = 0; i < 6; ++i) {
                legPhases_[i] = i / 6.0f;
            }
            break;
            
        case RIPPLE_GAIT:
            legPhases_[0] = 0.0f; legPhases_[1] = 0.5f;
            legPhases_[2] = 1/3.0f; legPhases_[3] = 5/6.0f;
            legPhases_[4] = 2/3.0f; legPhases_[5] = 1/6.0f;
            break;
            
        case TRIPOD_GAIT:
            legPhases_[0] = 0.0f; legPhases_[1] = 0.5f;
            legPhases_[2] = 0.5f; legPhases_[3] = 0.0f;
            legPhases_[4] = 0.0f; legPhases_[5] = 0.5f;
            break;
            
        case CUSTOM_GAIT:
            // Custom phases set by user
            break;
    }
}

void GaitGenerator::setGaitParams(const GaitParams& params) {
    params_ = params;
}

void GaitGenerator::update(float dt) {
    currentTime_ += dt;
    if (currentTime_ >= params_.cyclePeriod) {
        currentTime_ -= params_.cyclePeriod;
    }
}

GaitGenerator::FootTrajectory GaitGenerator::getFootTrajectory(int legIndex) const {
    if (legIndex < 0 || legIndex >= 6) {
        return FootTrajectory{};
    }
    
    float phase = std::fmod(currentTime_ / params_.cyclePeriod + legPhases_[legIndex], 1.0f);
    
    FootTrajectory traj;
    traj.phaseTime = phase;
    
    if (phase < params_.dutyCycle) {
        // Stance phase
        traj.inSwing = false;
        float stancePhase = phase / params_.dutyCycle;
        
        // Move foot backward relative to body
        vsg_vec3 start = vsg_vec3(params_.strideLength * 0.5f, 0, 0);
        traj.position = generateStanceTrajectory(stancePhase, start, params_.strideLength, 0);
        
        // Add body motion compensation
        traj.position.x += params_.bodyVelocity.x * currentTime_;
        traj.position.y += params_.bodyVelocity.y * currentTime_;
        
        traj.velocity = vsg_vec3(-params_.bodyVelocity.x, -params_.bodyVelocity.y, 0);
    } else {
        // Swing phase
        traj.inSwing = true;
        float swingPhase = (phase - params_.dutyCycle) / (1.0f - params_.dutyCycle);
        
        vsg_vec3 start = vsg_vec3(-params_.strideLength * 0.5f, 0, 0);
        vsg_vec3 end = vsg_vec3(params_.strideLength * 0.5f, 0, 0);
        
        traj.position = generateSwingTrajectory(swingPhase, start, end, params_.stepHeight);
        
        // Add body motion compensation
        traj.position.x += params_.bodyVelocity.x * currentTime_;
        traj.position.y += params_.bodyVelocity.y * currentTime_;
        
        // Calculate swing velocity
        float swingSpeed = params_.strideLength / ((1.0f - params_.dutyCycle) * params_.cyclePeriod);
        traj.velocity = vsg_vec3(swingSpeed, 0, 0);
    }
    
    return traj;
}

std::array<GaitGenerator::FootTrajectory, 6> GaitGenerator::getAllFootTrajectories() const {
    std::array<FootTrajectory, 6> trajectories;
    for (int i = 0; i < 6; ++i) {
        trajectories[i] = getFootTrajectory(i);
    }
    return trajectories;
}

vsg_vec3 GaitGenerator::generateSwingTrajectory(float phase, const vsg_vec3& start, const vsg_vec3& end, float height) const {
    // Smooth swing trajectory using spline
    vsg_vec3 pos;
    pos.x = Utils::lerp(start.x, end.x, phase);
    pos.y = Utils::lerp(start.y, end.y, phase);
    
    // Parabolic height profile
    pos.z = start.z + 4 * height * phase * (1 - phase);
    
    return pos;
}

vsg_vec3 GaitGenerator::generateStanceTrajectory(float phase, const vsg_vec3& start, float strideLength, float direction) const {
    vsg_vec3 pos = start;
    pos.x -= strideLength * phase;  // Move backward during stance
    return pos;
}

void GaitGenerator::setCustomGait(const std::array<float, 6>& phaseOffsets) {
    legPhases_ = phaseOffsets;
    params_.type = CUSTOM_GAIT;
}

// Terrain Adapter Implementation
void TerrainAdapter::update(const std::array<vsg_vec3, 6>& footPositions, class PhysicsWorld* physics) {
    // Analyze terrain at each foot position
    for (int i = 0; i < 6; ++i) {
        footTerrainInfo_[i] = analyzeLocalTerrain(footPositions[i], physics);
    }
    
    // Adjust body pose based on terrain
    adjustBodyPose(footTerrainInfo_);
}

TerrainAdapter::TerrainInfo TerrainAdapter::analyzeLocalTerrain(const vsg_vec3& position, class PhysicsWorld* physics) const {
    TerrainInfo info;
    
    if (!physics) {
        // Default values if no physics world
        info.normal = vsg_vec3(0, 0, 1);
        info.slope = 0;
        info.roughness = 0;
        info.hardness = 1.0f;
        info.isStable = true;
        return info;
    }
    
#if __has_include("PhysicsWorld.h")
    if (physics) {
        // Perform raycast to get terrain info
        vsg_vec3 rayStart = position + vsg_vec3(0, 0, 0.5f);
        vsg_vec3 rayDir = vsg_vec3(0, 0, -1);
        
        auto result = physics->raycast(rayStart, rayDir, 1.0f);
        
        if (result.hit) {
            info.normal = result.normal;
            info.slope = std::acos(Utils::dot(result.normal, vsg_vec3(0, 0, 1)));
            
            // Simple roughness estimation based on surrounding terrain
            info.roughness = 0.1f; // Placeholder
            info.hardness = 1.0f;  // Placeholder
            info.isStable = info.slope < params_.stabilityThreshold;
        } else {
            // No ground found
            info.normal = vsg_vec3(0, 0, 1);
            info.slope = 0;
            info.roughness = 0;
            info.hardness = 0;
            info.isStable = false;
        }
    } else
#endif
    {
        // No physics world available - use defaults
        info.normal = vsg_vec3(0, 0, 1);
        info.slope = 0;
        info.roughness = 0.1f;
        info.hardness = 1.0f;
        info.isStable = true;
    }
    
    return info;
}

void TerrainAdapter::adjustBodyPose(const std::array<TerrainInfo, 6>& terrainData) {
    // Calculate average terrain normal
    vsg_vec3 avgNormal(0, 0, 0);
    int validCount = 0;
    
    for (const auto& info : terrainData) {
        if (info.isStable) {
            avgNormal.x += info.normal.x;
            avgNormal.y += info.normal.y;
            avgNormal.z += info.normal.z;
            validCount++;
        }
    }
    
    if (validCount > 0) {
        avgNormal.x /= validCount;
        avgNormal.y /= validCount;
        avgNormal.z /= validCount;
        avgNormal = Utils::normalize(avgNormal);
        
        // Calculate desired body orientation to align with terrain
        float roll = std::atan2(avgNormal.y, avgNormal.z);
        float pitch = -std::atan2(avgNormal.x, avgNormal.z);
        
        bodyOrientation_.x = roll;
        bodyOrientation_.y = pitch;
        bodyOrientation_.z = 0;
    }
}

std::array<float, 6> TerrainAdapter::getFootHeightAdjustments() const {
    std::array<float, 6> adjustments = {};
    
    for (int i = 0; i < 6; ++i) {
        if (!footTerrainInfo_[i].isStable) {
            adjustments[i] = params_.minGroundClearance;
        } else {
            adjustments[i] = 0;
        }
    }
    
    return adjustments;
}

TerrainAdapter::TerrainInfo TerrainAdapter::getFootTerrainInfo(int legIndex) const {
    if (legIndex >= 0 && legIndex < 6) {
        return footTerrainInfo_[legIndex];
    }
    return TerrainInfo{};
}

bool TerrainAdapter::shouldModifyGait() const {
    int unstableCount = 0;
    for (const auto& info : footTerrainInfo_) {
        if (!info.isStable) unstableCount++;
    }
    return unstableCount > 2; // Modify gait if more than 2 feet on unstable terrain
}

float TerrainAdapter::getRecommendedSpeed() const {
    float avgSlope = 0;
    for (const auto& info : footTerrainInfo_) {
        avgSlope += info.slope;
    }
    avgSlope /= 6;
    
    // Reduce speed based on terrain difficulty
    float speedFactor = 1.0f - (avgSlope / (M_PI / 4)); // Reduce speed for slopes > 45 degrees
    return Utils::clamp(speedFactor, 0.1f, 1.0f);
}

// ============================================================================
// ADVANCED CONTROL ALGORITHMS IMPLEMENTATION
// ============================================================================

// Path Planner Implementation
float PathPlanner::heuristic(const vsg_vec3& a, const vsg_vec3& b) const {
    return Utils::length(b - a);
}

bool PathPlanner::isCollisionFree(const vsg_vec3& from, const vsg_vec3& to) const {
    // Simple collision checking with obstacles
    for (const auto& obstacle : obstacles_) {
        float dist = Utils::length(obstacle - from);
        if (dist < 0.5f) return false; // Too close to obstacle
        
        // Check if path intersects obstacle
        vsg_vec3 dir = Utils::normalize(to - from);
        vsg_vec3 toObstacle = obstacle - from;
        float projLength = Utils::dot(toObstacle, dir);
        
        if (projLength > 0 && projLength < Utils::length(to - from)) {
            vsg_vec3 closestPoint = from + dir * projLength;
            if (Utils::length(obstacle - closestPoint) < 0.5f) {
                return false;
            }
        }
    }
    return true;
}

std::vector<vsg_vec3> PathPlanner::planPath(const vsg_vec3& start, const vsg_vec3& goal) {
    std::vector<Node*> openSet;
    std::vector<Node*> closedSet;
    std::vector<std::unique_ptr<Node>> allNodes;
    
    auto startNode = std::make_unique<Node>(start);
    startNode->g_cost = 0;
    startNode->h_cost = heuristic(start, goal);
    
    openSet.push_back(startNode.get());
    allNodes.push_back(std::move(startNode));
    
    Node* goalNode = nullptr;
    
    while (!openSet.empty()) {
        // Find node with lowest f_cost
        auto current = std::min_element(openSet.begin(), openSet.end(),
                                      [](Node* a, Node* b) { return a->f_cost() < b->f_cost(); });
        Node* currentNode = *current;
        openSet.erase(current);
        closedSet.push_back(currentNode);
        
        // Check if we reached the goal
        if (heuristic(currentNode->position, goal) < params_.goalTolerance) {
            goalNode = currentNode;
            break;
        }
        
        // Get neighbors
        auto neighbors = getNeighbors(currentNode);
        
        for (Node* neighbor : neighbors) {
            // Check if in closed set
            if (std::find(closedSet.begin(), closedSet.end(), neighbor) != closedSet.end()) {
                continue;
            }
            
            float tentativeG = currentNode->g_cost + heuristic(currentNode->position, neighbor->position);
            
            // Check if in open set
            auto inOpen = std::find(openSet.begin(), openSet.end(), neighbor);
            if (inOpen == openSet.end()) {
                neighbor->g_cost = tentativeG;
                neighbor->h_cost = heuristic(neighbor->position, goal);
                neighbor->parent = currentNode;
                openSet.push_back(neighbor);
            } else if (tentativeG < neighbor->g_cost) {
                neighbor->g_cost = tentativeG;
                neighbor->parent = currentNode;
            }
        }
        
        // Cleanup dynamic neighbors
        for (Node* neighbor : neighbors) {
            delete neighbor;
        }
    }
    
    std::vector<vsg_vec3> path;
    
    if (goalNode) {
        // Reconstruct path
        Node* current = goalNode;
        while (current) {
            path.push_back(current->position);
            current = current->parent;
        }
        std::reverse(path.begin(), path.end());
        
        if (params_.optimizePath) {
            path = smoothPath(path);
        }
    }
    
    return path;
}

std::vector<PathPlanner::Node*> PathPlanner::getNeighbors(Node* node) const {
    std::vector<Node*> neighbors;
    
    // 8-directional movement in 2D plane
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) continue;
            
            vsg_vec3 newPos = node->position;
            newPos.x += dx * params_.stepSize;
            newPos.y += dy * params_.stepSize;
            
            if (isCollisionFree(node->position, newPos)) {
                neighbors.push_back(new Node(newPos));
            }
        }
    }
    
    return neighbors;
}

std::vector<vsg_vec3> PathPlanner::smoothPath(const std::vector<vsg_vec3>& path) const {
    if (path.size() <= 2) return path;
    
    std::vector<vsg_vec3> smoothed;
    smoothed.push_back(path[0]);
    
    size_t i = 0;
    while (i < path.size() - 1) {
        size_t j = path.size() - 1;
        
        // Find the farthest point we can reach directly
        while (j > i + 1 && !isCollisionFree(path[i], path[j])) {
            j--;
        }
        
        smoothed.push_back(path[j]);
        i = j;
    }
    
    return smoothed;
}

bool PathPlanner::isPathValid(const std::vector<vsg_vec3>& path) const {
    for (size_t i = 0; i < path.size() - 1; ++i) {
        if (!isCollisionFree(path[i], path[i + 1])) {
            return false;
        }
    }
    return true;
}

float PathPlanner::getPathLength(const std::vector<vsg_vec3>& path) const {
    float length = 0;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        length += Utils::length(path[i + 1] - path[i]);
    }
    return length;
}

// ============================================================================
// UTILITY FUNCTIONS IMPLEMENTATION
// ============================================================================

namespace Utils {
    float clamp(float value, float min, float max) {
        return std::max(min, std::min(value, max));
    }
    
    float lerp(float a, float b, float t) {
        return a + t * (b - a);
    }
    
    float smoothStep(float edge0, float edge1, float x) {
        float t = clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
        return t * t * (3.0f - 2.0f * t);
    }
    
    float dot(const vsg_vec3& a, const vsg_vec3& b) {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }
    
    vsg_vec3 cross(const vsg_vec3& a, const vsg_vec3& b) {
        return vsg_vec3(
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x
        );
    }
    
    float length(const vsg_vec3& v) {
        return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    }
    
    vsg_vec3 normalize(const vsg_vec3& v) {
        float len = length(v);
        if (len > 0) {
            return vsg_vec3(v.x / len, v.y / len, v.z / len);
        }
        return vsg_vec3(0, 0, 0);
    }
    
    vsg_quat slerp(const vsg_quat& a, const vsg_quat& b, float t) {
        float dotProduct = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
        
        // Handle the case where the quaternions are very similar
        if (std::abs(dotProduct) > 0.9995f) {
            vsg_quat result;
            result.x = lerp(a.x, b.x, t);
            result.y = lerp(a.y, b.y, t);
            result.z = lerp(a.z, b.z, t);
            result.w = lerp(a.w, b.w, t);
            
            float len = std::sqrt(result.x * result.x + result.y * result.y + 
                                result.z * result.z + result.w * result.w);
            if (len > 0) {
                result.x /= len;
                result.y /= len;
                result.z /= len;
                result.w /= len;
            }
            return result;
        }
        
        // Standard slerp
        float theta = std::acos(std::abs(dotProduct));
        float sinTheta = std::sin(theta);
        
        float wa = std::sin((1 - t) * theta) / sinTheta;
        float wb = std::sin(t * theta) / sinTheta;
        
        if (dotProduct < 0) wb = -wb;
        
        return vsg_quat(
            wa * a.x + wb * b.x,
            wa * a.y + wb * b.y,
            wa * a.z + wb * b.z,
            wa * a.w + wb * b.w
        );
    }
    
    vsg_vec3 eulerFromQuaternion(const vsg_quat& q) {
        vsg_vec3 euler;
        
        // Roll (x-axis rotation)
        float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
        float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
        euler.x = std::atan2(sinr_cosp, cosr_cosp);
        
        // Pitch (y-axis rotation)
        float sinp = 2 * (q.w * q.y - q.z * q.x);
        if (std::abs(sinp) >= 1)
            euler.y = std::copysign(M_PI / 2, sinp);
        else
            euler.y = std::asin(sinp);
        
        // Yaw (z-axis rotation)
        float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        euler.z = std::atan2(siny_cosp, cosy_cosp);
        
        return euler;
    }
    
    vsg_quat quaternionFromEuler(const vsg_vec3& euler) {
        float cy = std::cos(euler.z * 0.5f);
        float sy = std::sin(euler.z * 0.5f);
        float cp = std::cos(euler.y * 0.5f);
        float sp = std::sin(euler.y * 0.5f);
        float cr = std::cos(euler.x * 0.5f);
        float sr = std::sin(euler.x * 0.5f);
        
        vsg_quat q;
        q.w = cr * cp * cy + sr * sp * sy;
        q.x = sr * cp * cy - cr * sp * sy;
        q.y = cr * sp * cy + sr * cp * sy;
        q.z = cr * cp * sy - sr * sp * cy;
        
        return q;
    }
}

} // namespace ControlAlgorithms