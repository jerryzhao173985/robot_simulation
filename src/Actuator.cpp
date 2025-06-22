#include "Actuator.h"
#include "NoiseManager.h"
#include <algorithm>
#include <cmath>
#include <iostream>

// Position Motor
void PositionMotor::applyCommand(const Command& command) {
    if (!enabled_) return;
    currentCommand_ = command;
    
    // Clamp target position to limits
    currentCommand_.targetValue = std::clamp(
        currentCommand_.targetValue, 
        minPosition_, 
        maxPosition_
    );
}

void PositionMotor::update(double deltaTime) {
    if (!enabled_ || !joint_) return;
    
    // Check joint type
    dJointType type = dJointGetType(joint_);
    if (type != dJointTypeHinge) {
        // Position control only works for hinge joints
        currentEffort_ = 0.0f;
        return;
    }
    
    // Get current position and velocity
    float currentPos = getCurrentPosition();
    float currentVel = getCurrentVelocity();
    
    // PD control
    float posError = currentCommand_.targetValue - currentPos;
    float targetVel = kp_ * posError;
    
    // Clamp target velocity
    targetVel = std::clamp(targetVel, -maxVelocity_, maxVelocity_);
    
    // Apply velocity control with damping
    float velError = targetVel - currentVel;
    currentEffort_ = kd_ * velError;
    
    // Clamp effort
    float maxEff = std::min(currentCommand_.maxEffort, maxEffort_);
    currentEffort_ = std::clamp(currentEffort_, -maxEff, maxEff);
    
    // Apply to joint using velocity motor
    dJointSetHingeParam(joint_, dParamVel, targetVel);
    dJointSetHingeParam(joint_, dParamFMax, std::abs(currentEffort_));
}

float PositionMotor::getCurrentPosition() const {
    if (!joint_) return 0.0f;
    
    dJointType type = dJointGetType(joint_);
    if (type == dJointTypeHinge) {
        return dJointGetHingeAngle(joint_);
    }
    return 0.0f;
}

float PositionMotor::getCurrentVelocity() const {
    if (!joint_) return 0.0f;
    
    dJointType type = dJointGetType(joint_);
    if (type == dJointTypeHinge) {
        return dJointGetHingeAngleRate(joint_);
    }
    return 0.0f;
}

float PositionMotor::getCurrentEffort() const {
    return currentEffort_;
}

// Velocity Motor
void VelocityMotor::applyCommand(const Command& command) {
    if (!enabled_) return;
    currentCommand_ = command;
    
    // Clamp target velocity to limits
    currentCommand_.targetValue = std::clamp(
        currentCommand_.targetValue,
        -maxVelocity_,
        maxVelocity_
    );
}

void VelocityMotor::update(double deltaTime) {
    if (!enabled_ || !joint_) return;
    
    // Update noise manager for tracking time
    static int updateCounter = 0;
    static bool debugPrinted = false;
    
    // Check joint type and apply appropriate control
    dJointType type = dJointGetType(joint_);
    
    switch (type) {
        case dJointTypeHinge: {
            // For identification, use joint pointer as unique ID
            int jointIndex = reinterpret_cast<intptr_t>(joint_) & 0xFF; // Use lower bits as ID
            
            // Apply sinusoidal noise to create visible oscillation
            // Use different frequencies for different joints to create complex motion
            float baseFreq = 2.0f; // 2 Hz base frequency
            float freq = baseFreq + (jointIndex % 3) * 0.5f; // Vary frequency by joint
            float amplitude = 0.5f; // ±0.5 rad/s amplitude for more visible effect
            
            // Apply noise even when target velocity is zero to make shaking visible
            float baseVelocity = currentCommand_.targetValue;
            float noiseLevel = NoiseManager::getInstance().getNoiseLevel();
            
            if (noiseLevel > 0.0f && std::abs(baseVelocity) < 0.01f) {
                // When idle, still apply noise to make it visible
                baseVelocity = 0.0f;
            }
            
            float noisyVelocity = NoiseManager::getInstance().applySinusoidalNoise(
                baseVelocity, freq, amplitude, jointIndex);
            
            // Debug output for first few joints every second
            if (updateCounter++ % 60 == 0 && jointIndex < 3 && noiseLevel > 0.0f && !debugPrinted) {
                std::cout << "Motor " << jointIndex << " noise: base=" << baseVelocity 
                         << ", noisy=" << noisyVelocity 
                         << ", freq=" << freq << "Hz, amp=" << amplitude * noiseLevel << std::endl;
                if (jointIndex == 2) debugPrinted = true;
            }
            if (updateCounter % 60 != 0) debugPrinted = false;
            
            // Apply velocity directly for hinge joints
            dJointSetHingeParam(joint_, dParamVel, noisyVelocity);
            
            // Set max force (also with noise)
            currentEffort_ = std::min(currentCommand_.maxEffort, maxEffort_);
            float noisyEffort = NoiseManager::getInstance().applyMotorNoise(currentEffort_);
            dJointSetHingeParam(joint_, dParamFMax, std::abs(noisyEffort));
            break;
        }
            
        case dJointTypeBall:
            // Ball joints don't have direct velocity control
            // Apply torque to achieve desired angular velocity
            // For now, skip ball joints to avoid crashes
            currentEffort_ = 0.0f;
            break;
            
        default:
            currentEffort_ = 0.0f;
            break;
    }
}

float VelocityMotor::getCurrentPosition() const {
    if (!joint_) return 0.0f;
    
    dJointType type = dJointGetType(joint_);
    switch (type) {
        case dJointTypeHinge:
            return dJointGetHingeAngle(joint_);
        case dJointTypeBall:
            // Ball joints don't have a single angle
            return 0.0f;
        default:
            return 0.0f;
    }
}

float VelocityMotor::getCurrentVelocity() const {
    if (!joint_) return 0.0f;
    
    dJointType type = dJointGetType(joint_);
    switch (type) {
        case dJointTypeHinge:
            return dJointGetHingeAngleRate(joint_);
        case dJointTypeBall:
            // Ball joints don't have a single angular rate
            return 0.0f;
        default:
            return 0.0f;
    }
}

float VelocityMotor::getCurrentEffort() const {
    return currentEffort_;
}

// Torque Motor
void TorqueMotor::applyCommand(const Command& command) {
    if (!enabled_) return;
    currentCommand_ = command;
    
    // Clamp torque to limits
    float maxEff = std::min(currentCommand_.maxEffort, maxEffort_);
    currentTorque_ = std::clamp(currentCommand_.targetValue, -maxEff, maxEff);
}

void TorqueMotor::update(double deltaTime) {
    if (!enabled_ || !joint_) return;
    
    // Apply torque directly
    dJointAddHingeTorque(joint_, currentTorque_);
}

float TorqueMotor::getCurrentPosition() const {
    if (!joint_) return 0.0f;
    return dJointGetHingeAngle(joint_);
}

float TorqueMotor::getCurrentVelocity() const {
    if (!joint_) return 0.0f;
    return dJointGetHingeAngleRate(joint_);
}

float TorqueMotor::getCurrentEffort() const {
    return currentTorque_;
}

// Linear Motor
void LinearMotor::applyCommand(const Command& command) {
    if (!enabled_) return;
    currentCommand_ = command;
    
    // For slider joints, position control
    currentCommand_.targetValue = std::clamp(
        currentCommand_.targetValue,
        minPosition_,
        maxPosition_
    );
}

void LinearMotor::update(double deltaTime) {
    if (!enabled_ || !joint_) return;
    
    // Get current position
    float currentPos = dJointGetSliderPosition(joint_);
    float currentVel = dJointGetSliderPositionRate(joint_);
    
    // Simple P control for target position
    float posError = currentCommand_.targetValue - currentPos;
    float targetVel = 10.0f * posError; // P gain
    
    // Clamp velocity
    targetVel = std::clamp(targetVel, -maxVelocity_, maxVelocity_);
    
    // Apply velocity control
    dJointSetSliderParam(joint_, dParamVel, targetVel);
    
    // Set max force
    float maxEff = std::min(currentCommand_.maxEffort, maxEffort_);
    dJointSetSliderParam(joint_, dParamFMax, maxEff);
    
    currentForce_ = maxEff;
}

float LinearMotor::getCurrentPosition() const {
    if (!joint_) return 0.0f;
    return dJointGetSliderPosition(joint_);
}

float LinearMotor::getCurrentVelocity() const {
    if (!joint_) return 0.0f;
    return dJointGetSliderPositionRate(joint_);
}

float LinearMotor::getCurrentEffort() const {
    return currentForce_;
}