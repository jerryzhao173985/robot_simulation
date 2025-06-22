#pragma once

#include <memory>
#include <string>
#include <vector>
#include <functional>
#include <ode/ode.h>

#ifdef USE_OPENGL_FALLBACK
#include "FallbackTypes.h"
using vsg_vec3 = vec3;
using vsg_vec4 = vec4;
using vsg_quat = quat;
#else
#include <vsg/maths/vec3.h>
#include <vsg/maths/vec4.h>
#include <vsg/maths/quat.h>
using vsg_vec3 = vsg::vec3;
using vsg_vec4 = vsg::vec4;
using vsg_quat = vsg::quat;
#endif

// Base actuator interface
class Actuator {
public:
    enum Type {
        POSITION_MOTOR,
        VELOCITY_MOTOR,
        TORQUE_MOTOR,
        LINEAR_MOTOR
    };
    
    struct Command {
        double timestamp;
        float targetValue;
        float maxEffort;  // Maximum torque/force to apply
    };
    
    Actuator(const std::string& name, Type type)
        : name_(name), type_(type), enabled_(true) {}
    
    virtual ~Actuator() = default;
    
    // Apply command to actuator
    virtual void applyCommand(const Command& command) = 0;
    
    // Update actuator (called each physics step)
    virtual void update(double deltaTime) = 0;
    
    // Get current actuator state
    virtual float getCurrentPosition() const = 0;
    virtual float getCurrentVelocity() const = 0;
    virtual float getCurrentEffort() const = 0;
    
    // Enable/disable actuator
    void setEnabled(bool enabled) { enabled_ = enabled; }
    bool isEnabled() const { return enabled_; }
    
    // Get actuator info
    const std::string& getName() const { return name_; }
    Type getType() const { return type_; }
    
    // Set actuator limits
    void setPositionLimits(float min, float max) {
        minPosition_ = min;
        maxPosition_ = max;
    }
    
    void setVelocityLimit(float limit) {
        maxVelocity_ = limit;
    }
    
    void setEffortLimit(float limit) {
        maxEffort_ = limit;
    }
    
protected:
    std::string name_;
    Type type_;
    bool enabled_;
    
    float minPosition_ = -M_PI;
    float maxPosition_ = M_PI;
    float maxVelocity_ = 10.0f;
    float maxEffort_ = 10.0f;
    
    Command currentCommand_;
};

// Position-controlled motor (servo)
class PositionMotor : public Actuator {
public:
    PositionMotor(const std::string& name, dJointID joint,
                  float kp = 10.0f, float kd = 1.0f)
        : Actuator(name, POSITION_MOTOR), joint_(joint), kp_(kp), kd_(kd) {}
    
    void applyCommand(const Command& command) override;
    void update(double deltaTime) override;
    
    float getCurrentPosition() const override;
    float getCurrentVelocity() const override;
    float getCurrentEffort() const override;
    
    // Set PID gains
    void setGains(float kp, float kd) { kp_ = kp; kd_ = kd; }
    
private:
    dJointID joint_;
    float kp_, kd_;
    float currentEffort_ = 0.0f;
};

// Velocity-controlled motor
class VelocityMotor : public Actuator {
public:
    VelocityMotor(const std::string& name, dJointID joint)
        : Actuator(name, VELOCITY_MOTOR), joint_(joint) {}
    
    void applyCommand(const Command& command) override;
    void update(double deltaTime) override;
    
    float getCurrentPosition() const override;
    float getCurrentVelocity() const override;
    float getCurrentEffort() const override;
    
private:
    dJointID joint_;
    float currentEffort_ = 0.0f;
};

// Direct torque motor
class TorqueMotor : public Actuator {
public:
    TorqueMotor(const std::string& name, dJointID joint)
        : Actuator(name, TORQUE_MOTOR), joint_(joint) {}
    
    void applyCommand(const Command& command) override;
    void update(double deltaTime) override;
    
    float getCurrentPosition() const override;
    float getCurrentVelocity() const override;
    float getCurrentEffort() const override;
    
private:
    dJointID joint_;
    float currentTorque_ = 0.0f;
};

// Linear motor (for prismatic joints)
class LinearMotor : public Actuator {
public:
    LinearMotor(const std::string& name, dJointID joint)
        : Actuator(name, LINEAR_MOTOR), joint_(joint) {}
    
    void applyCommand(const Command& command) override;
    void update(double deltaTime) override;
    
    float getCurrentPosition() const override;
    float getCurrentVelocity() const override;
    float getCurrentEffort() const override;
    
private:
    dJointID joint_;
    float currentForce_ = 0.0f;
};