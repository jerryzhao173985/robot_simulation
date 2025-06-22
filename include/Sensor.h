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

class PhysicsWorld;
class Robot;

// Base sensor interface
class Sensor {
public:
    enum Type {
        JOINT_POSITION,
        JOINT_VELOCITY,
        JOINT_TORQUE,
        IMU,
        CONTACT,
        DISTANCE,
        FORCE_TORQUE
    };

    struct Reading {
        double timestamp;
        std::vector<float> values;
        bool valid;
    };

    Sensor(const std::string& name, Type type) 
        : name_(name), type_(type), enabled_(true) {}
    
    virtual ~Sensor() = default;

    // Update sensor reading
    virtual void update(double deltaTime) = 0;
    
    // Get latest sensor reading
    virtual Reading getReading() const = 0;
    
    // Enable/disable sensor
    void setEnabled(bool enabled) { enabled_ = enabled; }
    bool isEnabled() const { return enabled_; }
    
    // Get sensor info
    const std::string& getName() const { return name_; }
    Type getType() const { return type_; }
    
    // Set noise parameters for realistic simulation
    void setNoise(float mean, float stddev) {
        noiseMean_ = mean;
        noiseStddev_ = stddev;
    }
    
protected:
    std::string name_;
    Type type_;
    bool enabled_;
    float noiseMean_ = 0.0f;
    float noiseStddev_ = 0.0f;
    
    // Add noise to sensor reading
    float addNoise(float value);
};

// Joint position sensor
class JointPositionSensor : public Sensor {
public:
    JointPositionSensor(const std::string& name, dJointID joint)
        : Sensor(name, JOINT_POSITION), joint_(joint) {}
    
    void update(double deltaTime) override;
    Reading getReading() const override;
    
private:
    dJointID joint_;
    float position_ = 0.0f;
};

// Joint velocity sensor
class JointVelocitySensor : public Sensor {
public:
    JointVelocitySensor(const std::string& name, dJointID joint)
        : Sensor(name, JOINT_VELOCITY), joint_(joint) {}
    
    void update(double deltaTime) override;
    Reading getReading() const override;
    
private:
    dJointID joint_;
    float velocity_ = 0.0f;
};

// IMU sensor (measures orientation and angular velocity)
class IMUSensor : public Sensor {
public:
    IMUSensor(const std::string& name, dBodyID body)
        : Sensor(name, IMU), body_(body) {}
    
    void update(double deltaTime) override;
    Reading getReading() const override;
    
private:
    dBodyID body_;
    vsg_quat orientation_;
    vsg_vec3 angularVelocity_;
    vsg_vec3 linearAcceleration_;
};

// Contact sensor (detects foot contacts)
class ContactSensor : public Sensor {
public:
    ContactSensor(const std::string& name, dGeomID geom, PhysicsWorld* world)
        : Sensor(name, CONTACT), geom_(geom), physicsWorld_(world) {}
    
    void update(double deltaTime) override;
    Reading getReading() const override;
    
    void setPhysicsWorld(PhysicsWorld* world) { physicsWorld_ = world; }
    
private:
    dGeomID geom_;
    PhysicsWorld* physicsWorld_;
    bool hasContact_ = false;
    vsg_vec3 contactNormal_;
    float contactForce_ = 0.0f;
};

// Distance sensor (simple lidar/ultrasonic)
class DistanceSensor : public Sensor {
public:
    DistanceSensor(const std::string& name, dBodyID body, 
                   const vsg_vec3& localPos, const vsg_vec3& direction,
                   float maxRange = 10.0f)
        : Sensor(name, DISTANCE), body_(body), 
          localPosition_(localPos), direction_(direction), 
          maxRange_(maxRange) {}
    
    void update(double deltaTime) override;
    Reading getReading() const override;
    
    void setPhysicsWorld(PhysicsWorld* world) { physicsWorld_ = world; }
    
private:
    dBodyID body_;
    vsg_vec3 localPosition_;
    vsg_vec3 direction_;
    float maxRange_;
    float distance_ = -1.0f;
    PhysicsWorld* physicsWorld_ = nullptr;
};

// Force/Torque sensor (measures forces at joint)
class ForceTorqueSensor : public Sensor {
public:
    ForceTorqueSensor(const std::string& name, dJointID joint)
        : Sensor(name, FORCE_TORQUE), joint_(joint) {}
    
    void update(double deltaTime) override;
    Reading getReading() const override;
    
private:
    dJointID joint_;
    vsg_vec3 force_;
    vsg_vec3 torque_;
};