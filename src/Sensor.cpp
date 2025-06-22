#include "Sensor.h"
#include "PhysicsWorld.h"
#include "NoiseManager.h"
#include <random>
#include <chrono>

// Random number generator for sensor noise (kept for backward compatibility)
static std::random_device rd;
static std::mt19937 gen(rd());

float Sensor::addNoise(float value) {
    // Use NoiseManager if available, otherwise fall back to old implementation
    // This method is now mostly unused as noise is applied per sensor type
    if (noiseStddev_ > 0.0f) {
        std::normal_distribution<float> dist(noiseMean_, noiseStddev_);
        return value + dist(gen);
    }
    return value;
}

// Joint Position Sensor
void JointPositionSensor::update(double deltaTime) {
    if (!enabled_ || !joint_) return;
    
    // Get joint angle based on joint type
    dJointType type = dJointGetType(joint_);
    switch (type) {
        case dJointTypeHinge:
            position_ = dJointGetHingeAngle(joint_);
            break;
        case dJointTypeSlider:
            position_ = dJointGetSliderPosition(joint_);
            break;
        case dJointTypeUniversal:
            position_ = dJointGetUniversalAngle1(joint_);
            break;
        case dJointTypeBall:
            // Ball joints don't have a single angle - return 0
            position_ = 0.0f;
            break;
        default:
            position_ = 0.0f;
    }
    
    // Apply noise using NoiseManager
    position_ = NoiseManager::getInstance().applyJointPositionNoise(position_);
}

Sensor::Reading JointPositionSensor::getReading() const {
    Reading reading;
    reading.timestamp = std::chrono::system_clock::now().time_since_epoch().count() / 1e9;
    reading.values.push_back(position_);
    reading.valid = enabled_;
    return reading;
}

// Joint Velocity Sensor
void JointVelocitySensor::update(double deltaTime) {
    if (!enabled_ || !joint_) return;
    
    dJointType type = dJointGetType(joint_);
    switch (type) {
        case dJointTypeHinge:
            velocity_ = dJointGetHingeAngleRate(joint_);
            break;
        case dJointTypeSlider:
            velocity_ = dJointGetSliderPositionRate(joint_);
            break;
        case dJointTypeUniversal:
            velocity_ = dJointGetUniversalAngle1Rate(joint_);
            break;
        case dJointTypeBall:
            // Ball joints don't have a single angular rate - return 0
            velocity_ = 0.0f;
            break;
        default:
            velocity_ = 0.0f;
    }
    
    // Apply noise using NoiseManager
    velocity_ = NoiseManager::getInstance().applyJointVelocityNoise(velocity_);
}

Sensor::Reading JointVelocitySensor::getReading() const {
    Reading reading;
    reading.timestamp = std::chrono::system_clock::now().time_since_epoch().count() / 1e9;
    reading.values.push_back(velocity_);
    reading.valid = enabled_;
    return reading;
}

// IMU Sensor
void IMUSensor::update(double deltaTime) {
    if (!enabled_ || !body_) return;
    
    // Get orientation quaternion
    const dReal* q = dBodyGetQuaternion(body_);
    orientation_ = vsg_quat(q[1], q[2], q[3], q[0]); // ODE uses WXYZ, VSG uses XYZW
    
    // Get angular velocity
    const dReal* w = dBodyGetAngularVel(body_);
    angularVelocity_ = vsg_vec3(w[0], w[1], w[2]);
    
    // Get linear acceleration (would need to differentiate velocity)
    const dReal* v = dBodyGetLinearVel(body_);
    static vsg_vec3 lastVel(0, 0, 0);
    if (deltaTime > 0) {
        linearAcceleration_ = (vsg_vec3(v[0], v[1], v[2]) - lastVel) / float(deltaTime);
        lastVel = vsg_vec3(v[0], v[1], v[2]);
    }
    
    // Add noise to readings using NoiseManager
    auto& noiseManager = NoiseManager::getInstance();
    angularVelocity_.x = noiseManager.applyIMUNoise(angularVelocity_.x, true);
    angularVelocity_.y = noiseManager.applyIMUNoise(angularVelocity_.y, true);
    angularVelocity_.z = noiseManager.applyIMUNoise(angularVelocity_.z, true);
    
    // Also add small noise to orientation
    orientation_.x = noiseManager.applyIMUNoise(orientation_.x, false);
    orientation_.y = noiseManager.applyIMUNoise(orientation_.y, false);
    orientation_.z = noiseManager.applyIMUNoise(orientation_.z, false);
}

Sensor::Reading IMUSensor::getReading() const {
    Reading reading;
    reading.timestamp = std::chrono::system_clock::now().time_since_epoch().count() / 1e9;
    
    // Pack quaternion (4 values)
    reading.values.push_back(orientation_.x);
    reading.values.push_back(orientation_.y);
    reading.values.push_back(orientation_.z);
    reading.values.push_back(orientation_.w);
    
    // Pack angular velocity (3 values)
    reading.values.push_back(angularVelocity_.x);
    reading.values.push_back(angularVelocity_.y);
    reading.values.push_back(angularVelocity_.z);
    
    // Pack linear acceleration (3 values)
    reading.values.push_back(linearAcceleration_.x);
    reading.values.push_back(linearAcceleration_.y);
    reading.values.push_back(linearAcceleration_.z);
    
    reading.valid = enabled_;
    return reading;
}

// Contact Sensor
void ContactSensor::update(double deltaTime) {
    if (!enabled_ || !geom_ || !physicsWorld_) return;
    
    auto contacts = physicsWorld_->getContactPoints(geom_);
    hasContact_ = !contacts.empty();
    
    if (hasContact_) {
        // Average contact normal and force
        contactNormal_ = vsg_vec3(0, 0, 0);
        contactForce_ = 0.0f;
        
        for (const auto& contact : contacts) {
            contactNormal_ = contactNormal_ + contact.normal;
            contactForce_ += contact.depth * 1000.0f; // Simple force estimate
        }
        
        if (contacts.size() > 0) {
            contactNormal_ = vsg::normalize(contactNormal_);
            contactForce_ /= contacts.size();
        }
    } else {
        contactNormal_ = vsg_vec3(0, 0, 1);
        contactForce_ = 0.0f;
    }
    
    // Apply noise to contact force
    contactForce_ = NoiseManager::getInstance().applyContactNoise(contactForce_, true);
    
    // Also potentially flip contact state due to noise
    if (NoiseManager::getInstance().getNoiseLevel() > 0.0f) {
        float contactState = hasContact_ ? 1.0f : 0.0f;
        float noisyState = NoiseManager::getInstance().applyContactNoise(contactState, false);
        hasContact_ = noisyState > 0.5f;
    }
}

Sensor::Reading ContactSensor::getReading() const {
    Reading reading;
    reading.timestamp = std::chrono::system_clock::now().time_since_epoch().count() / 1e9;
    
    reading.values.push_back(hasContact_ ? 1.0f : 0.0f);
    reading.values.push_back(contactNormal_.x);
    reading.values.push_back(contactNormal_.y);
    reading.values.push_back(contactNormal_.z);
    reading.values.push_back(contactForce_);
    
    reading.valid = enabled_;
    return reading;
}

// Distance Sensor
void DistanceSensor::update(double deltaTime) {
    if (!enabled_ || !body_ || !physicsWorld_) return;
    
    // Get sensor position in world coordinates
    const dReal* pos = dBodyGetPosition(body_);
    const dReal* rot = dBodyGetRotation(body_);
    
    // Transform local position to world
    dVector3 worldPos;
    dBodyGetRelPointPos(body_, localPosition_.x, localPosition_.y, localPosition_.z, worldPos);
    
    // Transform direction to world
    dVector3 worldDir;
    dBodyVectorToWorld(body_, direction_.x, direction_.y, direction_.z, worldDir);
    
    // Perform ray cast
    PhysicsWorld::RaycastResult result = physicsWorld_->raycast(
        vsg_vec3(worldPos[0], worldPos[1], worldPos[2]),
        vsg_vec3(worldDir[0], worldDir[1], worldDir[2]),
        maxRange_
    );
    
    if (result.hit) {
        distance_ = result.distance;
    } else {
        distance_ = maxRange_;
    }
    
    // Apply proportional noise to distance reading
    if (distance_ >= 0) {
        distance_ = NoiseManager::getInstance().applyContactNoise(distance_, true);
    }
}

Sensor::Reading DistanceSensor::getReading() const {
    Reading reading;
    reading.timestamp = std::chrono::system_clock::now().time_since_epoch().count() / 1e9;
    reading.values.push_back(distance_);
    reading.valid = enabled_ && distance_ >= 0;
    return reading;
}

// Force/Torque Sensor
void ForceTorqueSensor::update(double deltaTime) {
    if (!enabled_ || !joint_) return;
    
    // Get joint feedback (requires enabling feedback on joint)
    dJointFeedback* feedback = dJointGetFeedback(joint_);
    if (feedback) {
        force_ = vsg_vec3(feedback->f1[0], feedback->f1[1], feedback->f1[2]);
        torque_ = vsg_vec3(feedback->t1[0], feedback->t1[1], feedback->t1[2]);
        
        // Add noise
        force_.x = addNoise(force_.x);
        force_.y = addNoise(force_.y);
        force_.z = addNoise(force_.z);
        torque_.x = addNoise(torque_.x);
        torque_.y = addNoise(torque_.y);
        torque_.z = addNoise(torque_.z);
    } else {
        force_ = vsg_vec3(0, 0, 0);
        torque_ = vsg_vec3(0, 0, 0);
    }
}

Sensor::Reading ForceTorqueSensor::getReading() const {
    Reading reading;
    reading.timestamp = std::chrono::system_clock::now().time_since_epoch().count() / 1e9;
    
    reading.values.push_back(force_.x);
    reading.values.push_back(force_.y);
    reading.values.push_back(force_.z);
    reading.values.push_back(torque_.x);
    reading.values.push_back(torque_.y);
    reading.values.push_back(torque_.z);
    
    reading.valid = enabled_;
    return reading;
}