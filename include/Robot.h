#pragma once

#include <vsg/all.h>
#include <ode/ode.h>
#include <vector>
#include <memory>
#include <array>

class Robot {
public:
    struct LegSegment {
        dBodyID body;
        dGeomID geom;
        dJointID joint;
        vsg::ref_ptr<vsg::MatrixTransform> transform;
        float length;
        float radius;
    };

    struct Leg {
        std::vector<LegSegment> segments;
        vsg::vec3 attachmentPoint;
        float targetAngle;
        float currentAngle;
        bool isGrounded;
    };

    struct Sensor {
        enum Type { PROXIMITY, GYROSCOPE, ACCELEROMETER, CONTACT };
        Type type;
        vsg::vec3 position;
        vsg::vec3 orientation;
        float range;
        float currentValue;
    };

    Robot(dWorldID world, dSpaceID space, vsg::ref_ptr<vsg::Group> sceneGraph);
    ~Robot();

    void update(double deltaTime);
    void applyControl(const std::vector<float>& motorCommands);
    void reset();
    
    // Advanced features
    void enableAdaptiveGait(bool enable) { adaptiveGaitEnabled = enable; }
    void setTerrainAdaptation(bool enable) { terrainAdaptationEnabled = enable; }
    void enableStabilization(bool enable) { stabilizationEnabled = enable; }
    
    // Getters
    vsg::vec3 getPosition() const;
    vsg::vec3 getVelocity() const;
    vsg::quat getOrientation() const;
    std::vector<float> getSensorReadings() const;
    float getEnergyConsumption() const { return energyConsumption; }
    bool isStable() const;
    
    // Visual customization
    void setBodyColor(const vsg::vec4& color);
    void setMetallic(float metallic) { metallicValue = metallic; }
    void setRoughness(float roughness) { roughnessValue = roughness; }

private:
    void createBody();
    void createLegs();
    void createLegSegments(int legIndex);
    void createSensors();
    void updateVisuals();
    void updatePhysics(double deltaTime);
    void calculateInverseKinematics(int legIndex, const vsg::vec3& targetPos);
    void updateGait(double time);
    void detectGround();
    void stabilize();
    void addBodyDecorations();
    vsg::vec3 eulerAnglesFromQuat(const vsg::quat& q) const;
    
    // Physics components
    dWorldID world;
    dSpaceID space;
    dBodyID bodyID;
    dGeomID bodyGeom;
    dMass mass;
    
    // Visual components
    vsg::ref_ptr<vsg::Group> robotGroup;
    vsg::ref_ptr<vsg::Group> sceneGraph;
    vsg::ref_ptr<vsg::MatrixTransform> bodyTransform;
    vsg::ref_ptr<vsg::PhongMaterialValue> material;
    
    // Robot configuration
    static constexpr int NUM_LEGS = 6;
    static constexpr int SEGMENTS_PER_LEG = 3;
    std::array<Leg, NUM_LEGS> legs;
    std::vector<Sensor> sensors;
    
    // Control parameters
    float moveSpeed = 2.0f;
    float turnSpeed = 1.0f;
    float stepHeight = 0.3f;
    float strideLength = 0.5f;
    
    // Gait parameters
    float gaitPhase = 0.0f;
    float gaitSpeed = 1.0f;
    int currentGaitPattern = 0;
    
    // Advanced features flags
    bool adaptiveGaitEnabled = true;
    bool terrainAdaptationEnabled = true;
    bool stabilizationEnabled = true;
    
    // State tracking
    float energyConsumption = 0.0f;
    vsg::vec3 targetVelocity;
    float targetAngularVelocity = 0.0f;
    
    // Visual parameters
    vsg::vec4 bodyColor = vsg::vec4(0.3f, 0.3f, 0.4f, 1.0f);
    float metallicValue = 0.8f;
    float roughnessValue = 0.3f;
};