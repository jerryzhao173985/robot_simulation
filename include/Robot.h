#pragma once

#ifdef USE_OPENGL_FALLBACK
    #include "FallbackTypes.h"
    using vsg_vec3 = vsg::vec3;
    using vsg_vec4 = vsg::vec4;
    using vsg_quat = vsg::quat;
    
#else
#include <vsg/all.h>
    using vsg_vec3 = vsg::vec3;
    using vsg_vec4 = vsg::vec4;
    using vsg_quat = vsg::quat;
#endif

#include <ode/ode.h>
#include <vector>
#include <memory>
#include <array>
#include <mutex>

class Robot {
public:
    // Leg segment indices for clear identification and reduced indexing errors
    enum class LegSegmentIndex : int {
        COXA = 0,   // Hip segment - connects to body via ball joint
        FEMUR = 1,  // Thigh segment - connects to coxa via hinge joint
        TIBIA = 2   // Shin/foot segment - connects to femur via hinge joint, used for ground contact
    };
    
    struct LegSegment {
        dBodyID body;
        dGeomID geom;
        dGeomID footGeom;  // Optional foot box geometry for tibia segments
        dJointID joint;
#ifdef USE_OPENGL_FALLBACK
        ref_ptr<MatrixTransform> transform;
#else
        vsg::ref_ptr<vsg::MatrixTransform> transform;
#endif
        float length;
        float radius;
    };

    struct Leg {
        std::vector<LegSegment> segments;  // [COXA=0, FEMUR=1, TIBIA=2] - anatomical order from body to foot
        vsg_vec3 attachmentPoint;
        float targetAngle;
        float currentAngle;
        bool isGrounded;
    };

    struct Sensor {
        enum Type { PROXIMITY, GYROSCOPE, ACCELEROMETER, CONTACT };
        Type type;
        vsg_vec3 position;
        vsg_vec3 orientation;
        float range;
        float currentValue;
    };

#ifdef USE_OPENGL_FALLBACK
    Robot(dWorldID world, dSpaceID space, ref_ptr<Group> sceneGraph);
#else
    Robot(dWorldID world, dSpaceID space, vsg::ref_ptr<vsg::Group> sceneGraph);
#endif
    ~Robot();

    void update(double deltaTime);
    void updateLegPositions();
    void applyControl(const std::vector<float>& motorCommands);
    void reset();
    
    // Advanced features
    void enableAdaptiveGait(bool enable) { adaptiveGaitEnabled = enable; }
    void setTerrainAdaptation(bool enable) { terrainAdaptationEnabled = enable; }
    void enableStabilization(bool enable) { stabilizationEnabled = enable; }
    
    // Getters
    vsg_vec3 getPosition() const;
    vsg_vec3 getVelocity() const;
    vsg_quat getOrientation() const;
    std::vector<float> getSensorReadings() const;
    float getEnergyConsumption() const { return energyConsumption; }
    bool isStable() const;
    
    // ODE body & geometry handles (for contact-based control)
    dBodyID getBody() const;
    dGeomID getBodyGeom() const;
    // Get all foot geometry IDs - specifically tibia segments used for ground contact sensing
    // Note: Only tibia segments (index 2) are returned as they represent the feet in hexapod anatomy
    std::vector<dGeomID> getFootGeoms() const;  // Returns actual foot box geometries
    
    // Visual customization
    void setBodyColor(const vsg_vec4& color);
    void setMetallic(float metallic) { metallicValue = metallic; }
    void setRoughness(float roughness) { roughnessValue = roughness; }

    // Visual representation
#ifndef USE_OPENGL_FALLBACK
    vsg::ref_ptr<vsg::MatrixTransform> bodyTransform;
    vsg::ref_ptr<vsg::MatrixTransform> robotTransform;
    std::vector<vsg::ref_ptr<vsg::MatrixTransform>> legTransforms;
    std::vector<vsg::ref_ptr<vsg::MatrixTransform>> legNodes; // one per segment
    void createVisualModel();
    vsg::ref_ptr<vsg::MatrixTransform> getRobotNode();
    void addToScene(vsg::ref_ptr<vsg::Group> scene);
    vsg_vec3 eulerAnglesFromQuat(const vsg_quat& q) const;
#else
    ref_ptr<MatrixTransform> bodyTransform;
    ref_ptr<MatrixTransform> robotTransform;
    std::vector<ref_ptr<MatrixTransform>> legTransforms;
    
    void createVisualModel();
    ref_ptr<MatrixTransform> getRobotNode();
    void addToScene(ref_ptr<Group> scene);
    vsg_vec3 eulerAnglesFromQuat(const vsg_quat& q) const;
#endif

private:
    void createBody();
    void createLegs();
    vsg_vec3 getLegPosition(int legIndex) const;
    void applyForces();
    void maintainBalance();
    
    std::mutex navigationMutex;

    // ODE simulation context
    dWorldID world;
    dSpaceID space;

    // Physics bodies
    dBodyID bodyId;
    dGeomID bodyGeom;    // box geometry for main body (used for contact queries)
    
    // Visual representation
#ifdef USE_OPENGL_FALLBACK
    ref_ptr<Group> robotGroup;
    ref_ptr<Group> sceneGraph;
#else
    vsg::ref_ptr<vsg::Group> robotGroup;
    vsg::ref_ptr<vsg::Group> sceneGraph;
#endif
    
    // Robot configuration - ANATOMICALLY CORRECT HEXAPOD DIMENSIONS
    // These values are carefully chosen to match real hexapod proportions and ensure stable locomotion
    struct Config {
        // Main body dimensions (matching Visualizer for perfect sync)
        vsg_vec3 bodySize = vsg_vec3(1.2f, 0.6f, 0.3f);     // Length x Width x Height
        
        // Leg segment dimensions (realistic hexapod anatomy with proper proportions)
        // Based on typical arthropod leg ratios: coxa < femur ≈ tibia for optimal reach and stability
        float coxaLength = 0.25f;    // Hip segment - shortest, provides body attachment and outward extension
        float femurLength = 0.35f;   // Thigh segment - primary support, angled downward for height
        float tibiaLength = 0.4f;    // Shin/foot segment - longest for ground reach and contact surface
        float legRadius = 0.04f;     // Consistent segment thickness for uniform appearance
        
        // Contact properties (tibia end serves as the foot contact point)
        float footRadius = 0.05f;    // Foot contact radius for ground interaction
    } config;
    
    static constexpr int NUM_LEGS = 6;
    static constexpr int SEGMENTS_PER_LEG = 3;  // coxa, femur, tibia (anatomical order)
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
    vsg_vec3 targetVelocity;
    float targetAngularVelocity = 0.0f;
    
    // Visual parameters
    vsg_vec4 bodyColor = vsg_vec4(0.3f, 0.3f, 0.4f, 1.0f);
    float metallicValue = 0.8f;
    float roughnessValue = 0.3f;
};