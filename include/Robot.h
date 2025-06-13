#pragma once

#ifdef USE_OPENGL_FALLBACK
    #include <cmath>
    
    // Simple vector classes for fallback mode
    struct vec3 {
        float x, y, z;
        vec3(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}
        vec3 operator+(const vec3& other) const { return vec3(x + other.x, y + other.y, z + other.z); }
        vec3 operator-(const vec3& other) const { return vec3(x - other.x, y - other.y, z - other.z); }
        vec3 operator*(float s) const { return vec3(x * s, y * s, z * s); }
        float length() const { return std::sqrt(x*x + y*y + z*z); }
        vec3 normalize() const { float l = length(); return l > 0 ? *this * (1.0f/l) : vec3(); }
    };
    
    struct vec4 {
        float x, y, z, w;
        vec4(float x = 0, float y = 0, float z = 0, float w = 1) : x(x), y(y), z(z), w(w) {}
    };
    
    struct quat {
        float x, y, z, w;
        quat(float x = 0, float y = 0, float z = 0, float w = 1) : x(x), y(y), z(z), w(w) {}
    };
    
    using vsg_vec3 = vec3;
    using vsg_vec4 = vec4;
    using vsg_quat = quat;
    
    template<typename T>
    struct ref_ptr {
        T* ptr = nullptr;
        ref_ptr() = default;
        ref_ptr(T* p) : ptr(p) {}
        T* operator->() { return ptr; }
        const T* operator->() const { return ptr; }
        T& operator*() { return *ptr; }
        const T& operator*() const { return *ptr; }
        operator bool() const { return ptr != nullptr; }
    };
    
    class Group {
    public:
        std::vector<void*> children;
        void addChild(void* child) { children.push_back(child); }
    };
    
    class MatrixTransform {};
    class PhongMaterialValue {};
    
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
    struct LegSegment {
        dBodyID body;
        dGeomID geom;
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
        std::vector<LegSegment> segments;
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
    
    // Visual customization
    void setBodyColor(const vsg_vec4& color);
    void setMetallic(float metallic) { metallicValue = metallic; }
    void setRoughness(float roughness) { roughnessValue = roughness; }

    // Visual representation
#ifndef USE_OPENGL_FALLBACK
    vsg::ref_ptr<vsg::MatrixTransform> bodyTransform;
    vsg::ref_ptr<vsg::MatrixTransform> robotTransform;
    std::vector<vsg::ref_ptr<vsg::MatrixTransform>> legTransforms;
    
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
    void updateLegPositions();
    vsg_vec3 getLegPosition(int legIndex) const;
    void applyForces();
    void maintainBalance();
    
    std::mutex navigationMutex;
    
    // Physics bodies
    dBodyID bodyId;
    std::vector<dBodyID> legBodies;
    std::vector<dJointID> legJoints;
    
    // Visual representation
#ifdef USE_OPENGL_FALLBACK
    ref_ptr<Group> robotGroup;
    ref_ptr<Group> sceneGraph;
#else
    vsg::ref_ptr<vsg::Group> robotGroup;
    vsg::ref_ptr<vsg::Group> sceneGraph;
#endif
    
    // Robot configuration
    struct Config {
        vsg_vec3 bodySize = vsg_vec3(1.2f, 0.6f, 0.3f);     // Match Visualizer hexapod body
        float upperLegLength = 0.3f;
        float lowerLegLength = 0.25f;
        float legRadius = 0.02f;
        float footRadius = 0.03f;
        float legAttachOffset = 0.3f;                        // Match leg spacing
    } config;
    
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
    vsg_vec3 targetVelocity;
    float targetAngularVelocity = 0.0f;
    
    // Visual parameters
    vsg_vec4 bodyColor = vsg_vec4(0.3f, 0.3f, 0.4f, 1.0f);
    float metallicValue = 0.8f;
    float roughnessValue = 0.3f;
};