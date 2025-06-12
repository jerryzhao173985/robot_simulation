#pragma once

#include <ode/ode.h>
#include <vsg/all.h>
#include <vector>
#include <memory>

class PhysicsWorld {
public:
    struct ContactPoint {
        vsg::vec3 position;
        vsg::vec3 normal;
        float depth;
        float friction;
    };

    struct PhysicsObject {
        dBodyID body;
        dGeomID geom;
        vsg::ref_ptr<vsg::MatrixTransform> transform;
        bool isStatic;
    };

    PhysicsWorld();
    ~PhysicsWorld();

    void step(double deltaTime);
    void setGravity(const vsg::vec3& gravity);
    void setGroundFriction(float friction) { groundFriction = friction; }
    void setGroundBounce(float bounce) { groundBounce = bounce; }
    
    // Object creation
    dBodyID createBox(const vsg::vec3& position, const vsg::vec3& size, float mass);
    dBodyID createSphere(const vsg::vec3& position, float radius, float mass);
    dBodyID createCylinder(const vsg::vec3& position, float radius, float length, float mass);
    dGeomID createStaticBox(const vsg::vec3& position, const vsg::vec3& size);
    dGeomID createStaticTrimesh(const std::vector<float>& vertices, const std::vector<int>& indices);
    
    // Advanced physics features
    void enableAdaptiveStepping(bool enable) { adaptiveStepping = enable; }
    void setContactSoftness(float softness) { contactSoftness = softness; }
    void setMaxContacts(int max) { maxContacts = max; }
    
    // Collision detection
    std::vector<ContactPoint> getContactPoints(dGeomID geom);
    bool checkRaycast(const vsg::vec3& start, const vsg::vec3& direction, float maxDistance, vsg::vec3& hitPoint);
    
    // Getters
    dWorldID getWorld() const { return world; }
    dSpaceID getSpace() const { return space; }
    dJointGroupID getContactGroup() const { return contactGroup; }
    
    // Debug visualization
    void enableDebugVisualization(bool enable) { debugVisualization = enable; }
    vsg::ref_ptr<vsg::Group> getDebugGeometry();

private:
    static void nearCallback(void* data, dGeomID o1, dGeomID o2);
    void handleCollision(dGeomID o1, dGeomID o2);
    void updateTransforms();
    
    // ODE components
    dWorldID world;
    dSpaceID space;
    dJointGroupID contactGroup;
    dGeomID groundPlane;
    
    // Physics parameters
    float groundFriction = 1.0f;
    float groundBounce = 0.1f;
    float contactSoftness = 0.001f;
    int maxContacts = 64;
    double stepSize = 0.01;
    bool adaptiveStepping = true;
    
    // Object tracking
    std::vector<PhysicsObject> objects;
    std::vector<ContactPoint> activeContacts;
    
    // Debug visualization
    bool debugVisualization = false;
    vsg::ref_ptr<vsg::Group> debugGroup;
    
    // Performance tracking
    double simulationTime = 0.0;
    double accumulator = 0.0;
    int stepCount = 0;
};