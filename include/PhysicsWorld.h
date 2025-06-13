#pragma once

#include <ode/ode.h>

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
    
    using vsg_vec3 = vec3;
    
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
    
#else
#include <vsg/all.h>
    using vsg_vec3 = vsg::vec3;
#endif

#include <vector>
#include <memory>

class PhysicsWorld {
public:
    struct ContactPoint {
        vsg_vec3 position;
        vsg_vec3 normal;
        float depth;
        float friction;
    };

    struct PhysicsObject {
        dBodyID body;
        dGeomID geom;
        dTriMeshDataID trimeshData; // For trimesh objects
#ifdef USE_OPENGL_FALLBACK
        ref_ptr<MatrixTransform> transform;
#else
        vsg::ref_ptr<vsg::MatrixTransform> transform;
#endif
        bool isStatic;
        
        PhysicsObject() : body(nullptr), geom(nullptr), trimeshData(nullptr), isStatic(false) {}
    };

    PhysicsWorld();
    ~PhysicsWorld();

    void step(double deltaTime);
    void setGravity(const vsg_vec3& gravity);
    void setGroundFriction(float friction) { groundFriction = friction; }
    void setGroundBounce(float bounce) { groundBounce = bounce; }
    
    // Object creation
    dBodyID createBox(const vsg_vec3& position, const vsg_vec3& size, float mass);
    dBodyID createSphere(const vsg_vec3& position, float radius, float mass);
    dBodyID createCylinder(const vsg_vec3& position, float radius, float length, float mass);
    dGeomID createStaticBox(const vsg_vec3& position, const vsg_vec3& size);
    dGeomID createStaticTrimesh(const std::vector<float>& vertices, const std::vector<int>& indices);
    
    // Advanced physics features
    void enableAdaptiveStepping(bool enable) { adaptiveStepping = enable; }
    void setContactSoftness(float softness) { contactSoftness = softness; }
    void setMaxContacts(int max) { maxContacts = max; }
    
    // Collision detection
    std::vector<ContactPoint> getContactPoints(dGeomID geom);
    bool checkRaycast(const vsg_vec3& start, const vsg_vec3& direction, float maxDistance, vsg_vec3& hitPoint);
    
    // Getters
    dWorldID getWorld() const { return world; }
    dSpaceID getSpace() const { return space; }
    dJointGroupID getContactGroup() const { return contactGroup; }
    
    // Debug visualization
    void enableDebugVisualization(bool enable) { debugVisualization = enable; }
#ifdef USE_OPENGL_FALLBACK
    ref_ptr<Group> getDebugGeometry();
#else
    vsg::ref_ptr<vsg::Group> getDebugGeometry();
#endif

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
#ifdef USE_OPENGL_FALLBACK
    ref_ptr<Group> debugGroup;
#else
    vsg::ref_ptr<vsg::Group> debugGroup;
#endif
    
    // Performance tracking
    double simulationTime = 0.0;
    double accumulator = 0.0;
    int stepCount = 0;
};