#include "PhysicsWorld.h"
#include <iostream>
#include <algorithm>

PhysicsWorld::PhysicsWorld() {
    // Create ODE world
    world = dWorldCreate();
    space = dHashSpaceCreate(0);
    contactGroup = dJointGroupCreate(0);
    
    // Set default parameters
    dWorldSetGravity(world, 0, -9.81f, 0);
    dWorldSetERP(world, 0.2f);
    dWorldSetCFM(world, 1e-5);
    dWorldSetContactMaxCorrectingVel(world, 0.9f);
    dWorldSetContactSurfaceLayer(world, 0.001f);
    dWorldSetAutoDisableFlag(world, 1);
    dWorldSetQuickStepNumIterations(world, 20);
    
    // Create ground plane
    groundPlane = dCreatePlane(space, 0, 1, 0, 0);
    
    // Initialize debug group
    debugGroup = vsg::Group::create();
}

PhysicsWorld::~PhysicsWorld() {
    // Disable ODE error callback to prevent crashes during cleanup
    dSetErrorHandler(nullptr);
    dSetDebugHandler(nullptr);
    dSetMessageHandler(nullptr);
    
    try {
        // Clean up in the correct ODE order to prevent crashes
        
        // 1. Empty contact group first (destroy all joints)
        if (contactGroup) {
            dJointGroupEmpty(contactGroup);
            dJointGroupDestroy(contactGroup);
            contactGroup = nullptr;
        }
        
        // 2. Destroy all bodies first (they reference geometries)
        for (auto& obj : objects) {
            if (obj.body && !obj.isStatic) {
                dBodyDestroy(obj.body);
                obj.body = nullptr;
            }
        }
        
        // 3. Destroy all geometries (they reference the space)
        for (auto& obj : objects) {
            if (obj.geom) {
                dGeomDestroy(obj.geom);
                obj.geom = nullptr;
            }
            // Clean up trimesh data if present
            if (obj.trimeshData) {
                dGeomTriMeshDataDestroy(obj.trimeshData);
                obj.trimeshData = nullptr;
            }
        }
        
        // 4. Destroy ground plane
        if (groundPlane) {
            dGeomDestroy(groundPlane);
            groundPlane = nullptr;
        }
        
        // 5. Destroy space (it references the world)
        if (space) {
            dSpaceDestroy(space);
            space = nullptr;
        }
        
        // 6. Finally destroy world
        if (world) {
            dWorldDestroy(world);
            world = nullptr;
        }
        
        // Clear objects vector
        objects.clear();
    } catch (...) {
        // Silently catch any ODE cleanup exceptions
        std::cerr << "Warning: Exception during ODE cleanup (this is a known ODE issue)" << std::endl;
    }
}

void PhysicsWorld::nearCallback(void* data, dGeomID o1, dGeomID o2) {
    PhysicsWorld* pw = static_cast<PhysicsWorld*>(data);
    pw->handleCollision(o1, o2);
}

void PhysicsWorld::handleCollision(dGeomID o1, dGeomID o2) {
    // Check if both geometries have bodies
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    
    // Exit if both bodies are connected by a joint
    if (b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) {
        return;
    }
    
    // Maximum number of contact points
    const int maxContacts = 32;
    dContact contact[maxContacts];
    
    // Get contact points
    int numContacts = dCollide(o1, o2, maxContacts, &contact[0].geom, sizeof(dContact));
    
    if (numContacts > 0) {
        for (int i = 0; i < numContacts; ++i) {
            // Set contact parameters
            contact[i].surface.mode = dContactBounce | dContactSoftCFM | dContactSoftERP | dContactApprox1;
            
            // Friction
            contact[i].surface.mu = groundFriction;
            contact[i].surface.mu2 = groundFriction;
            
            // Bounce
            contact[i].surface.bounce = groundBounce;
            contact[i].surface.bounce_vel = 0.1f;
            
            // Softness
            contact[i].surface.soft_cfm = contactSoftness;
            contact[i].surface.soft_erp = 0.2f;
            
            // Create contact joint
            dJointID c = dJointCreateContact(world, contactGroup, &contact[i]);
            dJointAttach(c, b1, b2);
            
            // Store contact information
            ContactPoint cp;
            cp.position = vsg::vec3(contact[i].geom.pos[0], contact[i].geom.pos[1], contact[i].geom.pos[2]);
            cp.normal = vsg::vec3(contact[i].geom.normal[0], contact[i].geom.normal[1], contact[i].geom.normal[2]);
            cp.depth = contact[i].geom.depth;
            cp.friction = contact[i].surface.mu;
            activeContacts.push_back(cp);
        }
    }
}

void PhysicsWorld::step(double deltaTime) {
    // Clear previous contacts
    activeContacts.clear();
    
    if (adaptiveStepping) {
        // Adaptive time stepping
        accumulator += deltaTime;
        
        while (accumulator >= stepSize) {
            // Collision detection
            dSpaceCollide(space, this, &nearCallback);
            
            // Physics step
            dWorldQuickStep(world, stepSize);
            
            // Clear contact joints
            dJointGroupEmpty(contactGroup);
            
            accumulator -= stepSize;
            simulationTime += stepSize;
            stepCount++;
        }
    } else {
        // Fixed time stepping
        dSpaceCollide(space, this, &nearCallback);
        dWorldQuickStep(world, deltaTime);
        dJointGroupEmpty(contactGroup);
        simulationTime += deltaTime;
        stepCount++;
    }
    
    // Update visual transforms
    updateTransforms();
}

void PhysicsWorld::updateTransforms() {
    for (auto& obj : objects) {
        if (obj.body && obj.transform) {
            const dReal* pos = dBodyGetPosition(obj.body);
            const dReal* rot = dBodyGetRotation(obj.body);
            
            vsg::dmat4 matrix;
            matrix[0][0] = rot[0]; matrix[0][1] = rot[1]; matrix[0][2] = rot[2]; matrix[0][3] = 0;
            matrix[1][0] = rot[4]; matrix[1][1] = rot[5]; matrix[1][2] = rot[6]; matrix[1][3] = 0;
            matrix[2][0] = rot[8]; matrix[2][1] = rot[9]; matrix[2][2] = rot[10]; matrix[2][3] = 0;
            matrix[3][0] = pos[0]; matrix[3][1] = pos[1]; matrix[3][2] = pos[2]; matrix[3][3] = 1;
            
            obj.transform->matrix = matrix;
        }
    }
}

void PhysicsWorld::setGravity(const vsg::vec3& gravity) {
    dWorldSetGravity(world, gravity.x, gravity.y, gravity.z);
}

dBodyID PhysicsWorld::createBox(const vsg::vec3& position, const vsg::vec3& size, float mass) {
    // Create body
    dBodyID body = dBodyCreate(world);
    
    // Set mass
    dMass m;
    dMassSetBoxTotal(&m, mass, size.x, size.y, size.z);
    dBodySetMass(body, &m);
    
    // Create geometry
    dGeomID geom = dCreateBox(space, size.x, size.y, size.z);
    dGeomSetBody(geom, body);
    
    // Set position
    dBodySetPosition(body, position.x, position.y, position.z);
    
    // Add visual representation
    PhysicsObject obj;
    obj.body = body;
    obj.geom = geom;
    obj.isStatic = false;
    
#ifndef USE_OPENGL_FALLBACK
    // TODO: Update for current VSG API
    // Visual representation will be added once we fix VSG geometry API
    obj.transform = vsg::MatrixTransform::create();
    // Commented out until VSG geometry API is updated
    /*
    auto transform = vsg::MatrixTransform::create();
    transform->matrix = vsg::translate(position.x, position.y, position.z);
    
    auto builder = vsg::Builder::create();
    auto box = vsg::Box::create();
    box->min = -size * 0.5f;
    box->max = size * 0.5f;
    
    vsg::GeometryInfo geomInfo;
    geomInfo.box = box;
    geomInfo.color = vsg::vec4(0.6f, 0.6f, 0.6f, 1.0f);
    
    auto node = builder->createBox(geomInfo);
    transform->addChild(node);
    
    obj.transform = transform;
    */
#endif
    
    objects.push_back(obj);
    return body;
}

dBodyID PhysicsWorld::createSphere(const vsg::vec3& position, float radius, float mass) {
    // Create body
    dBodyID body = dBodyCreate(world);
    
    // Set mass
    dMass m;
    dMassSetSphereTotal(&m, mass, radius);
    dBodySetMass(body, &m);
    
    // Create geometry
    dGeomID geom = dCreateSphere(space, radius);
    dGeomSetBody(geom, body);
    
    // Set position
    dBodySetPosition(body, position.x, position.y, position.z);
    
    // Add visual representation
    PhysicsObject obj;
    obj.body = body;
    obj.geom = geom;
    obj.isStatic = false;
    
#ifndef USE_OPENGL_FALLBACK
    // TODO: Update for current VSG API
    // Visual representation will be added once we fix VSG geometry API
    obj.transform = vsg::MatrixTransform::create();
    // Commented out until VSG geometry API is updated
    /*
    auto transform = vsg::MatrixTransform::create();
    transform->matrix = vsg::translate(position.x, position.y, position.z);
    
    auto builder = vsg::Builder::create();
    auto sphere = vsg::Sphere::create();
    sphere->radius = radius;
    
    vsg::GeometryInfo geomInfo;
    geomInfo.sphere = sphere;
    geomInfo.color = vsg::vec4(0.2f, 0.6f, 0.8f, 1.0f);
    
    auto node = builder->createSphere(geomInfo);
    transform->addChild(node);
    
    obj.transform = transform;
    */
#endif
    
    objects.push_back(obj);
    return body;
}

dBodyID PhysicsWorld::createCylinder(const vsg::vec3& position, float radius, float length, float mass) {
    // Create body
    dBodyID body = dBodyCreate(world);
    
    // Set mass
    dMass m;
    dMassSetCylinderTotal(&m, mass, 3, radius, length);
    dBodySetMass(body, &m);
    
    // Create geometry
    dGeomID geom = dCreateCylinder(space, radius, length);
    dGeomSetBody(geom, body);
    
    // Set position
    dBodySetPosition(body, position.x, position.y, position.z);
    
    // Add visual representation
    PhysicsObject obj;
    obj.body = body;
    obj.geom = geom;
    obj.isStatic = false;
    
#ifndef USE_OPENGL_FALLBACK
    // TODO: Update for current VSG API
    // Visual representation will be added once we fix VSG geometry API
    obj.transform = vsg::MatrixTransform::create();
    // Commented out until VSG geometry API is updated
    /*
    auto transform = vsg::MatrixTransform::create();
    transform->matrix = vsg::translate(position.x, position.y, position.z);
    
    auto builder = vsg::Builder::create();
    auto cylinder = vsg::Cylinder::create();
    cylinder->radius = radius;
    cylinder->height = length;
    
    vsg::GeometryInfo geomInfo;
    geomInfo.cylinder = cylinder;
    geomInfo.color = vsg::vec4(0.8f, 0.2f, 0.2f, 1.0f);
    
    auto node = builder->createCylinder(geomInfo);
    transform->addChild(node);
    
    obj.transform = transform;
    */
#endif
    
    objects.push_back(obj);
    return body;
}

dGeomID PhysicsWorld::createStaticBox(const vsg::vec3& position, const vsg::vec3& size) {
    // Create geometry without body (static)
    dGeomID geom = dCreateBox(space, size.x, size.y, size.z);
    dGeomSetPosition(geom, position.x, position.y, position.z);
    
    // Add visual representation
    PhysicsObject obj;
    obj.body = nullptr;
    obj.geom = geom;
    obj.trimeshData = nullptr;
    obj.isStatic = true;
    
#ifndef USE_OPENGL_FALLBACK
    // TODO: Update for current VSG API
    // Visual representation will be added once we fix VSG geometry API
    obj.transform = vsg::MatrixTransform::create();
    // Commented out until VSG geometry API is updated
    /*
    auto transform = vsg::MatrixTransform::create();
    transform->matrix = vsg::translate(position.x, position.y, position.z);
    
    auto builder = vsg::Builder::create();
    auto box = vsg::Box::create();
    box->min = -size * 0.5f;
    box->max = size * 0.5f;
    
    vsg::GeometryInfo geomInfo;
    geomInfo.box = box;
    geomInfo.color = vsg::vec4(0.5f, 0.5f, 0.5f, 1.0f);
    
    auto node = builder->createBox(geomInfo);
    transform->addChild(node);
    
    obj.transform = transform;
    */
#endif
    
    objects.push_back(obj);
    return geom;
}

dGeomID PhysicsWorld::createStaticTrimesh(const std::vector<float>& vertices, const std::vector<int>& indices) {
    // Create trimesh data
    dTriMeshDataID meshData = dGeomTriMeshDataCreate();
    
    // Build trimesh
    dGeomTriMeshDataBuildSingle(meshData,
        vertices.data(), 3 * sizeof(float), vertices.size() / 3,
        indices.data(), indices.size(), 3 * sizeof(int));
    
    // Create geometry
    dGeomID geom = dCreateTriMesh(space, meshData, 0, 0, 0);
    
    // Add visual representation
    PhysicsObject obj;
    obj.body = nullptr;
    obj.geom = geom;
    obj.trimeshData = meshData; // Store for proper cleanup
    obj.isStatic = true;
    
#ifndef USE_OPENGL_FALLBACK
    // TODO: Update for current VSG API
    // Visual representation will be added once we fix VSG geometry API
    obj.transform = vsg::MatrixTransform::create();
    // Commented out until VSG geometry API is updated
    /*
    auto transform = vsg::MatrixTransform::create();
    transform->matrix = vsg::translate(position.x, position.y, position.z);
    
    auto builder = vsg::Builder::create();
    auto trimesh = vsg::Trimesh::create();
    
    vsg::GeometryInfo geomInfo;
    geomInfo.trimesh = trimesh;
    geomInfo.color = vsg::vec4(0.5f, 0.5f, 0.5f, 1.0f);
    
    auto node = builder->createTrimesh(geomInfo);
    transform->addChild(node);
    
    obj.transform = transform;
    */
#endif
    
    objects.push_back(obj);
    return geom;
}

std::vector<PhysicsWorld::ContactPoint> PhysicsWorld::getContactPoints(dGeomID geom) {
    std::vector<ContactPoint> contacts;
    
    for (const auto& contact : activeContacts) {
        // This is a simplified check - in a real implementation,
        // we'd need to check if the contact involves the specified geometry
        contacts.push_back(contact);
    }
    
    return contacts;
}

bool PhysicsWorld::checkRaycast(const vsg_vec3& start, const vsg_vec3& direction, float maxDistance, vsg_vec3& hitPoint) {
    // Create temporary ray
    dGeomID ray = dCreateRay(space, maxDistance);
    dGeomRaySet(ray, start.x, start.y, start.z, direction.x, direction.y, direction.z);
    
    // Simple collision detection
    bool hit = false;
    for (const auto& obj : objects) {
        dContactGeom contact;
        int numContacts = dCollide(ray, obj.geom, 1, &contact, sizeof(dContactGeom));
        if (numContacts > 0) {
            hitPoint = vsg_vec3(contact.pos[0], contact.pos[1], contact.pos[2]);
            hit = true;
            break;
        }
    }
    
    dGeomDestroy(ray);
    return hit;
}

#ifndef USE_OPENGL_FALLBACK
vsg::ref_ptr<vsg::Group> PhysicsWorld::getDebugGeometry() {
    if (!debugVisualization) return nullptr;
    
    auto group = vsg::Group::create();
    
    // TODO: Update for current VSG API
    // Debug geometry will be added once we fix VSG geometry API
    /*
    for (const auto& obj : objects) {
        if (obj.transform) {
            auto sphere = vsg::Sphere::create();
            sphere->radius = 0.1f;
            
            vsg::GeometryInfo geomInfo;
            geomInfo.sphere = sphere;
            geomInfo.color = vsg::vec4(1.0f, 0.0f, 0.0f, 1.0f);
            
            auto builder = vsg::Builder::create();
            auto node = builder->createSphere(geomInfo);
            
            auto transform = vsg::MatrixTransform::create();
            const dReal* pos = dGeomGetPosition(obj.geom);
            transform->matrix = vsg::translate(pos[0], pos[1], pos[2]);
            transform->addChild(node);
            
            group->addChild(transform);
        }
    }
    */
    
    return group;
}
#else
ref_ptr<Group> PhysicsWorld::getDebugGeometry() {
    // OpenGL fallback - debug geometry handled differently
    return ref_ptr<Group>(new Group());
}
#endif