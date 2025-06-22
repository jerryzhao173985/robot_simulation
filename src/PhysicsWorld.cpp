#include "PhysicsWorld.h"
#include "DebugOutput.h"
#include <iostream>
#include <sstream>
#include <algorithm>

PhysicsWorld::PhysicsWorld() {
    // Create ODE world
    world = dWorldCreate();
    space = dHashSpaceCreate(0);
    contactGroup = dJointGroupCreate(0);
    
    // Set default parameters (use Z-up in sync with VSG)
    dWorldSetGravity(world, 0, 0, -9.81f);
    dWorldSetERP(world, 0.8f);  // Higher error reduction for stiffer contacts
    dWorldSetCFM(world, 1e-6);  // Stiffer constraint force mixing
    dWorldSetMaxAngularSpeed(world, 10.0f);  // Limit angular speed
    dWorldSetContactMaxCorrectingVel(world, 10.0f);  // Higher correcting velocity to prevent sinking
    dWorldSetContactSurfaceLayer(world, 0.0001f);  // Smaller surface layer to prevent sinking
    dWorldSetAutoDisableFlag(world, 0);
    dWorldSetQuickStepNumIterations(world, 20);  // Standard iteration count
    dWorldSetQuickStepW(world, 1.3);  // Standard over-relaxation
    
    // Create a large box as ground instead of plane for better collision
    // Box provides more reliable collision detection than infinite plane
    groundPlane = dCreateBox(space, 100.0f, 100.0f, 1.0f);  // 100x100x1 meter box
    dGeomSetPosition(groundPlane, 0, 0, -0.5f);  // Position so top is at Z=0
    
    // Make it completely immovable
    dGeomSetCategoryBits(groundPlane, ~0);
    dGeomSetCollideBits(groundPlane, ~0);
    
    std::cout << "[PhysicsWorld] Created ground box at Z=0 (100x100x1m)" << std::endl;
    
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
    // Debug counter
    static int collisionCount = 0;
    collisionCount++;
    
    // Check if both geometries have bodies
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    
    // Exit if both bodies exist and are connected by a joint (but not for ground!)
    if (b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) {
        return;
    }
    
    // Check if this is a ground collision (terrain or ground plane has no body)
    bool isGroundCollision = (!b1 || !b2); // One geometry has no body = static/ground
    
    // Check if it's specifically the ground plane
    bool isGroundPlane = (o1 == groundPlane || o2 == groundPlane);
    
    // Debug: Log collision checks with static geometry
    if (collisionCount < 10 && isGroundCollision) {
        dGeomID movingGeom = b1 ? o1 : o2;
        int cls = dGeomGetClass(movingGeom);
        std::cout << "[STATIC COLLISION] #" << collisionCount 
                 << " - " << (cls == dBoxClass ? "Box" : 
                            cls == dCapsuleClass ? "Capsule" : 
                            cls == dTriMeshClass ? "TriMesh" : "Other") 
                 << " hitting static geometry" << std::endl;
    }
    
    // Debug: Log ground plane collisions with more detail
    static int groundPlaneCollisions = 0;
    if (isGroundPlane && groundPlaneCollisions < 50) {
        groundPlaneCollisions++;
        dGeomID obj = (o1 == groundPlane) ? o2 : o1;
        const dReal* pos = dGeomGetPosition(obj);
        dBodyID body = dGeomGetBody(obj);
        int geomClass = dGeomGetClass(obj);
        const char* className = (geomClass == dSphereClass) ? "Sphere" : 
                               (geomClass == dCapsuleClass) ? "Capsule" : 
                               (geomClass == dBoxClass) ? "Box" : "Other";
        
        // Check if it's a robot part
        bool isRobotPart = (body != nullptr);
        
        std::cout << "[GROUND] #" << groundPlaneCollisions 
                  << " " << className 
                  << " at Z=" << pos[2] 
                  << (isRobotPart ? " (ROBOT PART)" : " (static)")
                  << std::endl;
    }
    
    // Allow more contact points for ground plane to handle 6 feet
    // Need at least 6 for hexapod, plus some extra
    const int maxContacts = isGroundPlane ? 12 : 8;
    dContact contact[16];  // Stack allocation size
    
    // Get contact points
    int numContacts = dCollide(o1, o2, maxContacts, &contact[0].geom, sizeof(dContact));
    
    if (numContacts > 0) {
        // Only log excessive contacts
        static int excessiveContactCount = 0;
        if (isGroundCollision && numContacts > 8 && excessiveContactCount < 10) {
            excessiveContactCount++;
            std::cout << "[WARNING] Excessive contacts: " << numContacts << " joints" << std::endl;
        }
        
        for (int i = 0; i < numContacts; ++i) {
            // Set contact parameters - extra stiff for ground contacts
            if (isGroundPlane) {
                // Ground box - extremely stiff parameters to prevent sinking
                contact[i].surface.mode = dContactBounce | dContactSoftCFM | dContactSoftERP | dContactApprox1;
                contact[i].surface.mu = 2.0;            // High friction for better grip
                contact[i].surface.mu2 = 2.0;
                contact[i].surface.bounce = 0.0f;       // No bounce
                contact[i].surface.bounce_vel = 0.1f;   // Minimum velocity for bounce
                contact[i].surface.soft_cfm = 0.0;      // Infinitely stiff contact
                contact[i].surface.soft_erp = 0.95;     // Very high error reduction for instant response
            } else if (isGroundCollision) {
                // Terrain or other static geometry
                contact[i].surface.mode = dContactBounce | dContactSoftCFM;
                contact[i].surface.mu = 1.0f;           // Reasonable friction
                contact[i].surface.mu2 = 1.0f;
                contact[i].surface.bounce = 0.0f;       // No bounce
                contact[i].surface.bounce_vel = 0.1f;   // Minimum velocity for bounce
                contact[i].surface.soft_cfm = 0.00001;  // Slightly soft for stability
            } else {
                // Normal parameters for object-object contacts
                contact[i].surface.mode = dContactBounce | dContactSoftCFM;
                contact[i].surface.mu = 0.5f;           // Moderate friction
                contact[i].surface.mu2 = 0.5f;
                contact[i].surface.bounce = 0.1f;       // Small bounce
                contact[i].surface.bounce_vel = 0.1f;   // Threshold velocity
                contact[i].surface.soft_cfm = 0.0001;   // Slightly soft for stability
            }
            
            // Create contact joint
            dJointID c = dJointCreateContact(world, contactGroup, &contact[i]);
            
            // For ground plane, ensure proper attachment
            if (isGroundPlane) {
                dBodyID movingBody = b1 ? b1 : b2;
                dJointAttach(c, movingBody, 0);  // Attach to world (0 = static world)
                
                // Debug first few ground contacts
                static int groundContactCount = 0;
                if (groundContactCount++ < 10) {
                    std::cout << "[GROUND CONTACT] Created joint for body at Z=" 
                              << contact[i].geom.pos[2] << " with depth=" 
                              << contact[i].geom.depth << std::endl;
                }
            } else if (!b1) {
                dJointAttach(c, b2, 0);  // First is static, attach second to world
            } else if (!b2) {
                dJointAttach(c, b1, 0);  // Second is static, attach first to world
            } else {
                dJointAttach(c, b1, b2);  // Both dynamic
            }
            
            // For ground plane contacts, ensure nothing goes below ground
            if (isGroundPlane) {
                // Force contact points to be at or above ground
                if (contact[i].geom.pos[2] < 0.0) {
                    contact[i].geom.pos[2] = 0.0;
                    contact[i].geom.depth = std::max(contact[i].geom.depth, 0.01);
                }
            }
            
            if (isGroundCollision && i == 0 && contact[i].geom.depth > 0.01f) { // Only log significant penetrations
                std::stringstream ss;
                ss << "[WARNING] Deep contact at Z=" << contact[i].geom.pos[2] 
                   << " depth=" << contact[i].geom.depth;
                DEBUG_PHYSICS(ss.str());
            }
            
            // Store contact information
            ContactPoint cp;
            cp.position = vsg::vec3(contact[i].geom.pos[0], contact[i].geom.pos[1], contact[i].geom.pos[2]);
            cp.normal = vsg::vec3(contact[i].geom.normal[0], contact[i].geom.normal[1], contact[i].geom.normal[2]);
            cp.depth = contact[i].geom.depth;
            cp.friction = contact[i].surface.mu;
            cp.geom1 = o1;
            cp.geom2 = o2;
            cp.body1 = b1;
            cp.body2 = b2;
            cp.normal1 = cp.normal;
            cp.normal2 = cp.normal * -1.0f;
            activeContacts.push_back(cp);
        }
    }
}

void PhysicsWorld::step(double deltaTime) {
    // Clear previous contacts
    activeContacts.clear();
    
    // Debug first few steps
    static int stepDebugCount = 0;
    if (stepDebugCount++ < 5) {
        std::cout << "[PHYSICS] Step " << stepDebugCount << " - deltaTime=" << deltaTime << std::endl;
        std::cout.flush();
    }
    
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

void PhysicsWorld::emergencyGroundClamping() {
    // Disabled - let physics handle collisions naturally
    // This function was causing artificial behavior
}

void PhysicsWorld::updateTransforms() {
    for (auto& obj : objects) {
        if (obj.body && obj.transform) {
            const dReal* pos = dBodyGetPosition(obj.body);
            const dReal* rot = dBodyGetRotation(obj.body);
            
            // Copy transform data to matrix
            // VSG uses column-major matrices, access with (row, col)
            obj.transform->matrix(0, 0) = rot[0]; obj.transform->matrix(0, 1) = rot[1]; obj.transform->matrix(0, 2) = rot[2]; obj.transform->matrix(0, 3) = 0;
            obj.transform->matrix(1, 0) = rot[4]; obj.transform->matrix(1, 1) = rot[5]; obj.transform->matrix(1, 2) = rot[6]; obj.transform->matrix(1, 3) = 0;
            obj.transform->matrix(2, 0) = rot[8]; obj.transform->matrix(2, 1) = rot[9]; obj.transform->matrix(2, 2) = rot[10]; obj.transform->matrix(2, 3) = 0;
            obj.transform->matrix(3, 0) = pos[0]; obj.transform->matrix(3, 1) = pos[1]; obj.transform->matrix(3, 2) = pos[2]; obj.transform->matrix(3, 3) = 1;
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
    for (const auto& cp : activeContacts) {
        if (cp.geom1 == geom) {
            ContactPoint oriented = cp;
            oriented.normal = cp.normal1;
            contacts.push_back(oriented);
        } else if (cp.geom2 == geom) {
            ContactPoint oriented = cp;
            oriented.normal = cp.normal2;
            contacts.push_back(oriented);
        }
    }
    return contacts;
}

// Return all active contacts (no filtering).
std::vector<PhysicsWorld::ContactPoint> PhysicsWorld::getActiveContacts() const {
    return activeContacts;
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

PhysicsWorld::RaycastResult PhysicsWorld::raycast(const vsg_vec3& origin, const vsg_vec3& direction, float maxDistance) {
    RaycastResult result;
    result.hit = false;
    result.distance = maxDistance;
    
    // Normalize direction
    vsg_vec3 dir = vsg::normalize(direction);
    
    // Create temporary ray
    dGeomID ray = dCreateRay(space, maxDistance);
    dGeomRaySet(ray, origin.x, origin.y, origin.z, dir.x, dir.y, dir.z);
    
    // Check collision with all objects
    float closestDistance = maxDistance;
    dContactGeom closestContact;
    dGeomID closestGeom = nullptr;
    
    // Check ground plane first
    if (groundPlane) {
        dContactGeom contact;
        int numContacts = dCollide(ray, groundPlane, 1, &contact, sizeof(dContactGeom));
        if (numContacts > 0 && contact.depth < closestDistance) {
            closestDistance = contact.depth;
            closestContact = contact;
            closestGeom = groundPlane;
        }
    }
    
    // Check all other objects
    for (const auto& obj : objects) {
        dContactGeom contact;
        int numContacts = dCollide(ray, obj.geom, 1, &contact, sizeof(dContactGeom));
        if (numContacts > 0 && contact.depth < closestDistance) {
            closestDistance = contact.depth;
            closestContact = contact;
            closestGeom = obj.geom;
        }
    }
    
    // Fill result if hit
    if (closestGeom) {
        result.hit = true;
        result.distance = closestDistance;
        result.point = vsg_vec3(closestContact.pos[0], closestContact.pos[1], closestContact.pos[2]);
        result.normal = vsg_vec3(closestContact.normal[0], closestContact.normal[1], closestContact.normal[2]);
        result.geom = closestGeom;
    }
    
    dGeomDestroy(ray);
    return result;
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