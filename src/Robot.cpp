#include "Robot.h"
#include <iostream>
#include <cmath>

Robot::Robot(dWorldID world, dSpaceID space, 
#ifdef USE_OPENGL_FALLBACK
             ref_ptr<Group> sceneGraph
#else
             vsg::ref_ptr<vsg::Group> sceneGraph
#endif
)
    : sceneGraph(sceneGraph) {
    
    // Initialize physics
    bodyId = dBodyCreate(world);
    
#ifndef USE_OPENGL_FALLBACK
    // Ensure no visual components are created
    robotTransform = nullptr;
#endif
    
    createBody();
    createLegs();
    
    // Create collision geometry
    dGeomID geom = dCreateBox(space, config.bodySize.x, config.bodySize.y, config.bodySize.z);
    dGeomSetBody(geom, bodyId);
    
    // Initialize visual representation
    createVisualModel();
    addToScene(sceneGraph);
    
    std::cout << "Robot initialized with physics and visual components" << std::endl;
    // Verify leg attachment positions (X, Y horizontal plane; Z height)
    for (int i = 0; i < NUM_LEGS; ++i) {
        vsg_vec3 pos = getLegPosition(i);
        std::cout << "Leg " << i << " attachment position (X, Y, Z): ("
                  << pos.x << ", " << pos.y << ", " << pos.z << ")" << std::endl;
    }
}

Robot::~Robot() {
    // Clean up physics bodies
    if (bodyId) dBodyDestroy(bodyId);
    
    // Clean up leg bodies
    for (auto body : legBodies) {
        if (body) dBodyDestroy(body);
    }
    
    // Clean up joints
    for (auto joint : legJoints) {
        if (joint) dJointDestroy(joint);
    }
}

void Robot::createBody() {
    // Create main body mass
    dMass mass;
    dMassSetBoxTotal(&mass, 5.0f, config.bodySize.x, config.bodySize.y, config.bodySize.z);
    dBodySetMass(bodyId, &mass);
    
    // Set initial position - Z-up coordinate system to match VSG visual model
    dBodySetPosition(bodyId, 0.0f, 0.0f, 0.8f);  // Z-up: body at 0.8m height
    
    // Initialize leg containers
    legBodies.resize(6);
    legJoints.resize(6);
    
    std::cout << "Robot physics body created with mass 5.0kg" << std::endl;
}

void Robot::createLegs() {
    // Simplified leg creation - just initialize the containers
    legBodies.resize(6);
    legJoints.resize(6);
    
    // Create simple leg bodies (this would be expanded with proper physics)
    for (int i = 0; i < 6; ++i) {
        // For now, just set to nullptr - will be implemented later
        legBodies[i] = nullptr;
        legJoints[i] = nullptr;
    }
}

void Robot::update(double deltaTime) {
    // Basic update - simplified version
    updateLegPositions();
    applyForces();
    maintainBalance();
    
    // Update visual transform to match physics body
#ifndef USE_OPENGL_FALLBACK
    // Visual representation completely handled by Visualizer class
    // No visual updates needed here - physics only
#endif
}

void Robot::updateLegPositions() {
    // Simplified leg position updates
    // This would contain the actual leg movement logic
}

vsg_vec3 Robot::getLegPosition(int legIndex) const {
    // Position legs in proper hexapod layout - 3 pairs along body sides
    const double bodyLength = config.bodySize.x;
    const double bodyWidth = config.bodySize.y;
    
    // X positions for front, middle, rear leg pairs
    std::vector<double> legXPositions = { 
        -bodyLength * 0.3,   // front legs
         0.0,                // middle legs  
         bodyLength * 0.3    // rear legs
    };
    
    int pairIndex = legIndex / 2;  // 0, 1, 2 for front, middle, rear
    int side = (legIndex % 2 == 0) ? -1 : 1;  // even = left, odd = right
    
    double x = legXPositions[pairIndex];
    double y = side * (bodyWidth * 0.5);  // at body edge
    
    return vsg_vec3(x, y, 0.0f);  // Z offset handled by attachment point
}

void Robot::applyForces() {
    // Apply movement forces
    if (vsg::length(targetVelocity) > 0.01f || std::abs(targetAngularVelocity) > 0.01f) {
        const dReal* currentVel = dBodyGetLinearVel(bodyId);
        vsg_vec3 velError = targetVelocity - vsg_vec3(currentVel[0], currentVel[1], currentVel[2]);
        
        // Apply force to reach target velocity
        dBodyAddForce(bodyId, 
            velError.x * 50.0f,
            0,
            velError.z * 50.0f);
        
        // Apply torque for rotation
        dBodyAddTorque(bodyId, 0, targetAngularVelocity * 20.0f, 0);
    }
}

void Robot::maintainBalance() {
    // Get body orientation
    const dReal* q = dBodyGetQuaternion(bodyId);
    vsg_quat orientation(q[1], q[2], q[3], q[0]);
    
    // Calculate Euler angles for stabilization
    vsg_vec3 euler = eulerAnglesFromQuat(orientation);
    float roll = euler.x;
    float pitch = euler.z;
    
    // Apply corrective torques
    dBodyAddTorque(bodyId, -pitch * 50.0f, 0, -roll * 50.0f);
}

void Robot::reset() {
    // Reset position and orientation
    dBodySetPosition(bodyId, 0, 0, 0.8f);  // Z-up coordinate system
    dBodySetLinearVel(bodyId, 0, 0, 0);
    dBodySetAngularVel(bodyId, 0, 0, 0);
    
    dQuaternion q;
    dQSetIdentity(q);
    dBodySetQuaternion(bodyId, q);
    
    // Reset state
    targetVelocity = vsg_vec3(0, 0, 0);
    targetAngularVelocity = 0;
}

vsg_vec3 Robot::getPosition() const {
    const dReal* pos = dBodyGetPosition(bodyId);
    return vsg_vec3(pos[0], pos[1], pos[2]);
}

vsg_vec3 Robot::getVelocity() const {
    const dReal* vel = dBodyGetLinearVel(bodyId);
    return vsg_vec3(vel[0], vel[1], vel[2]);
}

vsg_quat Robot::getOrientation() const {
    const dReal* q = dBodyGetQuaternion(bodyId);
    return vsg_quat(q[1], q[2], q[3], q[0]);
}

bool Robot::isStable() const {
    // Simple stability check based on orientation
    vsg_quat orientation = getOrientation();
    vsg_vec3 euler = eulerAnglesFromQuat(orientation);
    
    return std::abs(euler.x) < 0.5f && std::abs(euler.z) < 0.5f;
}

void Robot::setBodyColor(const vsg_vec4& color) {
    bodyColor = color;
}

void Robot::applyControl(const std::vector<float>& motorCommands) {
    // Simplified control application
    // This would apply motor commands to the leg joints
    // For now, just store the commands for future use
    if (motorCommands.size() >= 6) {
        // Apply some basic movement based on motor commands
        // This is a placeholder implementation
        float avgCommand = 0.0f;
        for (size_t i = 0; i < std::min(motorCommands.size(), size_t(6)); ++i) {
            avgCommand += motorCommands[i];
        }
        avgCommand /= 6.0f;
        
        // Use average command to influence target velocity
        targetVelocity.x = avgCommand * 2.0f;  // Scale as needed
    }
}

std::vector<float> Robot::getSensorReadings() const {
    // Return simplified sensor readings
    std::vector<float> readings;
    
    // Add position as sensor data
    vsg_vec3 pos = getPosition();
    readings.push_back(pos.x);
    readings.push_back(pos.y);
    readings.push_back(pos.z);
    
    // Add velocity as sensor data
    vsg_vec3 vel = getVelocity();
    readings.push_back(vel.x);
    readings.push_back(vel.y);
    readings.push_back(vel.z);
    
    // Add orientation as sensor data
    vsg_quat orient = getOrientation();
    readings.push_back(orient.x);
    readings.push_back(orient.y);
    readings.push_back(orient.z);
    readings.push_back(orient.w);
    
    // Add stability as sensor reading
    readings.push_back(isStable() ? 1.0f : 0.0f);
    
    return readings;
}

vsg_vec3 Robot::eulerAnglesFromQuat(const vsg_quat& q) const {
    vsg_vec3 euler;
    
    // Roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    euler.x = std::atan2(sinr_cosp, cosr_cosp);
    
    // Pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        euler.y = std::copysign(M_PI / 2, sinp);
    else
        euler.y = std::asin(sinp);
    
    // Yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    euler.z = std::atan2(siny_cosp, cosy_cosp);
    
    return euler;
}

void Robot::createVisualModel() {
#ifndef USE_OPENGL_FALLBACK
    robotTransform = vsg::MatrixTransform::create();
    
    try {
        // Create builder
        auto builder = vsg::Builder::create();
        
        // Create robot body
        vsg::GeometryInfo bodyGeomInfo;
        vsg::StateInfo bodyStateInfo;
        
        bodyGeomInfo.dx = vsg::vec3(config.bodySize.x, 0.0f, 0.0f);
        bodyGeomInfo.dy = vsg::vec3(0.0f, config.bodySize.y, 0.0f);
        bodyGeomInfo.dz = vsg::vec3(0.0f, 0.0f, config.bodySize.z);
        bodyGeomInfo.color = vsg::vec4(bodyColor.x, bodyColor.y, bodyColor.z, bodyColor.w);
        
        auto bodyNode = builder->createBox(bodyGeomInfo, bodyStateInfo);
        if (bodyNode) {
            robotTransform->addChild(bodyNode);
            std::cout << "Created robot body box" << std::endl;
        }
        
        // Create legs as spheres
        legTransforms.resize(6);
        for (int i = 0; i < 6; ++i) {
            auto legTransform = vsg::MatrixTransform::create();
            auto legPosition = getLegPosition(i);
            legTransform->matrix = vsg::translate(legPosition.x, legPosition.y, legPosition.z);
            
            vsg::GeometryInfo legGeomInfo;
            vsg::StateInfo legStateInfo;
            
            legGeomInfo.dx = vsg::vec3(config.footRadius, 0.0f, 0.0f);
            legGeomInfo.dy = vsg::vec3(0.0f, config.footRadius, 0.0f);
            legGeomInfo.dz = vsg::vec3(0.0f, 0.0f, config.footRadius);
            legGeomInfo.color = vsg::vec4(1.0f, 1.0f, 0.0f, 1.0f);  // Yellow feet
            
            auto legNode = builder->createSphere(legGeomInfo, legStateInfo);
            if (legNode) {
                legTransform->addChild(legNode);
                robotTransform->addChild(legTransform);
                std::cout << "Created leg " << i << std::endl;
            }
            
            legTransforms[i] = legTransform;
        }
        
        std::cout << "Robot visual model created successfully" << std::endl;
        
    } catch (const std::exception& e) {
        std::cout << "VSG creation failed: " << e.what() << std::endl;
        
        // Ultra-simple fallback - just create empty transforms
        legTransforms.resize(6);
        for (int i = 0; i < 6; ++i) {
            legTransforms[i] = vsg::MatrixTransform::create();
            robotTransform->addChild(legTransforms[i]);
        }
        
        std::cout << "Created fallback empty robot" << std::endl;
    }
#endif
}

#ifndef USE_OPENGL_FALLBACK
vsg::ref_ptr<vsg::MatrixTransform> Robot::getRobotNode() {
    if (!robotTransform) {
        createVisualModel();
    }
    return robotTransform;
}

void Robot::addToScene(vsg::ref_ptr<vsg::Group> scene) {
    auto node = getRobotNode();
    if (node) {
        scene->addChild(node);
    }
}
#else
ref_ptr<MatrixTransform> Robot::getRobotNode() {
    // OpenGL fallback - return placeholder
    return ref_ptr<MatrixTransform>(new MatrixTransform());
}

void Robot::addToScene(ref_ptr<Group> scene) {
    // OpenGL fallback - robot handled differently
}
#endif