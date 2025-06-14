#include "Robot.h"
#include <array>
#include <iostream>
#include <cmath>

Robot::Robot(dWorldID world, dSpaceID space, 
#ifdef USE_OPENGL_FALLBACK
             ref_ptr<Group> sceneGraph
#else
             vsg::ref_ptr<vsg::Group> sceneGraph
#endif
)
    : sceneGraph(sceneGraph)
    , world(world)
    , space(space) {
    
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
    bodyGeom = geom;
    
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
    // Add damping so oscillations decay
    dBodySetDamping(bodyId, 0.1f, 0.1f);
    
    // Set initial position so feet touch the ground at start
    // body half-height + leg lengths + foot radius
    float initZ = config.bodySize.z * 0.5f + config.upperLegLength + config.lowerLegLength + config.footRadius;
    dBodySetPosition(bodyId, 0.0f, 0.0f, initZ);
    std::cout << "[Robot] Initial body Z position = " << initZ << std::endl;
    
    // Initialize leg containers
    legBodies.resize(6);
    legJoints.resize(6);
    
    std::cout << "Robot physics body created with mass 5.0kg" << std::endl;
}

void Robot::createLegs() {
    legBodies.resize(NUM_LEGS);
    legJoints.resize(NUM_LEGS);
    for (int i = 0; i < NUM_LEGS; ++i) {
        legs[i].segments.clear();

        vsg_vec3 attach = getLegPosition(i);
        const dReal* bodyPos = dBodyGetPosition(bodyId);

        // ----- Upper leg (coxa) -----
        dBodyID upperBody = dBodyCreate(world);
        dMass m;
        dMassSetCapsuleTotal(&m, 0.2f, 3, config.legRadius, config.upperLegLength);
        dBodySetMass(upperBody, &m);
        dBodySetPosition(upperBody,
            bodyPos[0] + attach.x,
            bodyPos[1] + attach.y,
            bodyPos[2] + attach.z - config.upperLegLength * 0.5f);
        dGeomID upperGeom = dCreateCapsule(space, config.legRadius, config.upperLegLength);
        dGeomSetBody(upperGeom, upperBody);
        dJointID hip = dJointCreateBall(world, 0);
        dJointAttach(hip, bodyId, upperBody);
        dJointSetBallAnchor(hip,
            bodyPos[0] + attach.x,
            bodyPos[1] + attach.y,
            bodyPos[2] + attach.z);

        LegSegment segUpper{upperBody, upperGeom, hip, config.upperLegLength, config.legRadius};
        legs[i].segments.push_back(segUpper);

        // ----- Lower leg (femur) -----
        dBodyID lowerBody = dBodyCreate(world);
        dMassSetCapsuleTotal(&m, 0.15f, 3, config.legRadius, config.lowerLegLength);
        dBodySetMass(lowerBody, &m);
        dBodySetPosition(lowerBody,
            bodyPos[0] + attach.x,
            bodyPos[1] + attach.y,
            bodyPos[2] + attach.z - config.upperLegLength - config.lowerLegLength * 0.5f);
        dGeomID lowerGeom = dCreateCapsule(space, config.legRadius, config.lowerLegLength);
        dGeomSetBody(lowerGeom, lowerBody);
        dJointID knee = dJointCreateHinge(world, 0);
        dJointAttach(knee, upperBody, lowerBody);
        dJointSetHingeAnchor(knee,
            bodyPos[0] + attach.x,
            bodyPos[1] + attach.y,
            bodyPos[2] + attach.z - config.upperLegLength);
        dJointSetHingeAxis(knee, 1, 0, 0);

        LegSegment segLower{lowerBody, lowerGeom, knee, config.lowerLegLength, config.legRadius};
        legs[i].segments.push_back(segLower);

        // ----- Foot (tibia/foot sphere) -----
        dBodyID footBody = dBodyCreate(world);
        dMassSetSphereTotal(&m, 0.05f, config.footRadius);
        dBodySetMass(footBody, &m);
        dBodySetPosition(footBody,
            bodyPos[0] + attach.x,
            bodyPos[1] + attach.y,
            bodyPos[2] + attach.z - config.upperLegLength - config.lowerLegLength - config.footRadius);
        dGeomID footGeom = dCreateSphere(space, config.footRadius);
        dGeomSetBody(footGeom, footBody);
        dJointID ankle = dJointCreateHinge(world, 0);
        dJointAttach(ankle, lowerBody, footBody);
        dJointSetHingeAnchor(ankle,
            bodyPos[0] + attach.x,
            bodyPos[1] + attach.y,
            bodyPos[2] + attach.z - config.upperLegLength - config.lowerLegLength);
        dJointSetHingeAxis(ankle, 1, 0, 0);

        LegSegment segFoot{footBody, footGeom, ankle, config.footRadius, config.footRadius};
        legs[i].segments.push_back(segFoot);

        legBodies[i] = footBody;
        legJoints[i] = hip; // store hip joint for reference
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
    // Update all leg segment visuals under the robotTransform to match their ODE body transforms
#ifndef USE_OPENGL_FALLBACK
    // For each segment, update its transform using ODE pose
    int idx = 0;
    for (int i = 0; i < NUM_LEGS; ++i) {
        for (size_t j = 0; j < legs[i].segments.size(); ++j, ++idx) {
            auto& seg = legs[i].segments[j];
            if (!seg.transform) continue;
            const dReal* p = dBodyGetPosition(seg.body);
            const dReal* R = dBodyGetRotation(seg.body);
            vsg::dmat4 M(
                R[0], R[1], R[2], 0.0,
                R[4], R[5], R[6], 0.0,
                R[8], R[9], R[10], 0.0,
                p[0], p[1], p[2], 1.0);
            if (idx < int(legNodes.size()) && legNodes[idx]) {
                legNodes[idx]->matrix = M;
            }
        }
    }
#endif
}

vsg_vec3 Robot::getLegPosition(int legIndex) const {
    double halfWidth = config.bodySize.x * 0.5;
    double halfHeight = config.bodySize.z * 0.5;
    std::array<double, 3> legY = {
        -config.bodySize.y * 0.3,
         0.0,
         config.bodySize.y * 0.3
    };
    int pairIndex = legIndex / 2;
    int sideSign = (legIndex % 2 == 0) ? -1 : 1;
    double x = sideSign * halfWidth;
    double y = legY[pairIndex];
    double z = -halfHeight;
    return vsg_vec3(x, y, z);
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

dBodyID Robot::getBody() const {
    return bodyId;
}


std::vector<dGeomID> Robot::getFootGeoms() const {
    std::vector<dGeomID> geoms;
    geoms.reserve(NUM_LEGS);
    for (int i = 0; i < NUM_LEGS; ++i) {
        if (!legs[i].segments.empty()) {
            geoms.push_back(legs[i].segments.back().geom);
        }
    }
    return geoms;
}

dGeomID Robot::getBodyGeom() const {
    return bodyGeom;
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
        }

        int numSegmentsTotal = 0;
        for (int i = 0; i < NUM_LEGS; ++i) {
            numSegmentsTotal += int(legs[i].segments.size());
        }
        legNodes.clear();
        legNodes.resize(numSegmentsTotal);

        int idx = 0;
        for (int i = 0; i < NUM_LEGS; ++i) {
            for (size_t j = 0; j < legs[i].segments.size(); ++j, ++idx) {
                auto& seg = legs[i].segments[j];
                auto xform = vsg::MatrixTransform::create();
                seg.transform = xform;
                legNodes[idx] = xform;

                // Colors for debug: each leg a distinct color, segments: hue variation
                vsg::vec4 color = vsg::vec4(1.0f, 0.5f, 0.2f, 1.0f); // Default orange
                if (j == 0) color = vsg::vec4(1.0-(i/5.0f), i/5.0f, 1.0f, 1.0f); // rainbow feet for leg/debug

                vsg::GeometryInfo legGeomInfo;
                vsg::StateInfo legStateInfo;
                legGeomInfo.dx = vsg::vec3(seg.radius, 0.0f, 0.0f);
                legGeomInfo.dy = vsg::vec3(0.0f, seg.radius, 0.0f);
                legGeomInfo.dz = vsg::vec3(0.0f, 0.0f, seg.length);
                legGeomInfo.color = color;

                // Use cylinder if long, else sphere
                vsg::ref_ptr<vsg::Node> geomNode;
                if (seg.length > seg.radius*1.5f)
                    geomNode = builder->createCylinder(legGeomInfo, legStateInfo);
                else
                    geomNode = builder->createSphere(legGeomInfo, legStateInfo);

                if (geomNode)
                    xform->addChild(geomNode);
                robotTransform->addChild(xform);
            }
        }

    } catch (const std::exception& e) {
        // Fallback: empty transforms
        legNodes.clear();
        int numSegmentsTotal = 0;
        for (int i = 0; i < NUM_LEGS; ++i) numSegmentsTotal += int(legs[i].segments.size());
        legNodes.resize(numSegmentsTotal);
        int idx = 0;
        for (int i = 0; i < NUM_LEGS; ++i) {
            for (size_t j = 0; j < legs[i].segments.size(); ++j, ++idx) {
                auto xform = vsg::MatrixTransform::create();
                legs[i].segments[j].transform = xform;
                legNodes[idx] = xform;
                robotTransform->addChild(xform);
            }
        }
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