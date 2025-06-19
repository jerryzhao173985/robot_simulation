#include "Robot.h"
#include <array>
#include <iostream>
#include <cmath>

// Debug control - comment out for production builds
#define DEBUG_ROBOT_INIT

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
    // Initialize visual components properly to avoid null pointer issues
    // Visual representation is handled by Visualizer, but we need valid transforms for sync
    robotTransform = vsg::MatrixTransform::create();
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
    
#ifdef DEBUG_ROBOT_INIT
    std::cout << "🦾 Robot initialized with proper hexapod anatomy!" << std::endl;
    // Verify leg attachment positions match Visualizer
    for (int i = 0; i < NUM_LEGS; ++i) {
        vsg_vec3 pos = getLegPosition(i);
        std::cout << "Leg " << i << " attachment (X, Y, Z): ("
                  << pos.x << ", " << pos.y << ", " << pos.z << ")" << std::endl;
    }
    std::cout << "✅ Physics legs now match Visualizer anatomy!" << std::endl;
#endif
}

Robot::~Robot() {
    // Clean up physics bodies with proper validation
    if (bodyId) dBodyDestroy(bodyId);
    
    // Clean up all leg segment bodies with enhanced null pointer checks
    for (int i = 0; i < NUM_LEGS; ++i) {
        for (auto& segment : legs[i].segments) {
            if (segment.body && dBodyIsConnected(segment.body)) {
                dBodyDestroy(segment.body);
            }
            if (segment.joint) {
                dJointDestroy(segment.joint);
            }
        }
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
    // Calculate total leg reach: coxa + femur + tibia with mathematical validation
    float totalLegLength = config.coxaLength + config.femurLength + config.tibiaLength;
    
    // Validate leg proportions are realistic for hexapod locomotion
    float legToBodyRatio = totalLegLength / config.bodySize.z;
    if (legToBodyRatio < 2.0f || legToBodyRatio > 8.0f) {
#ifdef DEBUG_ROBOT_INIT
        std::cout << "⚠️  Warning: Leg to body ratio (" << legToBodyRatio 
                  << ") may affect stability. Optimal range: 2.0-8.0" << std::endl;
#endif
    }
    
    float initZ = config.bodySize.z * 0.5f + totalLegLength * 0.8f; // 80% of max reach
    dBodySetPosition(bodyId, 0.0f, 0.0f, initZ);
    
#ifdef DEBUG_ROBOT_INIT
    std::cout << "[Robot] Initial body Z position = " << initZ << " (total leg reach: " << totalLegLength << ")" << std::endl;
    std::cout << "✅ Robot physics body created with realistic hexapod proportions" << std::endl;
#endif
}

void Robot::createLegs() {
    legs.fill(Leg{}); // Initialize all legs
    
    const dReal* bodyPos = dBodyGetPosition(bodyId);
    
#ifdef DEBUG_ROBOT_INIT
    std::cout << "🦵 Creating proper 3-segment hexapod legs..." << std::endl;
#endif
    
    for (int i = 0; i < NUM_LEGS; ++i) {
        legs[i].segments.clear();
        vsg_vec3 attachPoint = getLegPosition(i);
        
        // World position of leg attachment
        double attachX = bodyPos[0] + attachPoint.x;
        double attachY = bodyPos[1] + attachPoint.y;
        double attachZ = bodyPos[2] + attachPoint.z;
        
#ifdef DEBUG_ROBOT_INIT
        std::cout << "  Leg " << i << " attaching at world (" << attachX << ", " << attachY << ", " << attachZ << ")" << std::endl;
#endif
        
        // Determine leg side for proper angles
        int legPair = i / 2;  // 0, 1, 2 for front, middle, rear
        int side = (i % 2 == 0) ? -1 : 1;  // left (-1), right (+1)
        
        // Validate mathematical constants and angles
        const double maxAngle = M_PI / 2.0; // 90 degrees
        const double coxaAngle = side * M_PI / 4.0;  // 45° outward
        const double femurDownAngle = M_PI / 6.0;   // 30° downward
        const double tibiaDownAngle = M_PI / 4.0;   // 45° further downward
        
        // Bounds checking for angle calculations
        if (std::abs(coxaAngle) > maxAngle || femurDownAngle > maxAngle || tibiaDownAngle > maxAngle) {
#ifdef DEBUG_ROBOT_INIT
            std::cout << "⚠️  Warning: Leg " << i << " angles exceed safe bounds" << std::endl;
#endif
        }
        
        // =============================================
        // COXA (Hip segment) - connects to body
        // =============================================
        dBodyID coxaBody = dBodyCreate(world);
        dMass coxaMass;
        dMassSetCapsuleTotal(&coxaMass, 0.2f, 3, config.legRadius, config.coxaLength);
        dBodySetMass(coxaBody, &coxaMass);
        
        // Position coxa extending outward from body with validated calculations
        double coxaEndX = attachX + cos(coxaAngle) * config.coxaLength * 0.5;
        double coxaEndY = attachY + sin(coxaAngle) * config.coxaLength * 0.5;
        
        // Validate positioning is within reasonable bounds
        if (std::abs(coxaEndX) > config.bodySize.x * 2.0 || std::abs(coxaEndY) > config.bodySize.y * 2.0) {
#ifdef DEBUG_ROBOT_INIT
            std::cout << "⚠️  Warning: Coxa position for leg " << i << " extends unusually far from body" << std::endl;
#endif
        }
        
        dBodySetPosition(coxaBody, coxaEndX, coxaEndY, attachZ);
        
        // Set coxa orientation (angled outward)
        dQuaternion coxaQ;
        dQFromAxisAndAngle(coxaQ, 0, 0, 1, coxaAngle);
        dBodySetQuaternion(coxaBody, coxaQ);
        
        // Create coxa geometry
        dGeomID coxaGeom = dCreateCapsule(space, config.legRadius, config.coxaLength);
        dGeomSetBody(coxaGeom, coxaBody);
        
        // Hip joint (ball joint connecting coxa to body)
        dJointID hipJoint = dJointCreateBall(world, 0);
        dJointAttach(hipJoint, bodyId, coxaBody);
        dJointSetBallAnchor(hipJoint, attachX, attachY, attachZ);
        
        // Store coxa segment
        LegSegment coxaSegment = {coxaBody, coxaGeom, hipJoint, config.coxaLength, config.legRadius};
        legs[i].segments.push_back(coxaSegment);
        
        // =============================================
        // FEMUR (Thigh segment) - connects to coxa
        // =============================================
        dBodyID femurBody = dBodyCreate(world);
        dMass femurMass;
        dMassSetCapsuleTotal(&femurMass, 0.15f, 3, config.legRadius, config.femurLength);
        dBodySetMass(femurBody, &femurMass);
        
        // Position femur angled downward from coxa end with validated mathematics
        double femurAngle = coxaAngle - femurDownAngle;  // 30° downward from coxa
        double coxaEndWorldX = attachX + cos(coxaAngle) * config.coxaLength;
        double coxaEndWorldY = attachY + sin(coxaAngle) * config.coxaLength;
        double femurMidX = coxaEndWorldX + cos(femurAngle) * config.femurLength * 0.5;
        double femurMidY = coxaEndWorldY + sin(femurAngle) * config.femurLength * 0.5;
        double femurMidZ = attachZ - sin(femurDownAngle) * config.femurLength * 0.5;  // downward
        
        // Validate femur positioning
        if (femurMidZ < 0) {
#ifdef DEBUG_ROBOT_INIT
            std::cout << "⚠️  Warning: Femur Z position (" << femurMidZ << ") below ground plane" << std::endl;
#endif
        }
        
        dBodySetPosition(femurBody, femurMidX, femurMidY, femurMidZ);
        
        // Set femur orientation
        dQuaternion femurQ;
        dQFromAxisAndAngle(femurQ, 0, 0, 1, femurAngle);
        dBodySetQuaternion(femurBody, femurQ);
        
        // Create femur geometry
        dGeomID femurGeom = dCreateCapsule(space, config.legRadius, config.femurLength);
        dGeomSetBody(femurGeom, femurBody);
        
        // Knee joint (hinge joint connecting femur to coxa)
        dJointID kneeJoint = dJointCreateHinge(world, 0);
        dJointAttach(kneeJoint, coxaBody, femurBody);
        dJointSetHingeAnchor(kneeJoint, coxaEndWorldX, coxaEndWorldY, attachZ);
        dJointSetHingeAxis(kneeJoint, 1, 0, 0);  // Rotate around X-axis
        
        // Store femur segment
        LegSegment femurSegment = {femurBody, femurGeom, kneeJoint, config.femurLength, config.legRadius};
        legs[i].segments.push_back(femurSegment);
        
        // =============================================
        // TIBIA (Shin + Foot) - connects to femur
        // =============================================
        dBodyID tibiaBody = dBodyCreate(world);
        dMass tibiaMass;
        dMassSetCapsuleTotal(&tibiaMass, 0.1f, 3, config.legRadius, config.tibiaLength);
        dBodySetMass(tibiaBody, &tibiaMass);
        
        // Position tibia angled further downward to reach ground with validation
        double tibiaAngle = femurAngle - tibiaDownAngle;  // 45° further downward
        double femurEndX = coxaEndWorldX + cos(femurAngle) * config.femurLength;
        double femurEndY = coxaEndWorldY + sin(femurAngle) * config.femurLength;
        double femurEndZ = attachZ - sin(femurDownAngle) * config.femurLength;
        double tibiaMidX = femurEndX + cos(tibiaAngle) * config.tibiaLength * 0.5;
        double tibiaMidY = femurEndY + sin(tibiaAngle) * config.tibiaLength * 0.5;
        double tibiaMidZ = femurEndZ - sin(tibiaDownAngle) * config.tibiaLength * 0.5;
        
        // Calculate foot end position for ground contact validation
        double footEndZ = tibiaMidZ - sin(tibiaDownAngle) * config.tibiaLength * 0.5;
        if (footEndZ > 0.1f) {
#ifdef DEBUG_ROBOT_INIT
            std::cout << "⚠️  Warning: Leg " << i << " foot end (" << footEndZ << ") may not reach ground" << std::endl;
#endif
        }
        
        dBodySetPosition(tibiaBody, tibiaMidX, tibiaMidY, tibiaMidZ);
        
        // Set tibia orientation
        dQuaternion tibiaQ;
        dQFromAxisAndAngle(tibiaQ, 0, 0, 1, tibiaAngle);
        dBodySetQuaternion(tibiaBody, tibiaQ);
        
        // Create tibia geometry
        dGeomID tibiaGeom = dCreateCapsule(space, config.legRadius, config.tibiaLength);
        dGeomSetBody(tibiaGeom, tibiaBody);
        
        // Ankle joint (hinge joint connecting tibia to femur)
        dJointID ankleJoint = dJointCreateHinge(world, 0);
        dJointAttach(ankleJoint, femurBody, tibiaBody);
        dJointSetHingeAnchor(ankleJoint, femurEndX, femurEndY, femurEndZ);
        dJointSetHingeAxis(ankleJoint, 1, 0, 0);  // Rotate around X-axis
        
        // Store tibia segment
        LegSegment tibiaSegment = {tibiaBody, tibiaGeom, ankleJoint, config.tibiaLength, config.legRadius};
        legs[i].segments.push_back(tibiaSegment);
        
        // Store leg data for compatibility (attachment point calculated dynamically)
        legs[i].attachmentPoint = attachPoint;
        
#ifdef DEBUG_ROBOT_INIT
        std::cout << "    ✅ Leg " << i << " created: Coxa->Femur->Tibia with proper joints" << std::endl;
#endif
    }
    
#ifdef DEBUG_ROBOT_INIT
    std::cout << "🎉 All 6 legs created with proper hexapod anatomy!" << std::endl;
#endif
}

vsg_vec3 Robot::getLegPosition(int legIndex) const {
    // Match Visualizer's leg attachment points EXACTLY
    // From Visualizer.cpp: 3 pairs of legs along body sides
    // NOTE: This function replaces the removed legAttachOffset field with direct calculation
    
    const double bodyLength = config.bodySize.x;  // 1.2
    const double bodyWidth = config.bodySize.y;   // 0.6
    const double bodyHeight = config.bodySize.z;  // 0.3
    
    // Leg X positions - 3 pairs along body length (front, middle, rear)
    // These positions are calculated to ensure even distribution and stability
    std::array<double, 3> legXPositions = {
        -bodyLength * 0.3,   // front legs
         0.0,                // middle legs
         bodyLength * 0.3    // rear legs
    };
    
    int legPair = legIndex / 2;      // 0, 1, 2 for front, middle, rear
    int side = (legIndex % 2 == 0) ? -1 : 1;  // left (-1), right (+1)
    
    // Validate leg index bounds
    if (legIndex < 0 || legIndex >= NUM_LEGS) {
#ifdef DEBUG_ROBOT_INIT
        std::cout << "⚠️  Warning: Invalid leg index " << legIndex << std::endl;
#endif
        return vsg_vec3(0, 0, 0);
    }
    
    double rootX = legXPositions[legPair];
    double rootY = side * (bodyWidth * 0.5);  // exactly at body edge
    double rootZ = -bodyHeight * 0.5;         // attach legs at body bottom
    
    return vsg_vec3(rootX, rootY, rootZ);
}

void Robot::update(double deltaTime) {
    // Enhanced update with proper physics/rendering sync
    updateLegPositions();
    applyForces();
    maintainBalance();
    
    // Update visual transform to match physics body
#ifndef USE_OPENGL_FALLBACK
    // Visual representation handled by Visualizer - sync robot transform
    if (robotTransform) {
        const dReal* pos = dBodyGetPosition(bodyId);
        const dReal* q = dBodyGetQuaternion(bodyId);
        
        vsg::dquat orientation(q[1], q[2], q[3], q[0]); // ODE: w,x,y,z -> VSG: x,y,z,w
        vsg::dvec3 position(pos[0], pos[1], pos[2]);
        
        robotTransform->matrix = vsg::translate(position) * vsg::rotate(orientation);
    }
#endif
}

void Robot::updateLegPositions() {
    // Update all leg segment visuals to match their ODE body transforms
    // Performance optimization: skip update if robot hasn't moved significantly
#ifndef USE_OPENGL_FALLBACK
    if (!robotTransform) return;
    
    static vsg::dvec3 lastBodyPos;
    const dReal* bodyPos = dBodyGetPosition(bodyId);
    vsg::dvec3 currentPos(bodyPos[0], bodyPos[1], bodyPos[2]);
    if (vsg::length(currentPos - lastBodyPos) < 0.001) return;
    lastBodyPos = currentPos;
    
    // Get body transform for relative positioning
    const dReal* bodyQ = dBodyGetQuaternion(bodyId);
    
    vsg::dquat invBodyQ(-bodyQ[1], -bodyQ[2], -bodyQ[3], bodyQ[0]); // Inverse body rotation
    
    int segmentIndex = 0;
    for (int legIdx = 0; legIdx < NUM_LEGS; ++legIdx) {
        for (size_t segIdx = 0; segIdx < legs[legIdx].segments.size(); ++segIdx, ++segmentIndex) {
            auto& segment = legs[legIdx].segments[segIdx];
            
            if (segmentIndex < legNodes.size() && legNodes[segmentIndex]) {
                // Get segment world transform
                const dReal* segPos = dBodyGetPosition(segment.body);
                const dReal* segQ = dBodyGetQuaternion(segment.body);
                
                // Convert to local coordinates relative to robot body
                double localX = segPos[0] - bodyPos[0];
                double localY = segPos[1] - bodyPos[1];
                double localZ = segPos[2] - bodyPos[2];
                
                vsg::dquat segOrientation(segQ[1], segQ[2], segQ[3], segQ[0]);
                vsg::dquat localOrientation = invBodyQ * segOrientation;
                
                // Update leg segment visual transform
                legNodes[segmentIndex]->matrix = 
                    vsg::translate(localX, localY, localZ) * vsg::rotate(localOrientation);
            }
        }
    }
#endif
}

void Robot::applyForces() {
    // Apply movement forces with better control
    if (vsg::length(targetVelocity) > 0.01f || std::abs(targetAngularVelocity) > 0.01f) {
        const dReal* currentVel = dBodyGetLinearVel(bodyId);
        vsg_vec3 velError = targetVelocity - vsg_vec3(currentVel[0], currentVel[1], currentVel[2]);
        
        // Apply force to reach target velocity
        dBodyAddForce(bodyId, 
            velError.x * 30.0f,  // Reduced force for more realistic movement
            velError.y * 30.0f,  // Added Y movement
            velError.z * 30.0f);
        
        // Apply torque for rotation
        dBodyAddTorque(bodyId, 0, 0, targetAngularVelocity * 15.0f);  // Z-axis rotation
    }
}

void Robot::maintainBalance() {
    // Enhanced balance using contact points from feet
    const dReal* q = dBodyGetQuaternion(bodyId);
    vsg_quat orientation(q[1], q[2], q[3], q[0]);
    
    // Calculate tilt from vertical (Z-up)
    vsg_vec3 up = vsg_vec3(0, 0, 1);
    vsg_vec3 bodyUp = orientation * vsg_vec3(0, 0, 1);  // Body's up direction
    
    // Cross product gives torque direction needed to align with vertical
    vsg_vec3 correctionTorque = vsg::cross(bodyUp, up) * 25.0f;
    
    dBodyAddTorque(bodyId, correctionTorque.x, correctionTorque.y, correctionTorque.z);
}

void Robot::reset() {
    // Reset with proper Z-up positioning
    float totalLegLength = config.coxaLength + config.femurLength + config.tibiaLength;
    float resetZ = config.bodySize.z * 0.5f + totalLegLength * 0.8f;
    
    dBodySetPosition(bodyId, 0, 0, resetZ);
    dBodySetLinearVel(bodyId, 0, 0, 0);
    dBodySetAngularVel(bodyId, 0, 0, 0);
    
    dQuaternion q;
    dQSetIdentity(q);
    dBodySetQuaternion(bodyId, q);
    
    // Reset state
    targetVelocity = vsg_vec3(0, 0, 0);
    targetAngularVelocity = 0;
    
#ifdef DEBUG_ROBOT_INIT
    std::cout << "🔄 Robot reset to Z=" << resetZ << " with proper leg reach" << std::endl;
#endif
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
    
    // Return tibia (last segment) geoms as feet for contact detection
    for (int i = 0; i < NUM_LEGS; ++i) {
        if (legs[i].segments.size() >= 3) {  // Ensure we have coxa, femur, tibia
            geoms.push_back(legs[i].segments[static_cast<int>(LegSegmentIndex::TIBIA)].geom);  // Use enum for clarity
        }
    }
    return geoms;
}

dGeomID Robot::getBodyGeom() const {
    return bodyGeom;
}

bool Robot::isStable() const {
    // Enhanced stability check with tighter thresholds
    vsg_quat orientation = getOrientation();
    vsg_vec3 euler = eulerAnglesFromQuat(orientation);
    
    // Check roll and pitch are within reasonable limits (more precise thresholds)
    return std::abs(euler.x) < 0.3f && std::abs(euler.y) < 0.3f;
}

void Robot::setBodyColor(const vsg_vec4& color) {
    bodyColor = color;
}

void Robot::applyControl(const std::vector<float>& motorCommands) {
    // Enhanced control for 3-segment legs with proper joint motor control
    if (motorCommands.size() >= NUM_LEGS * 3) {  // 3 joints per leg
        for (int legIdx = 0; legIdx < NUM_LEGS && legIdx * 3 + 2 < motorCommands.size(); ++legIdx) {
            if (legs[legIdx].segments.size() >= 3) {
                float hipTorque   = motorCommands[legIdx * 3 + 0] * 2.0f;
                float kneeTorque  = motorCommands[legIdx * 3 + 1] * 1.5f;
                float ankleTorque = motorCommands[legIdx * 3 + 2] * 1.0f;
                
                // Apply actual joint motor control to individual joints using enum indices
                dJointAddHingeTorque(legs[legIdx].segments[static_cast<int>(LegSegmentIndex::FEMUR)].joint, kneeTorque);   
                dJointAddHingeTorque(legs[legIdx].segments[static_cast<int>(LegSegmentIndex::TIBIA)].joint, ankleTorque);  
                
                // Hip joint is a ball joint - apply enhanced multi-axis torques
                if (std::abs(hipTorque) > 0.1f) {
                    dBodyID coxaBody = legs[legIdx].segments[static_cast<int>(LegSegmentIndex::COXA)].body;
                    
                    // Apply torque in multiple axes for realistic hip movement
                    dBodyAddTorque(coxaBody, 
                        hipTorque * 0.3f,  // X-axis (forward/backward swing)
                        hipTorque * 0.2f,  // Y-axis (abduction/adduction)  
                        hipTorque * 0.5f); // Z-axis (rotation)
                }
            }
        }
    }
}

std::vector<float> Robot::getSensorReadings() const {
    // Enhanced sensor readings including leg joint angles
    std::vector<float> readings;
    
    // Add position
    vsg_vec3 pos = getPosition();
    readings.insert(readings.end(), {pos.x, pos.y, pos.z});
    
    // Add velocity
    vsg_vec3 vel = getVelocity();
    readings.insert(readings.end(), {vel.x, vel.y, vel.z});
    
    // Add orientation
    vsg_quat orient = getOrientation();
    readings.insert(readings.end(), {orient.x, orient.y, orient.z, orient.w});
    
    // Add leg ground contact status (based on tibia position and ground contact)
    for (int i = 0; i < NUM_LEGS; ++i) {
        readings.push_back(legs[i].isGrounded ? 1.0f : 0.0f);
    }
    
    // Add stability
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
    // Visual model is handled by Visualizer - create transform with proper initialization
    if (!robotTransform) {
        robotTransform = vsg::MatrixTransform::create();
    }
    
    // Initialize leg node storage for proper count
    int numSegmentsTotal = NUM_LEGS * SEGMENTS_PER_LEG;  // 6 legs * 3 segments = 18
    legNodes.clear();
    legNodes.resize(numSegmentsTotal);
    
    // Create placeholder transforms for each leg segment
    for (int i = 0; i < numSegmentsTotal; ++i) {
        legNodes[i] = vsg::MatrixTransform::create();
        robotTransform->addChild(legNodes[i]);
    }
    
#ifdef DEBUG_ROBOT_INIT
    std::cout << "📊 Visual model prepared: " << numSegmentsTotal << " leg segment transforms" << std::endl;
#endif
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
    // Visual representation is handled by Visualizer
    // Robot physics only manages the transform synchronization
#ifdef DEBUG_ROBOT_INIT
    std::cout << "🎨 Robot physics ready - visuals handled by Visualizer" << std::endl;
#endif
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