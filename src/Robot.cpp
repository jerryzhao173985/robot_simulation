#include "Robot.h"
#include "DebugOutput.h"
#include "Sensor.h"
#include "Actuator.h"
#include "PhysicsWorld.h"
#include "NoiseManager.h"
#include <array>
#include <iostream>
#include <sstream>
#include <cmath>

// Debug control - now controlled by DebugOutput.h
// #define DEBUG_ROBOT_INIT

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
    
    // Initialize sensors and actuators
    createSensors();
    createActuators();
    
    // Initialize visual representation
    createVisualModel();
    addToScene(sceneGraph);
    
#ifdef DEBUG_ROBOT_INIT
    std::cout << "🦾 Robot initialized with proper hexapod anatomy!" << std::endl;
    // Verify leg attachment positions match Visualizer
    for (int i = 0; i < NUM_LEGS; ++i) {
        vsg_vec3 pos = getLegPosition(i);
        std::stringstream ss;
        ss << "Leg " << i << " attachment (X, Y, Z): ("
           << pos.x << ", " << pos.y << ", " << pos.z << ")";
        DEBUG_VERBOSE(ss.str());
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
            if (segment.body) {
                dBodyDestroy(segment.body);
            }
            if (segment.joint) {
                dJointDestroy(segment.joint);
            }
            // Note: geometries (including footGeom) are destroyed by PhysicsWorld
        }
    }
}

void Robot::createBody() {
    // Create main body mass
    dMass mass;
    dMassSetBoxTotal(&mass, 10.0f, config.bodySize.x, config.bodySize.y, config.bodySize.z); // Increased mass for stability
    dBodySetMass(bodyId, &mass);
    // Add stronger damping to prevent oscillations and sinking
    dBodySetLinearDamping(bodyId, 0.01f);   // Small linear damping
    dBodySetAngularDamping(bodyId, 0.5f);   // Higher angular damping for stability
    
    // Disable auto-disable to keep robot always active
    dBodySetAutoDisableFlag(bodyId, 0);
    dBodySetAutoDisableLinearThreshold(bodyId, 0.0);
    dBodySetAutoDisableAngularThreshold(bodyId, 0.0);
    
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
    
    // Set initial height to place feet just above ground level
    // Physics feet are at Z=0.339559 when body is at 0.95
    // So to put feet at ground (Z=0), body should be at 0.95 - 0.339559 = 0.610441
    // Start much higher to ensure clean settling without penetration
    float initZ = 1.0f; // This places feet well above ground for safe settling
    dBodySetPosition(bodyId, 0.0f, 0.0f, initZ);
    
    // No initial velocity - let robot settle naturally
    dBodySetLinearVel(bodyId, 0.0f, 0.0f, 0.0f);
    dBodySetAngularVel(bodyId, 0.0f, 0.0f, 0.0f); // Also zero angular velocity
    
    // Disable body until legs are created to prevent premature physics
    dBodyDisable(bodyId);
    
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
        
        // Match visual model angles for proper ground contact
        const double maxAngle = M_PI / 2.0; // 90 degrees
        const double coxaAngle = side * (70.0 * M_PI / 180.0);  // 70° outward - match visual
        const double femurDownAngle = 20.0 * M_PI / 180.0;      // 20° downward - match visual
        const double tibiaDownAngle = 65.0 * M_PI / 180.0;      // 65° total angle to match visual
        
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
        dBodySetLinearDamping(coxaBody, 0.001f);    // Very light linear damping
        dBodySetAngularDamping(coxaBody, 0.1f);     // Light angular damping
        dBodySetAutoDisableFlag(coxaBody, 0);       // Keep leg segments always active
        
        // Position coxa extending outward from body with slight upward angle
        double coxaUpAngle = 5.0 * M_PI / 180.0; // 5° upward to match visual
        double coxaEndX = attachX + cos(coxaAngle) * cos(coxaUpAngle) * config.coxaLength * 0.5;
        double coxaEndY = attachY + sin(coxaAngle) * cos(coxaUpAngle) * config.coxaLength * 0.5;
        double coxaEndZ = attachZ + sin(coxaUpAngle) * config.coxaLength * 0.5;
        
        // Validate positioning is within reasonable bounds
        if (std::abs(coxaEndX) > config.bodySize.x * 2.0 || std::abs(coxaEndY) > config.bodySize.y * 2.0) {
#ifdef DEBUG_ROBOT_INIT
            std::cout << "⚠️  Warning: Coxa position for leg " << i << " extends unusually far from body" << std::endl;
#endif
        }
        
        dBodySetPosition(coxaBody, coxaEndX, coxaEndY, coxaEndZ);
        
        // Set coxa orientation (angled outward)
        dQuaternion coxaQ;
        dQFromAxisAndAngle(coxaQ, 0, 0, 1, coxaAngle);
        dBodySetQuaternion(coxaBody, coxaQ);
        
        // Create coxa geometry
        dGeomID coxaGeom = dCreateCapsule(space, config.legRadius, config.coxaLength);
        dGeomSetBody(coxaGeom, coxaBody);
        // No offset rotation - body orientation handles segment alignment
        
        // Hip joint (ball joint connecting coxa to body)
        dJointID hipJoint = dJointCreateBall(world, 0);
        dJointAttach(hipJoint, bodyId, coxaBody);
        dJointSetBallAnchor(hipJoint, attachX, attachY, attachZ);
        
        // Store coxa segment
        LegSegment coxaSegment = {coxaBody, coxaGeom, nullptr, hipJoint, nullptr, config.coxaLength, config.legRadius};
        legs[i].segments.push_back(coxaSegment);
        
        // =============================================
        // FEMUR (Thigh segment) - connects to coxa
        // =============================================
        dBodyID femurBody = dBodyCreate(world);
        dMass femurMass;
        dMassSetCapsuleTotal(&femurMass, 0.15f, 3, config.legRadius, config.femurLength);
        dBodySetMass(femurBody, &femurMass);
        dBodySetLinearDamping(femurBody, 0.001f);   // Very light linear damping
        dBodySetAngularDamping(femurBody, 0.1f);    // Light angular damping
        dBodySetAutoDisableFlag(femurBody, 0);      // Keep leg segments always active
        
        // Position femur angled downward from coxa end with validated mathematics
        double coxaEndWorldX = attachX + cos(coxaAngle) * cos(coxaUpAngle) * config.coxaLength;
        double coxaEndWorldY = attachY + sin(coxaAngle) * cos(coxaUpAngle) * config.coxaLength;
        double coxaEndWorldZ = attachZ + sin(coxaUpAngle) * config.coxaLength;
        // Femur extends in same radial direction as coxa but angled down
        double femurMidX = coxaEndWorldX + cos(coxaAngle) * cos(femurDownAngle) * config.femurLength * 0.5;
        double femurMidY = coxaEndWorldY + sin(coxaAngle) * cos(femurDownAngle) * config.femurLength * 0.5;
        double femurMidZ = coxaEndWorldZ - sin(femurDownAngle) * config.femurLength * 0.5;  // downward
        
        // Validate femur positioning
        if (femurMidZ < 0) {
#ifdef DEBUG_ROBOT_INIT
            std::cout << "⚠️  Warning: Femur Z position (" << femurMidZ << ") below ground plane" << std::endl;
#endif
        }
        
        dBodySetPosition(femurBody, femurMidX, femurMidY, femurMidZ);
        
        // Set femur orientation - aligned with leg radial direction
        dQuaternion femurQ;
        dQFromAxisAndAngle(femurQ, 0, 0, 1, coxaAngle);
        dBodySetQuaternion(femurBody, femurQ);
        
        // Create femur geometry with proper orientation
        dGeomID femurGeom = dCreateCapsule(space, config.legRadius, config.femurLength);
        dGeomSetBody(femurGeom, femurBody);
        // Rotate capsule to align along segment
        dMatrix3 femurR;
        double femurAngle = atan2(femurMidZ - coxaEndWorldZ, 
                                 sqrt(pow(femurMidX - coxaEndWorldX, 2) + pow(femurMidY - coxaEndWorldY, 2)));
        dRFromAxisAndAngle(femurR, 0, 1, 0, M_PI/2 - femurAngle);
        dGeomSetOffsetRotation(femurGeom, femurR);
        
        // Knee joint (hinge joint connecting femur to coxa)
        dJointID kneeJoint = dJointCreateHinge(world, 0);
        dJointAttach(kneeJoint, coxaBody, femurBody);
        dJointSetHingeAnchor(kneeJoint, coxaEndWorldX, coxaEndWorldY, coxaEndWorldZ);
        dJointSetHingeAxis(kneeJoint, 1, 0, 0);  // Rotate around X-axis
        // Set wider joint limits for natural movement
        dJointSetHingeParam(kneeJoint, dParamLoStop, -M_PI/3);  // -60 degrees
        dJointSetHingeParam(kneeJoint, dParamHiStop, M_PI/3);   // +60 degrees
        
        // Store femur segment
        LegSegment femurSegment = {femurBody, femurGeom, nullptr, kneeJoint, nullptr, config.femurLength, config.legRadius};
        legs[i].segments.push_back(femurSegment);
        
        // =============================================
        // TIBIA (Shin + Foot) - connects to femur
        // =============================================
        dBodyID tibiaBody = dBodyCreate(world);
        dMass tibiaMass;
        dMassSetCapsuleTotal(&tibiaMass, 0.1f, 3, config.legRadius, config.tibiaLength);
        dBodySetMass(tibiaBody, &tibiaMass);
        dBodySetLinearDamping(tibiaBody, 0.001f);   // Very light linear damping
        dBodySetAngularDamping(tibiaBody, 0.1f);    // Light angular damping
        dBodySetAutoDisableFlag(tibiaBody, 0);      // Keep leg segments always active
        
        // Position tibia angled further downward to reach ground with validation
        double femurEndX = coxaEndWorldX + cos(coxaAngle) * cos(femurDownAngle) * config.femurLength;
        double femurEndY = coxaEndWorldY + sin(coxaAngle) * cos(femurDownAngle) * config.femurLength;
        double femurEndZ = coxaEndWorldZ - sin(femurDownAngle) * config.femurLength;
        // Tibia uses same angle as visual model
        double tibiaTotalAngle = tibiaDownAngle;
        double tibiaMidX = femurEndX + cos(coxaAngle) * cos(tibiaTotalAngle) * config.tibiaLength * 0.5;
        double tibiaMidY = femurEndY + sin(coxaAngle) * cos(tibiaTotalAngle) * config.tibiaLength * 0.5;
        double tibiaMidZ = femurEndZ - sin(tibiaTotalAngle) * config.tibiaLength * 0.5;
        
        // Calculate foot end position for ground contact validation
        double footEndX = femurEndX + cos(coxaAngle) * cos(tibiaTotalAngle) * config.tibiaLength;
        double footEndY = femurEndY + sin(coxaAngle) * cos(tibiaTotalAngle) * config.tibiaLength;
        double footEndZ = femurEndZ - sin(tibiaTotalAngle) * config.tibiaLength;
        
#ifdef DEBUG_ROBOT_INIT
        std::cout << "  Leg " << i << " foot position: (" << footEndX << ", " << footEndY << ", " << footEndZ << ")" << std::endl;
        if (footEndZ > 0.1f) {
            std::cout << "⚠️  Warning: Leg " << i << " foot end (" << footEndZ << ") may not reach ground" << std::endl;
        }
#endif
        
        dBodySetPosition(tibiaBody, tibiaMidX, tibiaMidY, tibiaMidZ);
        
        // Set tibia orientation - aligned with leg radial direction
        dQuaternion tibiaQ;
        dQFromAxisAndAngle(tibiaQ, 0, 0, 1, coxaAngle);
        dBodySetQuaternion(tibiaBody, tibiaQ);
        
        // Create tibia geometry with proper orientation
        dGeomID tibiaGeom = dCreateCapsule(space, config.legRadius, config.tibiaLength);
        dGeomSetBody(tibiaGeom, tibiaBody);
        // Rotate capsule to align along segment
        dMatrix3 tibiaR;
        double tibiaAngle = atan2(tibiaMidZ - femurEndZ,
                                 sqrt(pow(tibiaMidX - femurEndX, 2) + pow(tibiaMidY - femurEndY, 2)));
        dRFromAxisAndAngle(tibiaR, 0, 1, 0, M_PI/2 - tibiaAngle);
        dGeomSetOffsetRotation(tibiaGeom, tibiaR);
        
        // Add foot box for absolutely reliable ground contact
        float footSize = config.legRadius * 8.0f;  // Increased from 6x to 8x for better stability
        dGeomID footGeom = dCreateBox(space, footSize, footSize, footSize);
        dGeomSetBody(footGeom, tibiaBody);
        // Position at foot end
        dGeomSetOffsetPosition(footGeom, 0, 0, -config.tibiaLength * 0.5f);
        
        // Ankle joint (hinge joint connecting tibia to femur)
        dJointID ankleJoint = dJointCreateHinge(world, 0);
        dJointAttach(ankleJoint, femurBody, tibiaBody);
        dJointSetHingeAnchor(ankleJoint, femurEndX, femurEndY, femurEndZ);
        dJointSetHingeAxis(ankleJoint, 1, 0, 0);  // Rotate around X-axis
        // Set wider joint limits for natural movement
        dJointSetHingeParam(ankleJoint, dParamLoStop, -M_PI/3);  // -60 degrees
        dJointSetHingeParam(ankleJoint, dParamHiStop, M_PI/3);   // +60 degrees
        
        // Store tibia segment with foot geometry
        LegSegment tibiaSegment = {tibiaBody, tibiaGeom, footGeom, ankleJoint, nullptr, config.tibiaLength, config.legRadius};
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

    // Re-enable body now that legs are attached
    dBodyEnable(bodyId);
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
    // Update noise manager time
    NoiseManager::getInstance().updateTime(deltaTime);
    
    // Enhanced update with proper physics/rendering sync
    updateLegPositions();
    applyForces();
    maintainBalance();
    
    // Update all sensors
    updateSensors(deltaTime);
    
    // Update all actuators
    for (auto& [name, actuator] : actuators) {
        if (actuator) {
            actuator->update(deltaTime);
        }
    }
    
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
        
        // Apply very gentle forces to prevent instability
        dBodyAddForce(bodyId, 
            velError.x * 5.0f,   // Much reduced force
            velError.y * 5.0f,   
            velError.z * 5.0f);
        
        // Apply torque for rotation
        dBodyAddTorque(bodyId, 0, 0, targetAngularVelocity * 5.0f);  // Reduced torque
    }
}

void Robot::maintainBalance() {
    // Enhanced PID-based balance control with debug info
    static int balanceCounter = 0;
    balanceCounter++;
    
    const dReal* q = dBodyGetQuaternion(bodyId);
    vsg_quat orientation(q[1], q[2], q[3], q[0]);
    
    // Calculate tilt from vertical (Z-up)
    vsg_vec3 up = vsg_vec3(0, 0, 1);
    vsg_vec3 bodyUp = orientation * vsg_vec3(0, 0, 1);  // Body's up direction
    
    // Calculate roll and pitch errors
    vsg_vec3 euler = eulerAnglesFromQuat(orientation);
    float rollError = euler.x;   // Target is 0
    float pitchError = euler.y;  // Target is 0
    
    // Debug output when noise is active
    float noiseLevel = NoiseManager::getInstance().getNoiseLevel();
    if (balanceCounter % 120 == 0 && noiseLevel > 0.0f) {
        std::cout << "\n[Robot::maintainBalance] Noise=" << noiseLevel 
                  << ", Roll=" << std::fixed << std::setprecision(1) << rollError * 180.0f/M_PI 
                  << "°, Pitch=" << pitchError * 180.0f/M_PI << "°" << std::endl;
    }
    
    // PID control for stabilization
    static float rollIntegral = 0.0f;
    static float pitchIntegral = 0.0f;
    static float lastRollError = 0.0f;
    static float lastPitchError = 0.0f;
    
    const float kp = 30.0f;   // Proportional gain (reduced from 100)
    const float ki = 5.0f;    // Integral gain (reduced from 10)
    const float kd = 10.0f;   // Derivative gain (reduced from 20)
    const float dt = 1.0f / 60.0f; // Physics timestep
    
    // Update integrals
    rollIntegral += rollError * dt;
    pitchIntegral += pitchError * dt;
    
    // Limit integral windup
    rollIntegral = std::clamp(rollIntegral, -1.0f, 1.0f);
    pitchIntegral = std::clamp(pitchIntegral, -1.0f, 1.0f);
    
    // Calculate derivatives
    float rollDerivative = (rollError - lastRollError) / dt;
    float pitchDerivative = (pitchError - lastPitchError) / dt;
    
    // PID outputs
    float rollCorrection = kp * rollError + ki * rollIntegral + kd * rollDerivative;
    float pitchCorrection = kp * pitchError + ki * pitchIntegral + kd * pitchDerivative;
    
    // Apply corrective torques with limits
    const dReal* angVel = dBodyGetAngularVel(bodyId);
    
    // Limit corrections to prevent overcompensation
    rollCorrection = std::clamp(rollCorrection, -50.0f, 50.0f);
    pitchCorrection = std::clamp(pitchCorrection, -50.0f, 50.0f);
    
    dBodyAddTorque(bodyId, 
        -rollCorrection - angVel[0] * 5.0f,    // X-axis (roll) - reduced damping
        -pitchCorrection - angVel[1] * 5.0f,   // Y-axis (pitch) - reduced damping
        -angVel[2] * 2.0f);                    // Z-axis damping - reduced
    
    // Update last errors
    lastRollError = rollError;
    lastPitchError = pitchError;
    
    // Adjust leg positions for active stabilization
    adjustLegsForBalance(rollError, pitchError);
}

void Robot::adjustLegsForBalance(float rollError, float pitchError) {
    // Active stabilization by adjusting leg joint angles
    // When tilting right (positive roll), extend left legs and retract right legs
    // When tilting forward (positive pitch), extend rear legs and retract front legs
    
    const float maxAdjustment = 0.2f; // Maximum joint adjustment in radians
    
    for (int i = 0; i < NUM_LEGS; ++i) {
        if (legs[i].segments.size() < 3) continue;
        
        // Determine leg position (left/right, front/rear)
        int side = (i % 2 == 0) ? -1 : 1;  // -1 for left, 1 for right
        int legPair = i / 2;  // 0=front, 1=middle, 2=rear
        float frontRearFactor = (legPair == 0) ? -1.0f : (legPair == 2) ? 1.0f : 0.0f;
        
        // Calculate adjustment based on tilt
        float rollAdjustment = -side * rollError * maxAdjustment;
        float pitchAdjustment = frontRearFactor * pitchError * maxAdjustment;
        
        // Apply to knee joint (femur-tibia connection)
        dJointID kneeJoint = legs[i].segments[static_cast<int>(LegSegmentIndex::FEMUR)].joint;
        if (kneeJoint) {
            float targetAngle = rollAdjustment + pitchAdjustment;
            // Use joint motor to adjust angle
            dJointSetHingeParam(kneeJoint, dParamVel, targetAngle * 10.0f); // Velocity control
            dJointSetHingeParam(kneeJoint, dParamFMax, 5.0f); // Motor force
        }
        
        // Also adjust ankle for better ground contact
        dJointID ankleJoint = legs[i].segments[static_cast<int>(LegSegmentIndex::TIBIA)].joint;
        if (ankleJoint) {
            float ankleAdjustment = -(rollAdjustment + pitchAdjustment) * 0.5f;
            dJointSetHingeParam(ankleJoint, dParamVel, ankleAdjustment * 10.0f);
            dJointSetHingeParam(ankleJoint, dParamFMax, 3.0f);
        }
    }
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
    
    // Return actual foot box geometries from tibia segments
    for (int i = 0; i < NUM_LEGS; ++i) {
        if (legs[i].segments.size() >= 3) {  // Ensure we have coxa, femur, tibia
            dGeomID footGeom = legs[i].segments[static_cast<int>(LegSegmentIndex::TIBIA)].footGeom;
            if (footGeom) {
                geoms.push_back(footGeom);  // Return the actual foot box geometry
            }
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
    // Motor control using the new actuator system
    if (motorCommands.size() >= NUM_LEGS * 3) {  // 3 joints per leg
        
        // Apply commands through actuators
        for (int legIdx = 0; legIdx < NUM_LEGS && legIdx * 3 + 2 < motorCommands.size(); ++legIdx) {
            if (legs[legIdx].segments.size() >= 3) {
                // Get motor commands for this leg
                float hipCmd   = motorCommands[legIdx * 3 + 0];
                float kneeCmd  = motorCommands[legIdx * 3 + 1];
                float ankleCmd = motorCommands[legIdx * 3 + 2];
                
                // Hip joint (seg0) is a ball joint - apply torque directly
                if (legs[legIdx].segments.size() > 0) {
                    dJointID hipJoint = legs[legIdx].segments[0].joint;
                    if (hipJoint && hipCmd != 0.0f) {
                        // Apply torque around vertical axis for turning
                        dBodyID body = legs[legIdx].segments[0].body;
                        if (body) {
                            dBodyAddTorque(body, 0, 0, hipCmd * 5.0f);
                        }
                    }
                }
                
                // Apply to knee motor (femur) - this is a hinge joint
                std::string kneeMotorName = "leg" + std::to_string(legIdx) + "_seg1_motor";
                if (auto* kneeMotor = getActuator(kneeMotorName)) {
                    Actuator::Command cmd;
                    cmd.timestamp = 0;
                    // Add visible noise when noise level > 0 by setting target velocity
                    float noiseLevel = NoiseManager::getInstance().getNoiseLevel();
                    if (noiseLevel > 0.0f && std::abs(kneeCmd) < 0.01f) {
                        // When no command, add pure noise motion
                        cmd.targetValue = 0.0f; // Let noise in actuator do the work
                    } else {
                        cmd.targetValue = kneeCmd * 3.0f;
                    }
                    cmd.maxEffort = 8.0f;
                    kneeMotor->applyCommand(cmd);
                }
                
                // Apply to ankle motor (tibia) - this is a hinge joint
                std::string ankleMotorName = "leg" + std::to_string(legIdx) + "_seg2_motor";
                if (auto* ankleMotor = getActuator(ankleMotorName)) {
                    Actuator::Command cmd;
                    cmd.timestamp = 0;
                    // Add visible noise when noise level > 0
                    float noiseLevel = NoiseManager::getInstance().getNoiseLevel();
                    if (noiseLevel > 0.0f && std::abs(ankleCmd) < 0.01f) {
                        // When no command, add pure noise motion
                        cmd.targetValue = 0.0f; // Let noise in actuator do the work
                    } else {
                        cmd.targetValue = ankleCmd * 2.0f;
                    }
                    cmd.maxEffort = 5.0f;
                    ankleMotor->applyCommand(cmd);
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

void Robot::createSensors() {
    // Clear any existing sensors
    sensors.clear();
    footContactSensors.clear();
    jointPositionSensors.clear();
    jointVelocitySensors.clear();
    
    // Create IMU sensor on main body
    auto imu = std::make_unique<IMUSensor>("body_imu", bodyId);
    imu->setNoise(0.0f, 0.01f); // Small noise for realism
    imuSensor = imu.get();
    sensors["body_imu"] = std::move(imu);
    
    // Create sensors for each leg
    for (int legIdx = 0; legIdx < NUM_LEGS; ++legIdx) {
        const auto& leg = legs[legIdx];
        
        // Create foot contact sensor for tibia segment
        if (leg.segments.size() >= 3 && leg.segments[2].footGeom) {
            std::string contactName = "foot_contact_" + std::to_string(legIdx);
            auto contact = std::make_unique<ContactSensor>(
                contactName, 
                leg.segments[2].footGeom, 
                nullptr  // Will be set later when physics world is available
            );
            footContactSensors.push_back(contact.get());
            sensors[contactName] = std::move(contact);
        }
        
        // Create joint sensors for each segment
        for (size_t segIdx = 0; segIdx < leg.segments.size(); ++segIdx) {
            const auto& segment = leg.segments[segIdx];
            if (!segment.joint) continue;
            
            std::string baseName = "leg" + std::to_string(legIdx) + "_seg" + std::to_string(segIdx);
            
            // Position sensor
            auto posSensor = std::make_unique<JointPositionSensor>(
                baseName + "_pos", 
                segment.joint
            );
            posSensor->setNoise(0.0f, 0.001f); // 1 milliradian noise
            jointPositionSensors.push_back(posSensor.get());
            sensors[baseName + "_pos"] = std::move(posSensor);
            
            // Velocity sensor
            auto velSensor = std::make_unique<JointVelocitySensor>(
                baseName + "_vel", 
                segment.joint
            );
            velSensor->setNoise(0.0f, 0.01f); // Small velocity noise
            jointVelocitySensors.push_back(velSensor.get());
            sensors[baseName + "_vel"] = std::move(velSensor);
        }
    }
    
    DEBUG_INFO("Created " + std::to_string(sensors.size()) + " sensors for robot");
}

void Robot::setPhysicsWorld(PhysicsWorld* world) {
    // Set physics world for contact sensors
    for (auto& contactSensor : footContactSensors) {
        if (auto* sensor = dynamic_cast<ContactSensor*>(contactSensor)) {
            sensor->setPhysicsWorld(world);
        }
    }
}

void Robot::createActuators() {
    // Clear any existing actuators
    actuators.clear();
    jointMotors.clear();
    
    // Create actuators for each leg joint
    for (int legIdx = 0; legIdx < NUM_LEGS; ++legIdx) {
        const auto& leg = legs[legIdx];
        
        for (size_t segIdx = 0; segIdx < leg.segments.size(); ++segIdx) {
            const auto& segment = leg.segments[segIdx];
            if (!segment.joint) continue;
            
            std::string motorName = "leg" + std::to_string(legIdx) + 
                                   "_seg" + std::to_string(segIdx) + "_motor";
            
            // Skip motor creation for ball joints (hip/coxa) - they can't be velocity controlled
            if (segIdx == 0) {
                // Hip is a ball joint - skip motor creation
                // Ball joints will be controlled directly with torques in applyControl
                continue;
            }
            
            // Create velocity-controlled motor for hinge joints only
            auto motor = std::make_unique<VelocityMotor>(motorName, segment.joint);
            
            // Set appropriate limits based on segment type
            if (segIdx == 1) {  // Knee/femur joint
                motor->setVelocityLimit(7.0f);
                motor->setEffortLimit(12.0f);
            } else {  // Ankle/tibia joint
                motor->setVelocityLimit(7.0f);
                motor->setEffortLimit(10.0f);
            }
            
            jointMotors.push_back(motor.get());
            actuators[motorName] = std::move(motor);
        }
    }
    
    DEBUG_INFO("Created " + std::to_string(actuators.size()) + " actuators for robot");
}

void Robot::updateSensors(double deltaTime) {
    // Update all sensors
    for (auto& [name, sensor] : sensors) {
        if (sensor) {
            sensor->update(deltaTime);
        }
    }
}

Sensor* Robot::getSensor(const std::string& name) {
    auto it = sensors.find(name);
    return (it != sensors.end()) ? it->second.get() : nullptr;
}

Actuator* Robot::getActuator(const std::string& name) {
    auto it = actuators.find(name);
    return (it != actuators.end()) ? it->second.get() : nullptr;
}

std::vector<std::string> Robot::getSensorNames() const {
    std::vector<std::string> names;
    names.reserve(sensors.size());
    for (const auto& [name, sensor] : sensors) {
        names.push_back(name);
    }
    return names;
}

std::vector<std::string> Robot::getActuatorNames() const {
    std::vector<std::string> names;
    names.reserve(actuators.size());
    for (const auto& [name, actuator] : actuators) {
        names.push_back(name);
    }
    return names;
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