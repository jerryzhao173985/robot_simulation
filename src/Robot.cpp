#include "Robot.h"
#include <cmath>
#include <algorithm>
#include <iostream>

Robot::Robot(dWorldID world, dSpaceID space, vsg::ref_ptr<vsg::Group> sceneGraph)
    : world(world), space(space), sceneGraph(sceneGraph) {
    
    // Create robot group
    robotGroup = vsg::Group::create();
    sceneGraph->addChild(robotGroup);
    
    // Initialize robot components
    createBody();
    createLegs();
    createSensors();
    
    // Setup initial state
    targetVelocity = vsg::vec3(0.0f, 0.0f, 0.0f);
    targetAngularVelocity = 0.0f;
    gaitPhase = 0.0f;
    energyConsumption = 0.0f;
}

Robot::~Robot() {
    // Cleanup ODE bodies and geometries
    if (bodyGeom) dGeomDestroy(bodyGeom);
    if (bodyID) dBodyDestroy(bodyID);
    
    for (auto& leg : legs) {
        for (auto& segment : leg.segments) {
            if (segment.joint) dJointDestroy(segment.joint);
            if (segment.geom) dGeomDestroy(segment.geom);
            if (segment.body) dBodyDestroy(segment.body);
        }
    }
}

void Robot::createBody() {
    // Create main body
    bodyID = dBodyCreate(world);
    
    // Body dimensions
    const float bodyLength = 2.0f;
    const float bodyWidth = 1.5f;
    const float bodyHeight = 0.4f;
    const float bodyMass = 10.0f;
    
    // Set body mass
    dMassSetBoxTotal(&mass, bodyMass, bodyLength, bodyHeight, bodyWidth);
    dBodySetMass(bodyID, &mass);
    
    // Create body geometry
    bodyGeom = dCreateBox(space, bodyLength, bodyHeight, bodyWidth);
    dGeomSetBody(bodyGeom, bodyID);
    
    // Set initial position
    dBodySetPosition(bodyID, 0, 2.0f, 0);
    
    // Create visual representation
    bodyTransform = vsg::MatrixTransform::create();
    
    // Create material
    material = vsg::PhongMaterialValue::create();
    material->diffuse = bodyColor;
    material->specular = vsg::vec4(1.0f, 1.0f, 1.0f, 1.0f);
    material->shininess = 128.0f * (1.0f - roughnessValue);
    
    // Create body geometry
    auto builder = vsg::Builder::create();
    auto box = vsg::Box::create();
    box->min = vsg::vec3(-bodyLength/2, -bodyHeight/2, -bodyWidth/2);
    box->max = vsg::vec3(bodyLength/2, bodyHeight/2, bodyWidth/2);
    
    vsg::GeometryInfo geomInfo;
    geomInfo.box = box;
    geomInfo.color = bodyColor;
    
    auto stateGroup = vsg::StateGroup::create();
    auto phongMaterial = vsg::PhongMaterial::create();
    phongMaterial->value() = material;
    stateGroup->add(vsg::BindGraphicsPipeline::create(builder->createGraphicsPipeline(phongMaterial)));
    
    auto bodyNode = builder->createBox(geomInfo);
    stateGroup->addChild(bodyNode);
    bodyTransform->addChild(stateGroup);
    
    robotGroup->addChild(bodyTransform);
    
    // Add decorative elements
    addBodyDecorations();
}

void Robot::createLegs() {
    // Hexapod configuration
    const float legSpacing = 0.7f;
    const float legOffsetX = 0.8f;
    
    // Define attachment points for 6 legs
    vsg::vec3 attachmentPoints[NUM_LEGS] = {
        vsg::vec3(legOffsetX, 0, legSpacing),      // Front right
        vsg::vec3(legOffsetX, 0, -legSpacing),     // Front left
        vsg::vec3(0, 0, legSpacing),               // Middle right
        vsg::vec3(0, 0, -legSpacing),              // Middle left
        vsg::vec3(-legOffsetX, 0, legSpacing),     // Back right
        vsg::vec3(-legOffsetX, 0, -legSpacing)     // Back left
    };
    
    for (int i = 0; i < NUM_LEGS; ++i) {
        legs[i].attachmentPoint = attachmentPoints[i];
        legs[i].currentAngle = 0;
        legs[i].targetAngle = 0;
        legs[i].isGrounded = false;
        
        createLegSegments(i);
    }
}

void Robot::createLegSegments(int legIndex) {
    auto& leg = legs[legIndex];
    
    // Segment dimensions
    const float segmentLengths[SEGMENTS_PER_LEG] = {0.3f, 0.5f, 0.4f};
    const float segmentRadius = 0.05f;
    const float segmentMass = 0.2f;
    
    dBodyID prevBody = bodyID;
    vsg::vec3 currentPos = leg.attachmentPoint;
    
    for (int i = 0; i < SEGMENTS_PER_LEG; ++i) {
        LegSegment segment;
        segment.length = segmentLengths[i];
        segment.radius = segmentRadius;
        
        // Create ODE body
        segment.body = dBodyCreate(world);
        
        // Set mass
        dMass segMass;
        dMassSetCylinderTotal(&segMass, segmentMass, 3, segmentRadius, segment.length);
        dBodySetMass(segment.body, &segMass);
        
        // Create geometry
        segment.geom = dCreateCylinder(space, segmentRadius, segment.length);
        dGeomSetBody(segment.geom, segment.body);
        
        // Position segment
        float angle = (i == 0) ? -M_PI/4 : -M_PI/3;
        vsg::vec3 offset(0, -segment.length * sin(angle) / 2, 0);
        currentPos += offset;
        
        const dReal* bodyPos = dBodyGetPosition(bodyID);
        dBodySetPosition(segment.body, 
            bodyPos[0] + currentPos.x,
            bodyPos[1] + currentPos.y,
            bodyPos[2] + currentPos.z);
        
        // Create joint
        if (i == 0) {
            // Hip joint - 3 DOF
            segment.joint = dJointCreateBall(world, 0);
            dJointAttach(segment.joint, prevBody, segment.body);
            dJointSetBallAnchor(segment.joint,
                bodyPos[0] + leg.attachmentPoint.x,
                bodyPos[1] + leg.attachmentPoint.y,
                bodyPos[2] + leg.attachmentPoint.z);
        } else {
            // Knee/ankle joint - 1 DOF
            segment.joint = dJointCreateHinge(world, 0);
            dJointAttach(segment.joint, prevBody, segment.body);
            
            const dReal* prevPos = dBodyGetPosition(prevBody);
            dJointSetHingeAnchor(segment.joint,
                prevPos[0], prevPos[1] - segmentLengths[i-1]/2, prevPos[2]);
            dJointSetHingeAxis(segment.joint, 0, 0, 1);
            
            // Set joint limits
            dJointSetHingeParam(segment.joint, dParamLoStop, -M_PI/2);
            dJointSetHingeParam(segment.joint, dParamHiStop, M_PI/2);
        }
        
        // Create visual
        segment.transform = vsg::MatrixTransform::create();
        
        auto builder = vsg::Builder::create();
        auto cylinder = vsg::Cylinder::create();
        cylinder->radius = segmentRadius;
        cylinder->height = segment.length;
        
        vsg::GeometryInfo geomInfo;
        geomInfo.cylinder = cylinder;
        geomInfo.color = vsg::vec4(0.3f, 0.3f, 0.3f, 1.0f);
        
        auto segmentNode = builder->createCylinder(geomInfo);
        segment.transform->addChild(segmentNode);
        robotGroup->addChild(segment.transform);
        
        leg.segments.push_back(segment);
        prevBody = segment.body;
        currentPos.y -= segment.length * sin(angle);
    }
}

void Robot::createSensors() {
    // Add proximity sensors
    for (int i = 0; i < 8; ++i) {
        Sensor sensor;
        sensor.type = Sensor::PROXIMITY;
        float angle = i * M_PI / 4;
        sensor.position = vsg::vec3(cos(angle) * 1.2f, 0.2f, sin(angle) * 1.2f);
        sensor.orientation = vsg::vec3(cos(angle), 0, sin(angle));
        sensor.range = 5.0f;
        sensor.currentValue = 0.0f;
        sensors.push_back(sensor);
    }
    
    // Add gyroscope
    Sensor gyro;
    gyro.type = Sensor::GYROSCOPE;
    gyro.position = vsg::vec3(0, 0, 0);
    sensors.push_back(gyro);
    
    // Add accelerometer
    Sensor accel;
    accel.type = Sensor::ACCELEROMETER;
    accel.position = vsg::vec3(0, 0, 0);
    sensors.push_back(accel);
    
    // Add contact sensors for each foot
    for (int i = 0; i < NUM_LEGS; ++i) {
        Sensor contact;
        contact.type = Sensor::CONTACT;
        contact.position = legs[i].attachmentPoint;
        sensors.push_back(contact);
    }
}

void Robot::update(double deltaTime) {
    updatePhysics(deltaTime);
    updateGait(gaitPhase);
    detectGround();
    
    if (stabilizationEnabled) {
        stabilize();
    }
    
    updateVisuals();
    
    // Update energy consumption
    float powerUsage = 0.0f;
    for (const auto& leg : legs) {
        for (const auto& segment : leg.segments) {
            if (segment.joint) {
                // Simple power model based on joint velocity
                const dReal* vel = dBodyGetAngularVel(segment.body);
                powerUsage += vsg::length(vsg::vec3(vel[0], vel[1], vel[2])) * 2.0f;
            }
        }
    }
    energyConsumption = powerUsage;
    
    gaitPhase += gaitSpeed * deltaTime;
    if (gaitPhase > 2 * M_PI) {
        gaitPhase -= 2 * M_PI;
    }
}

void Robot::updatePhysics(double deltaTime) {
    // Apply movement forces
    if (vsg::length(targetVelocity) > 0.01f || std::abs(targetAngularVelocity) > 0.01f) {
        const dReal* currentVel = dBodyGetLinearVel(bodyID);
        vsg::vec3 velError = targetVelocity - vsg::vec3(currentVel[0], currentVel[1], currentVel[2]);
        
        // Apply force to reach target velocity
        dBodyAddForce(bodyID, 
            velError.x * 50.0f,
            0,
            velError.z * 50.0f);
        
        // Apply torque for rotation
        dBodyAddTorque(bodyID, 0, targetAngularVelocity * 20.0f, 0);
    }
}

void Robot::updateGait(double time) {
    // Tripod gait pattern for hexapod
    const int gaitPattern[2][3] = {{0, 3, 4}, {1, 2, 5}};
    
    for (int group = 0; group < 2; ++group) {
        float phaseOffset = group * M_PI;
        float liftPhase = sin(time + phaseOffset);
        
        for (int i = 0; i < 3; ++i) {
            int legIndex = gaitPattern[group][i];
            auto& leg = legs[legIndex];
            
            // Calculate target foot position
            float stepX = strideLength * cos(time + phaseOffset);
            float stepY = (liftPhase > 0) ? stepHeight * liftPhase : 0;
            
            vsg::vec3 targetPos = leg.attachmentPoint + vsg::vec3(stepX, -1.2f + stepY, 0);
            
            // Apply inverse kinematics
            calculateInverseKinematics(legIndex, targetPos);
        }
    }
}

void Robot::calculateInverseKinematics(int legIndex, const vsg::vec3& targetPos) {
    auto& leg = legs[legIndex];
    
    // Simplified IK for demonstration
    const dReal* bodyPos = dBodyGetPosition(bodyID);
    vsg::vec3 bodyPosition(bodyPos[0], bodyPos[1], bodyPos[2]);
    vsg::vec3 relativeTarget = targetPos - bodyPosition;
    
    // Calculate joint angles (simplified)
    float distance = vsg::length(relativeTarget);
    float angle1 = atan2(relativeTarget.z, relativeTarget.x);
    float angle2 = asin(relativeTarget.y / distance);
    
    // Apply to first segment (hip)
    if (!leg.segments.empty() && leg.segments[0].joint) {
        // Apply motor forces to achieve target angles
        dJointAddTorques(leg.segments[0].joint, angle1 * 10.0f, angle2 * 10.0f, 0);
    }
}

void Robot::detectGround() {
    // Check contact for each leg
    for (int i = 0; i < NUM_LEGS; ++i) {
        legs[i].isGrounded = false;
        
        if (!legs[i].segments.empty()) {
            const auto& foot = legs[i].segments.back();
            
            // Simple ground detection using position
            const dReal* pos = dBodyGetPosition(foot.body);
            if (pos[1] < foot.length/2 + 0.1f) {
                legs[i].isGrounded = true;
            }
        }
    }
}

void Robot::stabilize() {
    // Get body orientation
    const dReal* q = dBodyGetQuaternion(bodyID);
    vsg::quat orientation(q[1], q[2], q[3], q[0]);
    
    // Calculate pitch and roll
    vsg::vec3 euler = eulerAnglesFromQuat(orientation);
    float pitch = euler.x;
    float roll = euler.z;
    
    // Apply corrective torques
    dBodyAddTorque(bodyID, -pitch * 50.0f, 0, -roll * 50.0f);
    
    // Adjust leg positions for balance
    int groundedCount = 0;
    for (const auto& leg : legs) {
        if (leg.isGrounded) groundedCount++;
    }
    
    // Ensure at least 3 legs are grounded for stability
    if (groundedCount < 3) {
        // Emergency stabilization - lower body
        dBodyAddForce(bodyID, 0, -20.0f, 0);
    }
}

void Robot::updateVisuals() {
    // Update body transform
    const dReal* pos = dBodyGetPosition(bodyID);
    const dReal* rot = dBodyGetRotation(bodyID);
    
    vsg::dmat4 matrix;
    matrix[0][0] = rot[0]; matrix[0][1] = rot[1]; matrix[0][2] = rot[2]; matrix[0][3] = 0;
    matrix[1][0] = rot[4]; matrix[1][1] = rot[5]; matrix[1][2] = rot[6]; matrix[1][3] = 0;
    matrix[2][0] = rot[8]; matrix[2][1] = rot[9]; matrix[2][2] = rot[10]; matrix[2][3] = 0;
    matrix[3][0] = pos[0]; matrix[3][1] = pos[1]; matrix[3][2] = pos[2]; matrix[3][3] = 1;
    
    bodyTransform->matrix = matrix;
    
    // Update leg transforms
    for (auto& leg : legs) {
        for (auto& segment : leg.segments) {
            const dReal* segPos = dBodyGetPosition(segment.body);
            const dReal* segRot = dBodyGetRotation(segment.body);
            
            vsg::dmat4 segMatrix;
            segMatrix[0][0] = segRot[0]; segMatrix[0][1] = segRot[1]; segMatrix[0][2] = segRot[2]; segMatrix[0][3] = 0;
            segMatrix[1][0] = segRot[4]; segMatrix[1][1] = segRot[5]; segMatrix[1][2] = segRot[6]; segMatrix[1][3] = 0;
            segMatrix[2][0] = segRot[8]; segMatrix[2][1] = segRot[9]; segMatrix[2][2] = segRot[10]; segMatrix[2][3] = 0;
            segMatrix[3][0] = segPos[0]; segMatrix[3][1] = segPos[1]; segMatrix[3][2] = segPos[2]; segMatrix[3][3] = 1;
            
            segment.transform->matrix = segMatrix;
        }
    }
}

void Robot::applyControl(const std::vector<float>& motorCommands) {
    // Apply motor commands to joints
    size_t commandIndex = 0;
    
    for (auto& leg : legs) {
        for (auto& segment : leg.segments) {
            if (segment.joint && commandIndex < motorCommands.size()) {
                // Apply torque based on command
                float torque = motorCommands[commandIndex] * 10.0f;
                dJointAddTorques(segment.joint, torque, 0, 0);
                commandIndex++;
            }
        }
    }
    
    // Update target velocity from commands
    if (motorCommands.size() >= 2) {
        targetVelocity.x = motorCommands[motorCommands.size() - 2] * moveSpeed;
        targetVelocity.z = motorCommands[motorCommands.size() - 1] * moveSpeed;
    }
}

void Robot::reset() {
    // Reset position and orientation
    dBodySetPosition(bodyID, 0, 2.0f, 0);
    dBodySetLinearVel(bodyID, 0, 0, 0);
    dBodySetAngularVel(bodyID, 0, 0, 0);
    
    dQuaternion q;
    dQSetIdentity(q);
    dBodySetQuaternion(bodyID, q);
    
    // Reset legs
    for (auto& leg : legs) {
        leg.currentAngle = 0;
        leg.targetAngle = 0;
        leg.isGrounded = false;
        
        for (auto& segment : leg.segments) {
            dBodySetLinearVel(segment.body, 0, 0, 0);
            dBodySetAngularVel(segment.body, 0, 0, 0);
        }
    }
    
    // Reset state
    gaitPhase = 0;
    energyConsumption = 0;
    targetVelocity = vsg::vec3(0, 0, 0);
    targetAngularVelocity = 0;
}

vsg::vec3 Robot::getPosition() const {
    const dReal* pos = dBodyGetPosition(bodyID);
    return vsg::vec3(pos[0], pos[1], pos[2]);
}

vsg::vec3 Robot::getVelocity() const {
    const dReal* vel = dBodyGetLinearVel(bodyID);
    return vsg::vec3(vel[0], vel[1], vel[2]);
}

vsg::quat Robot::getOrientation() const {
    const dReal* q = dBodyGetQuaternion(bodyID);
    return vsg::quat(q[1], q[2], q[3], q[0]);
}

std::vector<float> Robot::getSensorReadings() const {
    std::vector<float> readings;
    
    for (const auto& sensor : sensors) {
        readings.push_back(sensor.currentValue);
    }
    
    return readings;
}

bool Robot::isStable() const {
    // Check if at least 3 legs are grounded
    int groundedCount = 0;
    for (const auto& leg : legs) {
        if (leg.isGrounded) groundedCount++;
    }
    
    // Check body tilt
    vsg::quat orientation = getOrientation();
    vsg::vec3 euler = eulerAnglesFromQuat(orientation);
    
    return groundedCount >= 3 && 
           std::abs(euler.x) < 0.5f && 
           std::abs(euler.z) < 0.5f;
}

void Robot::setBodyColor(const vsg::vec4& color) {
    bodyColor = color;
    if (material) {
        material->diffuse = color;
    }
}

void Robot::addBodyDecorations() {
    // Add eyes
    auto builder = vsg::Builder::create();
    
    for (int i = -1; i <= 1; i += 2) {
        auto eyeTransform = vsg::MatrixTransform::create();
        eyeTransform->matrix = vsg::translate(0.9f, 0.15f, i * 0.3f);
        
        auto sphere = vsg::Sphere::create();
        sphere->radius = 0.08f;
        
        vsg::GeometryInfo geomInfo;
        geomInfo.sphere = sphere;
        geomInfo.color = vsg::vec4(1.0f, 0.0f, 0.0f, 1.0f);
        
        auto eye = builder->createSphere(geomInfo);
        eyeTransform->addChild(eye);
        bodyTransform->addChild(eyeTransform);
    }
    
    // Add antenna
    auto antennaTransform = vsg::MatrixTransform::create();
    antennaTransform->matrix = vsg::translate(1.0f, 0.3f, 0.0f);
    
    auto cylinder = vsg::Cylinder::create();
    cylinder->radius = 0.02f;
    cylinder->height = 0.3f;
    
    vsg::GeometryInfo antennaInfo;
    antennaInfo.cylinder = cylinder;
    antennaInfo.color = vsg::vec4(0.7f, 0.7f, 0.7f, 1.0f);
    
    auto antenna = builder->createCylinder(antennaInfo);
    antennaTransform->addChild(antenna);
    bodyTransform->addChild(antennaTransform);
}

vsg::vec3 Robot::eulerAnglesFromQuat(const vsg::quat& q) const {
    vsg::vec3 euler;
    
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