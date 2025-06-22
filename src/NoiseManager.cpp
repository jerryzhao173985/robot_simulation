#include "NoiseManager.h"
#include <algorithm>
#include <ctime>
#include <iostream>
#include <cmath>

NoiseManager::NoiseManager() : noiseLevel(0.0f), simulationTime(0.0) {
    // Initialize random generator
    randGen = std::make_unique<RandGen>();
    randGen->init(static_cast<long>(std::time(nullptr)));
}

NoiseManager::~NoiseManager() {
}

void NoiseManager::init(unsigned int numSensors, unsigned int numActuators) {
    // Create noise generators
    sensorNoiseGen = std::make_unique<WhiteUniformNoise>();
    sensorNoiseGen->init(numSensors, randGen.get());
    
    actuatorNoiseGen = std::make_unique<WhiteUniformNoise>();
    actuatorNoiseGen->init(numActuators, randGen.get());
    
    std::cout << "[NoiseManager] Initialized with " << numSensors 
              << " sensor channels and " << numActuators << " actuator channels" << std::endl;
}

void NoiseManager::setNoiseLevel(float level) {
    noiseLevel = std::clamp(level, 0.0f, 1.0f);
}

void NoiseManager::increaseNoise(float delta) {
    setNoiseLevel(noiseLevel + delta);
    std::cout << "[NoiseManager] Noise level increased to: " << noiseLevel << std::endl;
}

void NoiseManager::decreaseNoise(float delta) {
    setNoiseLevel(noiseLevel - delta);
    std::cout << "[NoiseManager] Noise level decreased to: " << noiseLevel << std::endl;
}

void NoiseManager::applySensorNoise(double* values, unsigned int count) {
    if (noiseLevel > 0.0f && sensorNoiseGen && count <= sensorNoiseGen->getDimension()) {
        sensorNoiseGen->add(values, noiseLevel);
    }
}

void NoiseManager::applyActuatorNoise(double* commands, unsigned int count) {
    if (noiseLevel > 0.0f && actuatorNoiseGen && count <= actuatorNoiseGen->getDimension()) {
        actuatorNoiseGen->add(commands, noiseLevel);
    }
}

float NoiseManager::applyIMUNoise(float value, bool isAngularVelocity) {
    if (noiseLevel > 0.0f && sensorNoiseGen) {
        float scale = isAngularVelocity ? IMU_ANGULAR_VEL_NOISE_SCALE : IMU_ORIENTATION_NOISE_SCALE;
        return value + sensorNoiseGen->generate() * scale * noiseLevel;
    }
    return value;
}

float NoiseManager::applyJointPositionNoise(float value) {
    if (noiseLevel > 0.0f && sensorNoiseGen) {
        return value + sensorNoiseGen->generate() * JOINT_POSITION_NOISE_SCALE * noiseLevel;
    }
    return value;
}

float NoiseManager::applyJointVelocityNoise(float value) {
    if (noiseLevel > 0.0f && sensorNoiseGen) {
        return value + sensorNoiseGen->generate() * JOINT_VELOCITY_NOISE_SCALE * noiseLevel;
    }
    return value;
}

float NoiseManager::applyContactNoise(float value, bool isForce) {
    if (noiseLevel > 0.0f && sensorNoiseGen) {
        if (isForce) {
            // Add proportional noise to force readings
            return value * (1.0f + sensorNoiseGen->generate() * CONTACT_FORCE_NOISE_SCALE * noiseLevel);
        } else {
            // For binary contact detection, occasionally flip the state
            float noise = sensorNoiseGen->generate();
            if (std::abs(noise) > (1.0f - CONTACT_BINARY_NOISE_THRESHOLD * (1.0f - noiseLevel))) {
                // Flip the contact state
                return value > 0.5f ? 0.0f : 1.0f;
            }
        }
    }
    return value;
}

float NoiseManager::applyMotorNoise(float command) {
    if (noiseLevel > 0.0f && actuatorNoiseGen) {
        // Add proportional noise to motor commands
        float noise = actuatorNoiseGen->generate() * MOTOR_COMMAND_NOISE_SCALE * noiseLevel;
        
        // Also simulate dead zone at low commands
        float noisyCommand = command * (1.0f + noise);
        if (std::abs(noisyCommand) < 0.05f * noiseLevel) {
            return 0.0f; // Dead zone
        }
        
        return noisyCommand;
    }
    return command;
}

float NoiseManager::applySinusoidalNoise(float value, float frequency, float amplitude, int channelIndex) {
    if (noiseLevel <= 0.0f) {
        return value;
    }
    
    // Use different phase offsets for different channels to create more complex motion
    float phaseOffset = channelIndex * 0.7f; // Arbitrary phase offset per channel
    
    // Apply sinusoidal noise with given frequency and amplitude
    float noise = amplitude * noiseLevel * std::sin(2.0f * M_PI * frequency * simulationTime + phaseOffset);
    
    // Also add some random noise for realism
    if (sensorNoiseGen) {
        noise += sensorNoiseGen->generate() * amplitude * noiseLevel * 0.2f; // 20% random component
    }
    
    return value + noise;
}