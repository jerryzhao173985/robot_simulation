#pragma once

#include "noisegenerator.h"
#include <memory>
#include <vector>

class NoiseManager {
public:
    // Singleton pattern
    static NoiseManager& getInstance() {
        static NoiseManager instance;
        return instance;
    }
    
    // Initialize noise generators
    void init(unsigned int numSensors, unsigned int numActuators);
    
    // Set noise level (0.0 to 1.0)
    void setNoiseLevel(float level);
    float getNoiseLevel() const { return noiseLevel; }
    
    // Increase/decrease noise level
    void increaseNoise(float delta = 0.1f);
    void decreaseNoise(float delta = 0.1f);
    
    // Apply noise to sensor readings
    void applySensorNoise(double* values, unsigned int count);
    
    // Apply noise to actuator commands
    void applyActuatorNoise(double* commands, unsigned int count);
    
    // Apply noise to specific sensor types with appropriate scaling
    float applyIMUNoise(float value, bool isAngularVelocity = true);
    float applyJointPositionNoise(float value);
    float applyJointVelocityNoise(float value);
    float applyContactNoise(float value, bool isForce = false);
    
    // Apply noise to actuator commands with appropriate scaling
    float applyMotorNoise(float command);
    
    // Apply sinusoidal noise for visual testing
    float applySinusoidalNoise(float value, float frequency, float amplitude, int channelIndex = 0);
    
    // Update simulation time
    void updateTime(double deltaTime) { simulationTime += deltaTime; }
    
    // Check if noise compensation should be active
    bool shouldCompensate() const { return noiseLevel > 0.2f; }
    
    // Destructor
    ~NoiseManager();
    
private:
    NoiseManager();
    NoiseManager(const NoiseManager&) = delete;
    NoiseManager& operator=(const NoiseManager&) = delete;
    
    std::unique_ptr<WhiteUniformNoise> sensorNoiseGen;
    std::unique_ptr<WhiteUniformNoise> actuatorNoiseGen;
    std::unique_ptr<RandGen> randGen;
    
    float noiseLevel;
    double simulationTime;  // Track time for sinusoidal noise
    
    // Noise scaling factors for different sensor types
    static constexpr float IMU_ANGULAR_VEL_NOISE_SCALE = 0.1f;     // ±0.1 rad/s
    static constexpr float IMU_ORIENTATION_NOISE_SCALE = 0.02f;    // ±0.02 rad
    static constexpr float JOINT_POSITION_NOISE_SCALE = 0.05f;     // ±0.05 rad
    static constexpr float JOINT_VELOCITY_NOISE_SCALE = 0.1f;      // ±0.1 rad/s
    static constexpr float CONTACT_FORCE_NOISE_SCALE = 0.1f;       // ±10% variation
    static constexpr float CONTACT_BINARY_NOISE_THRESHOLD = 0.9f;  // 10% false readings
    
    // Noise scaling for actuators
    static constexpr float MOTOR_COMMAND_NOISE_SCALE = 0.05f;      // ±5% of command
};