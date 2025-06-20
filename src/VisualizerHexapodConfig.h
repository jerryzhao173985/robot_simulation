// Hexapod Visual Configuration
// Natural hexapod stance parameters

#ifndef VISUALIZER_HEXAPOD_CONFIG_H
#define VISUALIZER_HEXAPOD_CONFIG_H

namespace HexapodVisualConfig {
    // Leg spread configuration
    const double COXA_SPREAD_ANGLE = 70.0;     // degrees - wide spread for stability
    const double COXA_ELEVATION = 5.0;         // degrees - slight upward angle
    
    // Segment angles for natural stance
    const double FEMUR_DOWN_ANGLE = 20.0;      // degrees - mostly horizontal
    const double TIBIA_DOWN_ANGLE = 65.0;      // degrees - steep to reach ground
    
    // Leg-specific variations (degrees)
    const double FRONT_LEG_FORWARD = 15.0;     // Front legs angle forward
    const double REAR_LEG_BACKWARD = -15.0;    // Rear legs angle backward
    const double MIDDLE_LEG_OFFSET = 0.0;      // Middle legs straight out
}

#endif // VISUALIZER_HEXAPOD_CONFIG_H