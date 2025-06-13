#pragma once

#ifdef USE_OPENGL_FALLBACK
// vec3 is defined in each header when USE_OPENGL_FALLBACK is enabled.
inline vec3 makePosition(float x, float zAsUp, float yForward)
{
    return vec3(x, yForward, zAsUp);
}
#else
#include <vsg/all.h>

// Helper to construct a position vector in a Z-up coordinate system.
// x: horizontal X-axis, zAsUp: height along Z-axis (up), yForward: forward along Y-axis.
inline vsg::vec3 makePosition(float x, float zAsUp, float yForward)
{
    return vsg::vec3(x, yForward, zAsUp);
}
#endif