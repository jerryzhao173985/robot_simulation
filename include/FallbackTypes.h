#pragma once
#include <cmath>
#include <vector>

#ifndef SIMPLE_MATH_TYPES_DEFINED
#define SIMPLE_MATH_TYPES_DEFINED

struct vec3 {
    float x, y, z;
    constexpr vec3(float x_ = 0, float y_ = 0, float z_ = 0) : x(x_), y(y_), z(z_) {}
    constexpr vec3 operator+(const vec3& other) const { return {x + other.x, y + other.y, z + other.z}; }
    constexpr vec3 operator-(const vec3& other) const { return {x - other.x, y - other.y, z - other.z}; }
    constexpr vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    float length() const { return std::sqrt(x * x + y * y + z * z); }
    vec3 normalize() const { float l = length(); return l > 0 ? (*this) * (1.0f / l) : vec3(); }
};

struct vec2 {
    float x, y;
    constexpr vec2(float x_ = 0, float y_ = 0) : x(x_), y(y_) {}
    constexpr vec2 operator+(const vec2& other) const { return {x + other.x, y + other.y}; }
    constexpr vec2 operator-(const vec2& other) const { return {x - other.x, y - other.y}; }
    constexpr vec2 operator*(float s) const { return {x * s, y * s}; }
    float length() const { return std::sqrt(x * x + y * y); }
    vec2 normalize() const { float l = length(); return l > 0 ? (*this) * (1.0f / l) : vec2(); }
};

struct vec4 {
    float x, y, z, w;
    constexpr vec4(float x_ = 0, float y_ = 0, float z_ = 0, float w_ = 1) : x(x_), y(y_), z(z_), w(w_) {}
};

struct quat {
    float x, y, z, w;
    constexpr quat(float x_ = 0, float y_ = 0, float z_ = 0, float w_ = 1) : x(x_), y(y_), z(z_), w(w_) {}
};

// Minimal smart-pointer-like helper to keep existing fallback code compiling.
// It does NOT manage ownership – it is only meant for compile-time compatibility.
template<typename T>
struct ref_ptr {
    T* ptr = nullptr;
    ref_ptr() = default;
    explicit ref_ptr(T* p) : ptr(p) {}
    T* operator->() { return ptr; }
    const T* operator->() const { return ptr; }
    T& operator*() { return *ptr; }
    const T& operator*() const { return *ptr; }
    explicit operator bool() const { return ptr != nullptr; }
};

class Group {
public:
    std::vector<void*> children;
    void addChild(void* c) { children.push_back(c); }
};

class MatrixTransform {};
class PhongMaterialValue {};

//  add at the bottom (still inside SIMPLE_MATH_TYPES_DEFINED)
namespace vsg          // lets old code that says “vsg::vec3” keep compiling
{
    using vec3 = ::vec3;
    using vec2 = ::vec2;
    using vec4 = ::vec4;
    using quat = ::quat;

    inline float length(const vec3& v) { return v.length(); }
    inline vec3 normalize(const vec3& v) { return v.normalize(); }
    inline float dot(const vec3& a, const vec3& b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
    inline vec3 cross(const vec3& a, const vec3& b)
    {
        return vec3(a.y*b.z - a.z*b.y,
                    a.z*b.x - a.x*b.z,
                    a.x*b.y - a.y*b.x);
    }

    inline float length(const vec2& v) { return v.length(); }
    inline vec2 normalize(const vec2& v) { return v.normalize(); }
    inline float dot(const vec2& a, const vec2& b) { return a.x*b.x + a.y*b.y; }
}
#endif   // SIMPLE_MATH_TYPES_DEFINED
