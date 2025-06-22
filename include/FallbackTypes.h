#pragma once
#include <cmath>
#include <vector>

#ifdef USE_OPENGL_FALLBACK

#ifndef SIMPLE_MATH_TYPES_DEFINED
#define SIMPLE_MATH_TYPES_DEFINED

struct vec3 {
    float x, y, z;
    constexpr vec3(float x_ = 0, float y_ = 0, float z_ = 0) : x(x_), y(y_), z(z_) {}
    constexpr vec3 operator+(const vec3& other) const { return {x + other.x, y + other.y, z + other.z}; }
    constexpr vec3 operator-(const vec3& other) const { return {x - other.x, y - other.y, z - other.z}; }
    constexpr vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    vec3 operator/(float s) const { return {x / s, y / s, z / s}; }
    float length() const { return std::sqrt(x * x + y * y + z * z); }
    vec3 normalize() const { float l = length(); return l > 0 ? (*this) * (1.0f / l) : vec3(); }
};

// Length function for compatibility
inline float length(const vec3& v) { return v.length(); }

struct vec2 {
    float x, y;
    constexpr vec2(float x_ = 0, float y_ = 0) : x(x_), y(y_) {}
    constexpr vec2 operator+(const vec2& other) const { return {x + other.x, y + other.y}; }
    constexpr vec2 operator-(const vec2& other) const { return {x - other.x, y - other.y}; }
    constexpr vec2 operator*(float s) const { return {x * s, y * s}; }
    vec2 operator/(float s) const { return {x / s, y / s}; }
    float length() const { return std::sqrt(x * x + y * y); }
};

inline float length(const vec2& v) { return v.length(); }

// Dot product functions
inline float dot(const vec2& a, const vec2& b) { return a.x * b.x + a.y * b.y; }
inline float dot(const vec3& a, const vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

// Cross product - must be defined before quat
inline vec3 cross(const vec3& a, const vec3& b) {
    return vec3(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    );
}

struct vec4 {
    float x, y, z, w;
    constexpr vec4(float x_ = 0, float y_ = 0, float z_ = 0, float w_ = 1) : x(x_), y(y_), z(z_), w(w_) {}
};

struct quat {
    float x, y, z, w;
    constexpr quat(float x_ = 0, float y_ = 0, float z_ = 0, float w_ = 1) : x(x_), y(y_), z(z_), w(w_) {}
    
    // Quaternion * Vector rotation
    vec3 operator*(const vec3& v) const {
        // Simplified quaternion rotation
        vec3 qv(x, y, z);
        vec3 t = cross(qv, v) * 2.0f;
        return v + t * w + cross(qv, t);
    }
};

// Minimal smart-pointer-like helper to keep existing fallback code compiling.
// It does NOT manage ownership – it is only meant for compile-time compatibility.
template<typename T>
struct ref_ptr {
    T* ptr = nullptr;
    ref_ptr() = default;
    ref_ptr(std::nullptr_t) : ptr(nullptr) {}  // Allow nullptr initialization
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

class MatrixTransform {
public:
    struct {
        double m[4][4];
        double* operator[](int i) { return m[i]; }
        const double* operator[](int i) const { return m[i]; }
    } matrix;
};

class PhongMaterialValue {};

// Helper functions
inline vec3 normalize(const vec3& v) { return v.normalize(); }

// For compatibility with VSG code
namespace vsg {
    using vec2 = ::vec2;
    using vec3 = ::vec3;
    using vec4 = ::vec4;
    using quat = ::quat;
    
    struct dmat4 {
        double m[4][4];
        double* operator[](int i) { return m[i]; }
        const double* operator[](int i) const { return m[i]; }
    };
    
    class Group {
    public:
        static ref_ptr<::Group> create() { return ref_ptr<::Group>(new ::Group()); }
    };
    
    inline vec3 normalize(const vec3& v) { return v.normalize(); }
    inline vec3 cross(const vec3& a, const vec3& b) { return ::cross(a, b); }
    inline float length(const vec3& v) { return v.length(); }
    inline float length(const vec2& v) { return v.length(); }
    inline float dot(const vec2& a, const vec2& b) { return ::dot(a, b); }
    inline float dot(const vec3& a, const vec3& b) { return ::dot(a, b); }
}

#endif   // SIMPLE_MATH_TYPES_DEFINED

#endif   // USE_OPENGL_FALLBACK