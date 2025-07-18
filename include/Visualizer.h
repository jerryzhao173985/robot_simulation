#pragma once

#include "FallbackTypes.h"

#ifdef USE_OPENGL_FALLBACK
    #include <GLFW/glfw3.h>
    #ifdef __APPLE__
        #include <OpenGL/gl.h>
        #include <OpenGL/glu.h>
    #else
        #include <GL/gl.h>
        #include <GL/glu.h>
    #endif
    
    using vsg_vec3 = vec3;
    using vsg_vec4 = vec4;
    using vsg_quat = quat;
#else
#include <vsg/all.h>
#include <vsgXchange/all.h>
#include <vsg/text/Text.h>
#include <vsg/text/Font.h>
#include <vsg/text/StandardLayout.h>
#include <vsg/text/GpuLayoutTechnique.h>
    using vsg_vec3 = vsg::vec3;
    using vsg_vec4 = vsg::vec4;  
    using vsg_quat = vsg::quat;
#endif

#include <memory>
#include <map>
#include <string>

class Visualizer {
public:
    struct Light {
        vsg_vec3 position;
        vsg_vec3 color;
        float intensity;
#ifndef USE_OPENGL_FALLBACK
        vsg::ref_ptr<vsg::DirectionalLight> directional;
        vsg::ref_ptr<vsg::PointLight> point;
        vsg::ref_ptr<vsg::SpotLight> spot;
#endif
    };

    enum class CameraMode {
        FOLLOW = 0,
        FREE,
        ORBIT,
        TOP,
        FRONT,
        SIDE
    };

    struct Camera {
        vsg_vec3 position;
        vsg_vec3 target;
        vsg_vec3 up;
        float fov;
        float nearPlane;
        float farPlane;
        bool followRobot;
        float followDistance;
        float followHeight;
        float orbitAngle = 0.0f;
        float orbitSpeed = 0.5f;
    };

    Visualizer(uint32_t width = 1920, uint32_t height = 1080);
    ~Visualizer();

    bool initialize();
    void render();
    bool shouldClose() const;
    void close(); // Request window close
    
#ifndef USE_OPENGL_FALLBACK
    // Add event handler to viewer
    void addEventHandler(vsg::ref_ptr<vsg::Visitor> handler);
#endif
    
    // Scene management
#ifdef USE_OPENGL_FALLBACK
    void setSceneRoot(ref_ptr<Group> root) { sceneRoot = root; }
    ref_ptr<Group> getSceneRoot() const { return sceneRoot; }
#else
    void setSceneRoot(vsg::ref_ptr<vsg::Group> root) { sceneRoot = root; }
    vsg::ref_ptr<vsg::Group> getSceneRoot() const { return sceneRoot; }
#endif
    
    // Camera control
    void setCameraPosition(const vsg_vec3& position);
    void setCameraTarget(const vsg_vec3& target);
    void enableCameraFollow(bool enable, const vsg_vec3& targetPos = vsg_vec3());
    void setCameraMode(CameraMode mode);
    CameraMode getCameraMode() const { return currentCameraMode; }
    void adjustCameraDistance(float delta);
    
    // Lighting
    void addDirectionalLight(const vsg_vec3& direction, const vsg_vec3& color, float intensity);
    void addPointLight(const vsg_vec3& position, const vsg_vec3& color, float intensity, float range);
    void addSpotLight(const vsg_vec3& position, const vsg_vec3& direction, const vsg_vec3& color, 
                      float intensity, float angle);
    void enableShadows(bool enable) { shadowsEnabled = enable; }
    void setAmbientLight(const vsg_vec3& color) { ambientColor = color; }
    
    // Visual effects
    void enablePostProcessing(bool enable) { postProcessingEnabled = enable; }
    void enableBloom(bool enable) { bloomEnabled = enable; }
    void enableSSAO(bool enable) { ssaoEnabled = enable; }
    void enableMotionBlur(bool enable) { motionBlurEnabled = enable; }
    void setExposure(float exposure) { this->exposure = exposure; }
    
    // Environment
    void setSkybox(const std::string& path);
    void setFog(const vsg_vec3& color, float density);
    void enableReflections(bool enable) { reflectionsEnabled = enable; }
    
    // Debug visualization
    void drawLine(const vsg_vec3& start, const vsg_vec3& end, const vsg_vec4& color);
    void drawSphere(const vsg_vec3& center, float radius, const vsg_vec4& color);
    void drawBox(const vsg_vec3& center, const vsg_vec3& size, const vsg_quat& rotation, const vsg_vec4& color);
    void clearDebugGeometry();
    
    // Robot synchronization
    void updateRobotTransform(const vsg_vec3& position, const vsg_quat& orientation);
#ifndef USE_OPENGL_FALLBACK
    vsg::ref_ptr<vsg::MatrixTransform> getRobotTransform() const { return robotTransform; }
    void compileScene() { if (viewer) viewer->compile(); }
#endif
    
    // UI overlay
    void showStats(bool show) { showStatistics = show; }
    void showControls(bool show) { showControlsOverlay = show; }
    
    // Advanced graphics features
    void enableRayTracing(bool enable) { rayTracingEnabled = enable; }
    void setMSAASamples(uint32_t samples) { msaaSamples = samples; }
    void enableVSync(bool enable) { vsyncEnabled = enable; }

    void addSkybox(const std::string& skyboxPath);
    void setEnvironmentLighting(float intensity, const vsg_vec3& direction);
    
    // Helper functions
    void createAxisIndicator();
    void createLighting();
    void addBox(const vsg_vec3& position, const vsg_vec3& size, const vsg_vec4& color);

    // Input handling
    void handleKeyPress(int key);
    void handleKeyRelease(int key);
    void setManualControlEnabled(bool enabled) { manualControlEnabled = enabled; }
    bool isManualControlEnabled() const { return manualControlEnabled; }
    vsg_vec3 getManualControlVelocity() const { return manualControlVelocity; }
    
    // Text rendering
    void createTextOverlay();
    void updateStatsText(float fps, float frameTime, const vsg_vec3& robotPos, 
                        const vsg_vec3& robotVel, bool stable, const std::string& controlMode,
                        const vsg_vec3& moveCommand = vsg_vec3(0.0f, 0.0f, 0.0f), 
                        float rotCommand = 0.0f,
                        int footContacts = 0,
                        const vsg_vec3& angularVelocity = vsg_vec3(0.0f, 0.0f, 0.0f),
                        float avgJointVel = 0.0f);
    void updateControlsText(bool visible);
    void setShadowsEnabled(bool enable) { shadowsEnabled = enable; }
    void setTextVisible(bool stats, bool controls);
    
    // Camera controls  
    void setCameraMode(const std::string& mode);
    void cycleCamera();
    std::string getCameraModeString() const;

#ifndef USE_OPENGL_FALLBACK
    // Modern VSG scene creation methods
    vsg::ref_ptr<vsg::Group> createScene(vsg::ref_ptr<vsg::Options> options);
    void setupModernLighting(vsg::ref_ptr<vsg::Group> scene);
    
    // Input handler removed - now handled externally
#endif

private:
    void setupPostProcessing();
    void updateCamera();
    void updateStats();
    
#ifdef USE_OPENGL_FALLBACK
    // OpenGL/GLFW backend
    GLFWwindow* window = nullptr;
    ref_ptr<Group> sceneRoot;
    ref_ptr<Group> scene;
    ref_ptr<Group> axisIndicator;
    void renderOpenGL();
    void drawCube(const vsg_vec3& position, const vsg_vec3& size, const vsg_vec4& color);
    void drawSphereOpenGL(const vsg_vec3& center, float radius, const vsg_vec4& color);

    // free-camera spherical coords for interactive view (fallback)
    float freeAzimuth = -3.14f/4.0f;
    float freeElevation = 0.2f;
    float freeDistance = 10.0f;
    bool mouseDragging = false;
    double dragLastX = 0.0;
    double dragLastY = 0.0;
#else
    // VSG backend
    vsg::ref_ptr<vsg::Window> window;
    vsg::ref_ptr<vsg::Viewer> viewer;
    vsg::ref_ptr<vsg::Group> sceneRoot;
    vsg::ref_ptr<vsg::Group> scene;
    vsg::ref_ptr<vsg::Group> axisIndicator;
    vsg::ref_ptr<vsg::CommandGraph> commandGraph;
    vsg::ref_ptr<vsg::RenderGraph> renderGraph;

    // Robot visual transform - synced with physics
    vsg::ref_ptr<vsg::MatrixTransform> robotTransform;
    
    vsg::ref_ptr<vsg::Camera> vsgCamera;
    vsg::ref_ptr<vsg::LookAt> lookAt;
    vsg::ref_ptr<vsg::Perspective> perspective;
#endif
    
    // Camera
    Camera camera;
    CameraMode currentCameraMode = CameraMode::FOLLOW;
    vsg_vec3 robotPosition;
    vsg_quat robotOrientation;
    
    // Lighting
    std::vector<Light> lights;
    vsg_vec3 ambientColor = vsg_vec3(0.1f, 0.1f, 0.15f);
    bool shadowsEnabled = true;
    
    // Visual effects
    bool postProcessingEnabled = true;
    bool bloomEnabled = true;
    bool ssaoEnabled = true;
    bool motionBlurEnabled = false;
    bool reflectionsEnabled = true;
    bool rayTracingEnabled = false;
    float exposure = 1.0f;
    
    // Window properties
    uint32_t windowWidth;
    uint32_t windowHeight;
    uint32_t msaaSamples = 1;  // Reduced from 4 for performance
    bool vsyncEnabled = false;  // Disabled for performance testing
    
    // UI
    bool showStatistics = true;
    bool showControlsOverlay = true;
    
    // Performance tracking
    double frameTime = 0.0;
    double renderTime = 0.0;
    uint32_t frameCount = 0;
    std::chrono::high_resolution_clock::time_point lastTime;
    
    // Input handling
    bool manualControlEnabled = false;
    vsg_vec3 manualControlVelocity;
    std::map<int, bool> keyStates;
    
    // Text rendering
#ifndef USE_OPENGL_FALLBACK
    vsg::ref_ptr<vsg::Group> textGroup;
    vsg::ref_ptr<vsg::Text> statsText;
    vsg::ref_ptr<vsg::Text> controlsText;
    vsg::ref_ptr<vsg::Font> font;
    vsg::ref_ptr<vsg::Options> textOptions;
    vsg::ref_ptr<vsg::MatrixTransform> statsTransform;
    vsg::ref_ptr<vsg::MatrixTransform> controlsTransform;
    bool statsVisible = true;
    bool controlsVisible = false;
#endif
};