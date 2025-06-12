#pragma once

#include <vsg/all.h>
#include <vsgXchange/all.h>
#include <memory>

class Visualizer {
public:
    struct Light {
        vsg::vec3 position;
        vsg::vec3 color;
        float intensity;
        vsg::ref_ptr<vsg::DirectionalLight> directional;
        vsg::ref_ptr<vsg::PointLight> point;
        vsg::ref_ptr<vsg::SpotLight> spot;
    };

    struct Camera {
        vsg::vec3 position;
        vsg::vec3 target;
        vsg::vec3 up;
        float fov;
        float nearPlane;
        float farPlane;
        bool followRobot;
        float followDistance;
        float followHeight;
    };

    Visualizer(uint32_t width = 1920, uint32_t height = 1080);
    ~Visualizer();

    bool initialize();
    void render();
    bool shouldClose() const;
    
    // Scene management
    void setSceneRoot(vsg::ref_ptr<vsg::Group> root) { sceneRoot = root; }
    vsg::ref_ptr<vsg::Group> getSceneRoot() const { return sceneRoot; }
    
    // Camera control
    void setCameraPosition(const vsg::vec3& position);
    void setCameraTarget(const vsg::vec3& target);
    void enableCameraFollow(bool enable, const vsg::vec3& targetPos = vsg::vec3(0.0f));
    void setCameraMode(int mode) { cameraMode = mode; }
    
    // Lighting
    void addDirectionalLight(const vsg::vec3& direction, const vsg::vec3& color, float intensity);
    void addPointLight(const vsg::vec3& position, const vsg::vec3& color, float intensity, float range);
    void addSpotLight(const vsg::vec3& position, const vsg::vec3& direction, const vsg::vec3& color, 
                      float intensity, float angle);
    void enableShadows(bool enable) { shadowsEnabled = enable; }
    void setAmbientLight(const vsg::vec3& color) { ambientColor = color; }
    
    // Visual effects
    void enablePostProcessing(bool enable) { postProcessingEnabled = enable; }
    void enableBloom(bool enable) { bloomEnabled = enable; }
    void enableSSAO(bool enable) { ssaoEnabled = enable; }
    void enableMotionBlur(bool enable) { motionBlurEnabled = enable; }
    void setExposure(float exposure) { this->exposure = exposure; }
    
    // Environment
    void setSkybox(const std::string& path);
    void setFog(const vsg::vec3& color, float density);
    void enableReflections(bool enable) { reflectionsEnabled = enable; }
    
    // Debug visualization
    void drawLine(const vsg::vec3& start, const vsg::vec3& end, const vsg::vec4& color);
    void drawSphere(const vsg::vec3& center, float radius, const vsg::vec4& color);
    void drawBox(const vsg::vec3& center, const vsg::vec3& size, const vsg::quat& rotation, const vsg::vec4& color);
    void clearDebugGeometry();
    
    // UI overlay
    void showStats(bool show) { showStatistics = show; }
    void showControls(bool show) { showControlsOverlay = show; }
    
    // Advanced graphics features
    void enableRayTracing(bool enable) { rayTracingEnabled = enable; }
    void setMSAASamples(uint32_t samples) { msaaSamples = samples; }
    void enableVSync(bool enable) { vsyncEnabled = enable; }

private:
    void createWindow();
    void createViewer();
    void setupRenderGraph();
    void setupLighting();
    void setupPostProcessing();
    void updateCamera();
    void updateStats();
    
    // Window and viewer
    vsg::ref_ptr<vsg::Window> window;
    vsg::ref_ptr<vsg::Viewer> viewer;
    vsg::ref_ptr<vsg::Group> sceneRoot;
    vsg::ref_ptr<vsg::CommandGraph> commandGraph;
    vsg::ref_ptr<vsg::RenderGraph> renderGraph;
    
    // Camera
    Camera camera;
    vsg::ref_ptr<vsg::Camera> vsgCamera;
    vsg::ref_ptr<vsg::LookAt> lookAt;
    vsg::ref_ptr<vsg::Perspective> perspective;
    int cameraMode = 0; // 0: free, 1: follow, 2: orbit
    
    // Lighting
    std::vector<Light> lights;
    vsg::vec3 ambientColor = vsg::vec3(0.1f, 0.1f, 0.15f);
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
    uint32_t msaaSamples = 4;
    bool vsyncEnabled = true;
    
    // Debug visualization
    vsg::ref_ptr<vsg::Group> debugGroup;
    vsg::ref_ptr<vsg::StateGroup> debugStateGroup;
    
    // UI
    bool showStatistics = true;
    bool showControlsOverlay = true;
    
    // Performance tracking
    double frameTime = 0.0;
    double renderTime = 0.0;
    uint32_t frameCount = 0;
};