#include "Visualizer.h"
#include <iostream>
#include <vsg/io/Options.h>

Visualizer::Visualizer(uint32_t width, uint32_t height) 
    : windowWidth(width), windowHeight(height) {
    
    // Initialize camera
    camera.position = vsg::vec3(10.0f, 10.0f, 10.0f);
    camera.target = vsg::vec3(0.0f, 0.0f, 0.0f);
    camera.up = vsg::vec3(0.0f, 1.0f, 0.0f);
    camera.fov = 60.0f;
    camera.nearPlane = 0.1f;
    camera.farPlane = 1000.0f;
    camera.followRobot = false;
    camera.followDistance = 10.0f;
    camera.followHeight = 5.0f;
}

Visualizer::~Visualizer() {
    // Cleanup handled by smart pointers
}

bool Visualizer::initialize() {
    try {
        // Create window
        createWindow();
        
        // Create viewer
        createViewer();
        
        // Setup render graph
        setupRenderGraph();
        
        // Setup lighting
        setupLighting();
        
        // Compile
        viewer->compile();
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Failed to initialize visualizer: " << e.what() << std::endl;
        return false;
    }
}

void Visualizer::createWindow() {
    // Window traits
    auto traits = vsg::WindowTraits::create();
    traits->width = windowWidth;
    traits->height = windowHeight;
    traits->windowTitle = "VSG ODE Robot Simulation";
    traits->fullscreen = false;
    traits->samples = msaaSamples;
    traits->swapchainPreferences.presentMode = vsyncEnabled ? VK_PRESENT_MODE_FIFO_KHR : VK_PRESENT_MODE_IMMEDIATE_KHR;
    
    // Create window
    window = vsg::Window::create(traits);
    if (!window) {
        throw std::runtime_error("Failed to create window");
    }
}

void Visualizer::createViewer() {
    viewer = vsg::Viewer::create();
    viewer->addWindow(window);
    
    // Create scene root if not set
    if (!sceneRoot) {
        sceneRoot = vsg::Group::create();
    }
    
    // Add debug group
    debugGroup = vsg::Group::create();
    sceneRoot->addChild(debugGroup);
}

void Visualizer::setupRenderGraph() {
    // Create command graph
    commandGraph = vsg::CommandGraph::create(window);
    
    // Create render graph
    renderGraph = vsg::RenderGraph::create(window);
    
    // Setup camera
    lookAt = vsg::LookAt::create(
        camera.position,
        camera.target,
        camera.up
    );
    
    perspective = vsg::Perspective::create(
        camera.fov,
        static_cast<double>(windowWidth) / static_cast<double>(windowHeight),
        camera.nearPlane,
        camera.farPlane
    );
    
    vsgCamera = vsg::Camera::create(perspective, lookAt, vsg::ViewportState::create(window->extent2D()));
    
    // Add camera to render graph
    renderGraph->addChild(vsgCamera);
    
    // Add scene to render graph
    renderGraph->addChild(sceneRoot);
    
    // Add render graph to command graph
    commandGraph->addChild(renderGraph);
    
    // Add event handlers
    viewer->addEventHandler(vsg::CloseHandler::create(viewer));
    viewer->addEventHandler(vsg::Trackball::create(vsgCamera));
}

void Visualizer::setupLighting() {
    // Ambient light is set through the material properties
    
    // The actual lights are added through the public interface
}

void Visualizer::render() {
    // Update camera if following
    updateCamera();
    
    // Update stats
    updateStats();
    
    // Frame
    if (viewer->advanceToNextFrame()) {
        viewer->handleEvents();
        viewer->update();
        viewer->recordAndSubmit();
        viewer->present();
    }
}

bool Visualizer::shouldClose() const {
    return viewer->compileManager->active() || !viewer->active();
}

void Visualizer::setCameraPosition(const vsg::vec3& position) {
    camera.position = position;
    if (lookAt) {
        lookAt->eye = position;
    }
}

void Visualizer::setCameraTarget(const vsg::vec3& target) {
    camera.target = target;
    if (lookAt) {
        lookAt->center = target;
    }
}

void Visualizer::enableCameraFollow(bool enable, const vsg::vec3& targetPos) {
    camera.followRobot = enable;
    if (enable) {
        camera.target = targetPos;
    }
}

void Visualizer::updateCamera() {
    if (camera.followRobot && cameraMode == 1) {
        // Calculate follow position
        vsg::vec3 offset(
            -camera.followDistance * cos(frameTime * 0.1f),
            camera.followHeight,
            -camera.followDistance * sin(frameTime * 0.1f)
        );
        
        camera.position = camera.target + offset;
        
        if (lookAt) {
            lookAt->eye = camera.position;
            lookAt->center = camera.target;
        }
    } else if (cameraMode == 2) {
        // Orbit mode
        float angle = frameTime * 0.2f;
        float radius = vsg::length(camera.position - camera.target);
        
        camera.position = camera.target + vsg::vec3(
            radius * cos(angle),
            camera.position.y - camera.target.y,
            radius * sin(angle)
        );
        
        if (lookAt) {
            lookAt->eye = camera.position;
        }
    }
}

void Visualizer::addDirectionalLight(const vsg::vec3& direction, const vsg::vec3& color, float intensity) {
    Light light;
    light.position = -direction * 1000.0f; // Far away for directional effect
    light.color = color;
    light.intensity = intensity;
    
    auto directionalLight = vsg::DirectionalLight::create();
    directionalLight->direction = direction;
    directionalLight->color = color;
    directionalLight->intensity = intensity;
    
    if (shadowsEnabled) {
        // Setup shadow mapping for this light
        // This would require shadow map render passes
    }
    
    light.directional = directionalLight;
    lights.push_back(light);
    
    sceneRoot->addChild(directionalLight);
}

void Visualizer::addPointLight(const vsg::vec3& position, const vsg::vec3& color, float intensity, float range) {
    Light light;
    light.position = position;
    light.color = color;
    light.intensity = intensity;
    
    auto pointLight = vsg::PointLight::create();
    pointLight->position = position;
    pointLight->color = color;
    pointLight->intensity = intensity;
    
    light.point = pointLight;
    lights.push_back(light);
    
    // Create transform for the light
    auto transform = vsg::MatrixTransform::create();
    transform->matrix = vsg::translate(position);
    transform->addChild(pointLight);
    
    sceneRoot->addChild(transform);
}

void Visualizer::addSpotLight(const vsg::vec3& position, const vsg::vec3& direction, 
                             const vsg::vec3& color, float intensity, float angle) {
    Light light;
    light.position = position;
    light.color = color;
    light.intensity = intensity;
    
    auto spotLight = vsg::SpotLight::create();
    spotLight->position = position;
    spotLight->direction = direction;
    spotLight->color = color;
    spotLight->intensity = intensity;
    spotLight->innerConeAngle = angle * 0.8f;
    spotLight->outerConeAngle = angle;
    
    light.spot = spotLight;
    lights.push_back(light);
    
    sceneRoot->addChild(spotLight);
}

void Visualizer::setSkybox(const std::string& path) {
    // Load skybox texture
    auto options = vsg::Options::create();
    options->paths = { path };
    
    // Create skybox geometry
    auto builder = vsg::Builder::create();
    
    // Large box for skybox
    auto box = vsg::Box::create();
    box->min = vsg::vec3(-500.0f, -500.0f, -500.0f);
    box->max = vsg::vec3(500.0f, 500.0f, 500.0f);
    
    vsg::GeometryInfo geomInfo;
    geomInfo.box = box;
    
    auto skyboxNode = builder->createBox(geomInfo);
    
    // Add to scene at the beginning so it renders behind everything
    sceneRoot->children.insert(sceneRoot->children.begin(), skyboxNode);
}

void Visualizer::setFog(const vsg::vec3& color, float density) {
    // Fog would be implemented in shaders
    // This is a placeholder for the interface
}

void Visualizer::drawLine(const vsg::vec3& start, const vsg::vec3& end, const vsg::vec4& color) {
    auto builder = vsg::Builder::create();
    
    // Create line geometry
    auto vertices = vsg::vec3Array::create(2);
    (*vertices)[0] = start;
    (*vertices)[1] = end;
    
    auto colors = vsg::vec4Array::create(2);
    (*colors)[0] = color;
    (*colors)[1] = color;
    
    // Create drawable
    auto drawCommands = vsg::Commands::create();
    drawCommands->addChild(vsg::Draw::create(2, 0, 0, 0));
    
    auto vid = vsg::VertexInputData::create();
    vid->arrays = { vertices, colors };
    
    auto node = vsg::StateGroup::create();
    node->addChild(drawCommands);
    
    debugGroup->addChild(node);
}

void Visualizer::drawSphere(const vsg::vec3& center, float radius, const vsg::vec4& color) {
    auto transform = vsg::MatrixTransform::create();
    transform->matrix = vsg::translate(center);
    
    auto builder = vsg::Builder::create();
    auto sphere = vsg::Sphere::create();
    sphere->radius = radius;
    
    vsg::GeometryInfo geomInfo;
    geomInfo.sphere = sphere;
    geomInfo.color = color;
    
    auto node = builder->createSphere(geomInfo);
    transform->addChild(node);
    
    debugGroup->addChild(transform);
}

void Visualizer::drawBox(const vsg::vec3& center, const vsg::vec3& size, 
                        const vsg::quat& rotation, const vsg::vec4& color) {
    auto transform = vsg::MatrixTransform::create();
    transform->matrix = vsg::translate(center) * vsg::rotate(rotation);
    
    auto builder = vsg::Builder::create();
    auto box = vsg::Box::create();
    box->min = -size * 0.5f;
    box->max = size * 0.5f;
    
    vsg::GeometryInfo geomInfo;
    geomInfo.box = box;
    geomInfo.color = color;
    
    auto node = builder->createBox(geomInfo);
    transform->addChild(node);
    
    debugGroup->addChild(transform);
}

void Visualizer::clearDebugGeometry() {
    debugGroup->children.clear();
}

void Visualizer::setupPostProcessing() {
    if (!postProcessingEnabled) return;
    
    // Post-processing would involve:
    // 1. Rendering scene to offscreen framebuffer
    // 2. Applying effects in screen-space passes
    // 3. Final composite to screen
    
    // This is a complex topic that would require significant implementation
}

void Visualizer::updateStats() {
    frameTime += 0.016; // Approximate for 60 FPS
    frameCount++;
    
    if (showStatistics && frameCount % 60 == 0) {
        // Update statistics display
        // In a real implementation, this would update an overlay
    }
}