#include "Visualizer.h"
#include <iostream>

#ifdef USE_OPENGL_FALLBACK
    #include <cmath>
    #include <cstring>
    
    // Simple OpenGL matrix operations
    static void gluLookAt(float eyeX, float eyeY, float eyeZ,
                          float centerX, float centerY, float centerZ,
                          float upX, float upY, float upZ) {
        float forward[3], side[3], up[3];
        float m[4][4];
        
        forward[0] = centerX - eyeX;
        forward[1] = centerY - eyeY;
        forward[2] = centerZ - eyeZ;
        
        up[0] = upX;
        up[1] = upY;
        up[2] = upZ;
        
        float forwardMag = sqrt(forward[0]*forward[0] + forward[1]*forward[1] + forward[2]*forward[2]);
        forward[0] /= forwardMag;
        forward[1] /= forwardMag;
        forward[2] /= forwardMag;
        
        // Side = forward x up
        side[0] = forward[1] * up[2] - forward[2] * up[1];
        side[1] = forward[2] * up[0] - forward[0] * up[2];
        side[2] = forward[0] * up[1] - forward[1] * up[0];
        
        float sideMag = sqrt(side[0]*side[0] + side[1]*side[1] + side[2]*side[2]);
        side[0] /= sideMag;
        side[1] /= sideMag;
        side[2] /= sideMag;
        
        // Recompute up as: up = side x forward
        up[0] = side[1] * forward[2] - side[2] * forward[1];
        up[1] = side[2] * forward[0] - side[0] * forward[2];
        up[2] = side[0] * forward[1] - side[1] * forward[0];
        
        m[0][0] = side[0];
        m[1][0] = side[1];
        m[2][0] = side[2];
        m[3][0] = 0.0;
        
        m[0][1] = up[0];
        m[1][1] = up[1];
        m[2][1] = up[2];
        m[3][1] = 0.0;
        
        m[0][2] = -forward[0];
        m[1][2] = -forward[1];
        m[2][2] = -forward[2];
        m[3][2] = 0.0;
        
        m[0][3] = 0.0;
        m[1][3] = 0.0;
        m[2][3] = 0.0;
        m[3][3] = 1.0;
        
        glMultMatrixf(&m[0][0]);
        glTranslatef(-eyeX, -eyeY, -eyeZ);
    }
    
    static void gluPerspective(float fovy, float aspect, float zNear, float zFar) {
        float fH = tan(fovy / 360 * M_PI) * zNear;
        float fW = fH * aspect;
        glFrustum(-fW, fW, -fH, fH, zNear, zFar);
    }
#else
#include <vsg/io/Options.h>
#include <vsg/nodes/MatrixTransform.h>
#include <vsg/utils/Builder.h>
#include <vsg/utils/ComputeBounds.h>
#include <vsg/lighting/AmbientLight.h>
#include <vsg/lighting/DirectionalLight.h>
#include <vsg/lighting/PointLight.h>
#include <vsg/lighting/SpotLight.h>
#include <vsg/nodes/CullGroup.h>
#include <vsg/nodes/AbsoluteTransform.h>
#include <vsg/utils/ShaderSet.h>
#endif

Visualizer::Visualizer(uint32_t width, uint32_t height) 
    : windowWidth(width), windowHeight(height) {
    
    // Initialize camera
    camera.position = vsg_vec3(10.0f, 10.0f, 10.0f);
    camera.target = vsg_vec3(0.0f, 0.0f, 0.0f);
    camera.up = vsg_vec3(0.0f, 1.0f, 0.0f);
    camera.fov = 60.0f;
    camera.nearPlane = 0.1f;
    camera.farPlane = 1000.0f;
    camera.followRobot = false;
    camera.followDistance = 10.0f;
    camera.followHeight = 5.0f;
    
    lastTime = std::chrono::high_resolution_clock::now();
}

Visualizer::~Visualizer() {
#ifdef USE_OPENGL_FALLBACK
    if (window) {
        glfwDestroyWindow(window);
        glfwTerminate();
    }
#endif
}

bool Visualizer::initialize() {
    try {
#ifdef USE_OPENGL_FALLBACK
        // Initialize GLFW
        if (!glfwInit()) {
            std::cerr << "Failed to initialize GLFW" << std::endl;
            return false;
        }
        
        // Create window
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);
        
        window = glfwCreateWindow(windowWidth, windowHeight, "Robot Simulation (OpenGL)", nullptr, nullptr);
        if (!window) {
            std::cerr << "Failed to create GLFW window" << std::endl;
            glfwTerminate();
            return false;
        }
        
        glfwMakeContextCurrent(window);
        glfwSwapInterval(vsyncEnabled ? 1 : 0);
        
        // Initialize OpenGL
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LIGHTING);
        glEnable(GL_LIGHT0);
        
        float lightPos[] = {1.0f, 1.0f, 1.0f, 0.0f};
        float lightColor[] = {1.0f, 1.0f, 1.0f, 1.0f};
        glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
        glLightfv(GL_LIGHT0, GL_DIFFUSE, lightColor);
        
        glClearColor(0.2f, 0.3f, 0.4f, 1.0f);
        
        // Create scene root
        sceneRoot = ref_ptr<Group>(new Group());
        
        return true;
#else
        // Modern VSG initialization using patterns from examples
        
        // 1. Create options with vsgXchange support (from vsghelloworld example)
        auto options = vsg::Options::create();
        options->sharedObjects = vsg::SharedObjects::create();
        options->fileCache = vsg::getEnv("VSG_FILE_CACHE");
        options->paths = vsg::getEnvPaths("VSG_FILE_PATH");
        
        // Add vsgXchange support for loading various file formats
        options->add(vsgXchange::all::create());
        
        // 2. Create window traits
        auto windowTraits = vsg::WindowTraits::create();
        windowTraits->windowTitle = "VSG-ODE Robot Simulation";
        windowTraits->width = windowWidth;
        windowTraits->height = windowHeight;
        windowTraits->decoration = true;
        windowTraits->samples = msaaSamples;
        
        // 3. Create window
        window = vsg::Window::create(windowTraits);
        if (!window) {
            throw std::runtime_error("Failed to create VSG window");
        }
        
        // 4. Create viewer and add window
        viewer = vsg::Viewer::create();
        viewer->addWindow(window);
        
        // 5. Create scene with some basic geometry
        sceneRoot = createScene(options);
        
        // 6. Compute bounds for camera positioning
        vsg::ComputeBounds computeBounds;
        sceneRoot->accept(computeBounds);
        vsg::dvec3 centre = (computeBounds.bounds.min + computeBounds.bounds.max) * 0.5;
        double radius = vsg::length(computeBounds.bounds.max - computeBounds.bounds.min) * 0.6;
        
        // 7. Setup camera (using patterns from examples)
        lookAt = vsg::LookAt::create(
            centre + vsg::dvec3(0.0, -radius * 3.5, 0.0), 
            centre, 
            vsg::dvec3(0.0, 0.0, 1.0)
        );
        
        double nearFarRatio = 0.001;
        perspective = vsg::Perspective::create(
            30.0, 
            static_cast<double>(window->extent2D().width) / static_cast<double>(window->extent2D().height), 
            nearFarRatio * radius, 
            radius * 4.5
        );
        
        vsgCamera = vsg::Camera::create(perspective, lookAt, vsg::ViewportState::create(window->extent2D()));
        
        // 8. Add event handlers
        viewer->addEventHandler(vsg::CloseHandler::create(viewer));
        viewer->addEventHandler(vsg::Trackball::create(vsgCamera));
        
        // 9. Create command graph using modern simplified approach
        auto commandGraph = vsg::createCommandGraphForView(window, vsgCamera, sceneRoot);
        viewer->assignRecordAndSubmitTaskAndPresentation({commandGraph});
        
        // 10. Compile everything
        viewer->compile();
        
        std::cout << "VSG initialization complete with modern API patterns" << std::endl;
        return true;
#endif
    } catch (const std::exception& e) {
        std::cerr << "Failed to initialize visualizer: " << e.what() << std::endl;
        return false;
    }
}

void Visualizer::render() {
#ifdef USE_OPENGL_FALLBACK
    renderOpenGL();
#else
    // Update frame time
    auto currentTime = std::chrono::high_resolution_clock::now();
    frameTime = std::chrono::duration<double>(currentTime - lastTime).count();
    lastTime = currentTime;
    
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
#endif
}

bool Visualizer::shouldClose() const {
#ifdef USE_OPENGL_FALLBACK
    return glfwWindowShouldClose(window);
#else
    return !viewer->active();
#endif
}

#ifdef USE_OPENGL_FALLBACK
void Visualizer::renderOpenGL() {
    // Update camera if following
    updateCamera();
    
    // Clear
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    // Setup matrices
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(camera.fov, (float)windowWidth / windowHeight, camera.nearPlane, camera.farPlane);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(camera.position.x, camera.position.y, camera.position.z,
              camera.target.x, camera.target.y, camera.target.z,
              camera.up.x, camera.up.y, camera.up.z);
    
    // Draw ground plane
    glColor3f(0.3f, 0.6f, 0.3f);
    glBegin(GL_QUADS);
    glNormal3f(0, 1, 0);
    glVertex3f(-50, 0, -50);
    glVertex3f(50, 0, -50);
    glVertex3f(50, 0, 50);
    glVertex3f(-50, 0, 50);
    glEnd();
    
    // Draw some sample geometry representing the robot
    glPushMatrix();
    glTranslatef(0, 1, 0);
    glColor3f(0.2f, 0.3f, 0.8f);
    
    // Simple robot body representation
    glPushMatrix();
    glScalef(2, 0.5f, 1);
    drawCube(vsg_vec3(0, 0, 0), vsg_vec3(1, 1, 1), vsg_vec4(0.2f, 0.3f, 0.8f, 1.0f));
    glPopMatrix();
    
    // Robot legs (simplified)
    for (int i = 0; i < 6; ++i) {
        float angle = i * M_PI / 3.0f;
        float x = cos(angle) * 1.5f;
        float z = sin(angle) * 1.0f;
        
        glPushMatrix();
        glTranslatef(x, -0.5f, z);
        glScalef(0.1f, 1.0f, 0.1f);
        drawCube(vsg_vec3(0, 0, 0), vsg_vec3(1, 1, 1), vsg_vec4(0.4f, 0.4f, 0.4f, 1.0f));
        glPopMatrix();
    }
    
    glPopMatrix();
    
    // Swap buffers
    glfwSwapBuffers(window);
    glfwPollEvents();
    
    frameCount++;
}

void Visualizer::drawCube(const vsg_vec3& position, const vsg_vec3& size, const vsg_vec4& color) {
    glColor4f(color.x, color.y, color.z, color.w);
    
    float x = size.x * 0.5f;
    float y = size.y * 0.5f;
    float z = size.z * 0.5f;
    
    glBegin(GL_QUADS);
    // Front face
    glNormal3f(0, 0, 1);
    glVertex3f(-x, -y, z);
    glVertex3f(x, -y, z);
    glVertex3f(x, y, z);
    glVertex3f(-x, y, z);
    
    // Back face
    glNormal3f(0, 0, -1);
    glVertex3f(-x, -y, -z);
    glVertex3f(-x, y, -z);
    glVertex3f(x, y, -z);
    glVertex3f(x, -y, -z);
    
    // Top face
    glNormal3f(0, 1, 0);
    glVertex3f(-x, y, -z);
    glVertex3f(-x, y, z);
    glVertex3f(x, y, z);
    glVertex3f(x, y, -z);
    
    // Bottom face
    glNormal3f(0, -1, 0);
    glVertex3f(-x, -y, -z);
    glVertex3f(x, -y, -z);
    glVertex3f(x, -y, z);
    glVertex3f(-x, -y, z);
    
    // Right face
    glNormal3f(1, 0, 0);
    glVertex3f(x, -y, -z);
    glVertex3f(x, y, -z);
    glVertex3f(x, y, z);
    glVertex3f(x, -y, z);
    
    // Left face
    glNormal3f(-1, 0, 0);
    glVertex3f(-x, -y, -z);
    glVertex3f(-x, -y, z);
    glVertex3f(-x, y, z);
    glVertex3f(-x, y, -z);
    glEnd();
}

void Visualizer::drawSphereOpenGL(const vsg_vec3& center, float radius, const vsg_vec4& color) {
    glColor4f(color.x, color.y, color.z, color.w);
    
    glPushMatrix();
    glTranslatef(center.x, center.y, center.z);
    
    // Simple sphere approximation using quad strips
    const int slices = 16;
    const int stacks = 16;
    
    for (int i = 0; i < stacks; ++i) {
        float lat0 = M_PI * (-0.5f + (float)i / stacks);
        float z0 = sin(lat0);
        float zr0 = cos(lat0);
        
        float lat1 = M_PI * (-0.5f + (float)(i + 1) / stacks);
        float z1 = sin(lat1);
        float zr1 = cos(lat1);
        
        glBegin(GL_QUAD_STRIP);
        for (int j = 0; j <= slices; ++j) {
            float lng = 2 * M_PI * (float)j / slices;
            float x = cos(lng);
            float y = sin(lng);
            
            glNormal3f(x * zr0, y * zr0, z0);
            glVertex3f(radius * x * zr0, radius * y * zr0, radius * z0);
            
            glNormal3f(x * zr1, y * zr1, z1);
            glVertex3f(radius * x * zr1, radius * y * zr1, radius * z1);
        }
        glEnd();
    }
    
    glPopMatrix();
}

#endif

// Common implementations
void Visualizer::setCameraPosition(const vsg_vec3& position) {
    camera.position = position;
#ifndef USE_OPENGL_FALLBACK
    if (lookAt) {
        lookAt->eye = vsg::dvec3(position.x, position.y, position.z);
    }
#endif
}

void Visualizer::setCameraTarget(const vsg_vec3& target) {
    camera.target = target;
#ifndef USE_OPENGL_FALLBACK
    if (lookAt) {
        lookAt->center = vsg::dvec3(target.x, target.y, target.z);
    }
#endif
}

void Visualizer::enableCameraFollow(bool enable, const vsg_vec3& targetPos) {
    camera.followRobot = enable;
    if (enable) {
        camera.target = targetPos;
    }
}

void Visualizer::updateCamera() {
    if (camera.followRobot && cameraMode == 1) {
        // Calculate follow position
        vsg_vec3 offset(
            -camera.followDistance * cos(frameTime * 0.1f),
            camera.followHeight,
            -camera.followDistance * sin(frameTime * 0.1f)
        );
        
        camera.position = camera.target + offset;
        
#ifndef USE_OPENGL_FALLBACK
        if (lookAt) {
            lookAt->eye = vsg::dvec3(camera.position.x, camera.position.y, camera.position.z);
            lookAt->center = vsg::dvec3(camera.target.x, camera.target.y, camera.target.z);
        }
#endif
    } else if (cameraMode == 2) {
        // Orbit mode
        float angle = frameTime * 0.2f;
        vsg_vec3 diff = camera.position - camera.target;
        float radius = vsg::length(diff);
        
        camera.position = camera.target + vsg_vec3(
            radius * cos(angle),
            camera.position.y - camera.target.y,
            radius * sin(angle)
        );
        
#ifndef USE_OPENGL_FALLBACK
        if (lookAt) {
            lookAt->eye = vsg::dvec3(camera.position.x, camera.position.y, camera.position.z);
        }
#endif
    }
}

void Visualizer::addDirectionalLight(const vsg_vec3& direction, const vsg_vec3& color, float intensity) {
    Light light;
    light.position = direction * -1000.0f; // Far away for directional effect
    light.color = color;
    light.intensity = intensity;
    
#ifdef USE_OPENGL_FALLBACK
    // Simple OpenGL lighting setup
    static int lightIndex = 0;
    if (lightIndex < 8) {
        GLenum lightId = GL_LIGHT0 + lightIndex;
        glEnable(lightId);
        
        float pos[] = {direction.x, direction.y, direction.z, 0.0f}; // Directional light
        float col[] = {color.x * intensity, color.y * intensity, color.z * intensity, 1.0f};
        
        glLightfv(lightId, GL_POSITION, pos);
        glLightfv(lightId, GL_DIFFUSE, col);
        glLightfv(lightId, GL_SPECULAR, col);
        
        lightIndex++;
    }
#else
    auto directionalLight = vsg::DirectionalLight::create();
    directionalLight->direction = direction;
    directionalLight->color = color;
    directionalLight->intensity = intensity;
    
    if (shadowsEnabled) {
        // Setup shadow mapping for this light
        // This would require shadow map render passes
    }
    
    light.directional = directionalLight;
    sceneRoot->addChild(directionalLight);
#endif
    
    lights.push_back(light);
}

void Visualizer::addPointLight(const vsg_vec3& position, const vsg_vec3& color, float intensity, float range) {
    Light light;
    light.position = position;
    light.color = color;
    light.intensity = intensity;
    
#ifdef USE_OPENGL_FALLBACK
    // OpenGL point light (simplified)
    static int lightIndex = 1; // Start from 1 since 0 is often used by directional
    if (lightIndex < 8) {
        GLenum lightId = GL_LIGHT0 + lightIndex;
        glEnable(lightId);
        
        float pos[] = {position.x, position.y, position.z, 1.0f}; // Point light
        float col[] = {color.x * intensity, color.y * intensity, color.z * intensity, 1.0f};
        
        glLightfv(lightId, GL_POSITION, pos);
        glLightfv(lightId, GL_DIFFUSE, col);
        glLightfv(lightId, GL_SPECULAR, col);
        
        // Attenuation
        glLightf(lightId, GL_CONSTANT_ATTENUATION, 1.0f);
        glLightf(lightId, GL_LINEAR_ATTENUATION, 2.0f / range);
        glLightf(lightId, GL_QUADRATIC_ATTENUATION, 1.0f / (range * range));
        
        lightIndex++;
    }
#else
    auto pointLight = vsg::PointLight::create();
    pointLight->position = position;
    pointLight->color = color;
    pointLight->intensity = intensity;
    
    light.point = pointLight;
    
    // Create transform for the light
    auto transform = vsg::MatrixTransform::create();
    transform->matrix = vsg::translate(position);
    transform->addChild(pointLight);
    
    sceneRoot->addChild(transform);
#endif
    
    lights.push_back(light);
}

void Visualizer::addSpotLight(const vsg_vec3& position, const vsg_vec3& direction, 
                             const vsg_vec3& color, float intensity, float angle) {
    Light light;
    light.position = position;
    light.color = color;
    light.intensity = intensity;
    
#ifdef USE_OPENGL_FALLBACK
    // OpenGL spot light (simplified implementation)
    // For now, treat as point light
    addPointLight(position, color, intensity, 50.0f);
#else
    auto spotLight = vsg::SpotLight::create();
    spotLight->position = position;
    spotLight->direction = direction;
    spotLight->color = color;
    spotLight->intensity = intensity;
    spotLight->innerAngle = angle * 0.8f;
    spotLight->outerAngle = angle;
    
    light.spot = spotLight;
    sceneRoot->addChild(spotLight);
#endif
    
    lights.push_back(light);
}

void Visualizer::setSkybox(const std::string& path) {
#ifdef USE_OPENGL_FALLBACK
    // Simple skybox color for OpenGL fallback
    glClearColor(0.5f, 0.7f, 1.0f, 1.0f); // Sky blue
#else
    // Load skybox texture
    auto options = vsg::Options::create();
    options->paths = { path };
    
    // TODO: Update for current VSG API
    // Skybox will be added once we fix VSG geometry API
    /*
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
    */
#endif
}

void Visualizer::setFog(const vsg_vec3& color, float density) {
#ifdef USE_OPENGL_FALLBACK
    glEnable(GL_FOG);
    glFogi(GL_FOG_MODE, GL_EXP2);
    glFogf(GL_FOG_DENSITY, density);
    float fogColor[] = {color.x, color.y, color.z, 1.0f};
    glFogfv(GL_FOG_COLOR, fogColor);
#else
    // Fog would be implemented in shaders
    // This is a placeholder for the interface
#endif
}

void Visualizer::drawLine(const vsg_vec3& start, const vsg_vec3& end, const vsg_vec4& color) {
#ifdef USE_OPENGL_FALLBACK
    glDisable(GL_LIGHTING);
    glColor4f(color.x, color.y, color.z, color.w);
    glBegin(GL_LINES);
    glVertex3f(start.x, start.y, start.z);
    glVertex3f(end.x, end.y, end.z);
    glEnd();
    glEnable(GL_LIGHTING);
#else
    // TODO: Update for current VSG API
    // Line drawing will be implemented once we update the VSG geometry API
#endif
}

void Visualizer::drawSphere(const vsg_vec3& center, float radius, const vsg_vec4& color) {
#ifdef USE_OPENGL_FALLBACK
    glPushMatrix();
    glTranslatef(center.x, center.y, center.z);
    drawSphereOpenGL(vsg_vec3(0, 0, 0), radius, color);
    glPopMatrix();
#else
    // TODO: Update for current VSG API
    // Sphere drawing will be implemented once we update the VSG geometry API
#endif
}

void Visualizer::drawBox(const vsg_vec3& center, const vsg_vec3& size, 
                        const vsg_quat& rotation, const vsg_vec4& color) {
#ifdef USE_OPENGL_FALLBACK
    glPushMatrix();
    glTranslatef(center.x, center.y, center.z);
    // Simplified rotation (just identity for now)
    drawCube(vsg_vec3(0, 0, 0), size, color);
    glPopMatrix();
#else
    // TODO: Update for current VSG API
    // Box drawing will be implemented once we update the VSG geometry API
#endif
}

void Visualizer::clearDebugGeometry() {
#ifndef USE_OPENGL_FALLBACK
    // TODO: Implement debug geometry clearing when VSG is fully supported
    /*
    if (debugGroup) {
        debugGroup->children.clear();
    }
    */
#endif
}

void Visualizer::setupPostProcessing() {
    if (!postProcessingEnabled) return;
    
#ifdef USE_OPENGL_FALLBACK
    // Post-processing not implemented in OpenGL fallback
#else
    // Post-processing would involve:
    // 1. Rendering scene to offscreen framebuffer
    // 2. Applying effects in screen-space passes
    // 3. Final composite to screen
    
    // This is a complex topic that would require significant implementation
#endif
}

void Visualizer::updateStats() {
    frameCount++;
    
    if (showStatistics && frameCount % 60 == 0) {
        renderTime = frameTime * 1000.0; // Convert to milliseconds
        double fps = frameTime > 0.0 ? 1.0 / frameTime : 0.0;
        std::cout << "Frame: " << frameCount << ", Frame time: " << renderTime 
                  << "ms, FPS: " << fps << std::endl;
    }
}

void Visualizer::createAxisIndicator() {
#ifndef USE_OPENGL_FALLBACK
    // TODO: Update for current VSG API
    // Axis indicator will be added once we fix VSG geometry API
    
    axisIndicator = vsg::Group::create();
    
    /*
    axisIndicator = vsg::Group::create();
    
    auto builder = vsg::Builder::create();
    
    // Create axis lines
    auto box = vsg::Box::create();
    box->min = vsg::vec3(-0.01f, -0.01f, 0.0f);
    box->max = vsg::vec3(0.01f, 0.01f, 0.2f);
    
    vsg::GeometryInfo geomInfo;
    geomInfo.box = box;
    geomInfo.color = vsg::vec4(1.0f, 0.0f, 0.0f, 1.0f);
    
    auto node = builder->createBox(geomInfo);
    axisIndicator->addChild(node);
    */
#endif
}

void Visualizer::createLighting() {
#ifndef USE_OPENGL_FALLBACK
    // TODO: Update for current VSG API
    // Lighting will be added once we fix VSG geometry API
    
    /*
    // Create directional light
    auto light = vsg::DirectionalLight::create();
    light->name = "sun";
    light->color = vsg::vec3(1.0f, 1.0f, 0.95f);
    light->intensity = 1.0f;
    light->direction = vsg::vec3(0.5f, -1.0f, 0.5f);
    
    scene->addChild(light);
    
    // Create ambient light
    auto ambientLight = vsg::AmbientLight::create();
    ambientLight->name = "ambient";
    ambientLight->color = vsg::vec3(0.1f, 0.1f, 0.15f);
    ambientLight->intensity = 0.1f;
    
    scene->addChild(ambientLight);
    
    // Create spot lights for dramatic effect
    for (int i = 0; i < 2; ++i) {
        auto spotLight = vsg::SpotLight::create();
        spotLight->name = "spot" + std::to_string(i);
        spotLight->color = vsg::vec3(1.0f, 0.8f, 0.6f);
        spotLight->intensity = 2.0f;
        spotLight->position = vsg::vec3(i == 0 ? 10.0f : -10.0f, 8.0f, 5.0f);
        spotLight->direction = vsg::vec3(0.0f, -1.0f, 0.0f);
        spotLight->innerAngle = 30.0f;
        spotLight->outerAngle = 45.0f;
        
        scene->addChild(spotLight);
    }
    */
#endif
}

void Visualizer::addBox(const vsg_vec3& position, const vsg_vec3& size, const vsg_vec4& color) {
#ifndef USE_OPENGL_FALLBACK
    // TODO: Update for current VSG API
    // Box creation will be added once we fix VSG geometry API
    
    /*
    auto transform = vsg::MatrixTransform::create();
    transform->matrix = vsg::translate(position);
    
    auto builder = vsg::Builder::create();
    auto box = vsg::Box::create();
    box->min = -size * 0.5f;
    box->max = size * 0.5f;
    
    vsg::GeometryInfo geomInfo;
    geomInfo.box = box;
    geomInfo.color = color;
    
    auto node = builder->createBox(geomInfo);
    transform->addChild(node);
    
    scene->addChild(transform);
    */
#endif
}

#ifndef USE_OPENGL_FALLBACK
vsg::ref_ptr<vsg::Group> Visualizer::createScene(vsg::ref_ptr<vsg::Options> options) {
    // Create builder for generating geometry
    auto builder = vsg::Builder::create();
    builder->options = options;
    
    auto scene = vsg::Group::create();
    
    // Create some sample geometry representing a robot simulation
    vsg::GeometryInfo geomInfo;
    vsg::StateInfo stateInfo;
    
    // Ground plane
    geomInfo.position.set(0.0f, 0.0f, 0.0f);
    geomInfo.dx.set(50.0f, 0.0f, 0.0f);
    geomInfo.dy.set(0.0f, 0.0f, 50.0f);
    scene->addChild(builder->createQuad(geomInfo, stateInfo));
    
    // Robot body (box)
    geomInfo.position.set(0.0f, 1.0f, 0.0f);
    geomInfo.dx.set(2.0f, 0.0f, 0.0f);
    geomInfo.dy.set(0.0f, 0.5f, 0.0f);
    geomInfo.dz.set(0.0f, 0.0f, 1.0f);
    scene->addChild(builder->createBox(geomInfo, stateInfo));
    
    // Robot legs (simplified as cylinders)
    for (int i = 0; i < 6; ++i) {
        double angle = i * vsg::PI / 3.0;
        double x = cos(angle) * 1.5;
        double z = sin(angle) * 1.0;
        
        auto transform = vsg::MatrixTransform::create();
        transform->matrix = vsg::translate(x, 0.0, z);
        
        geomInfo.position.set(0.0f, 0.0f, 0.0f);
        geomInfo.dx.set(0.1f, 0.0f, 0.0f);
        geomInfo.dy.set(0.0f, 1.0f, 0.0f);
        geomInfo.dz.set(0.0f, 0.0f, 0.1f);
        
        transform->addChild(builder->createCylinder(geomInfo, stateInfo));
        scene->addChild(transform);
    }
    
    // Add lighting (using patterns from vsglights example)
    setupModernLighting(scene);
    
    return scene;
}

void Visualizer::setupModernLighting(vsg::ref_ptr<vsg::Group> scene) {
    // Ambient light
    auto ambientLight = vsg::AmbientLight::create();
    ambientLight->name = "ambient";
    ambientLight->color.set(1.0f, 1.0f, 1.0f);
    ambientLight->intensity = 0.1f;
    scene->addChild(ambientLight);
    
    // Directional light (sun)
    auto directionalLight = vsg::DirectionalLight::create();
    directionalLight->name = "sun";
    directionalLight->color.set(1.0f, 1.0f, 0.9f);
    directionalLight->intensity = 0.8f;
    directionalLight->direction.set(0.2f, -1.0f, -0.3f);
    scene->addChild(directionalLight);
    
    // Point light for some accent lighting
    auto pointLight = vsg::PointLight::create();
    pointLight->name = "accent";
    pointLight->color.set(0.3f, 0.6f, 1.0f);
    pointLight->intensity = 10.0f;
    pointLight->position.set(5.0f, 5.0f, 5.0f);
    
    // Enable culling for the point light
    auto cullGroup = vsg::CullGroup::create();
    cullGroup->bound.center = pointLight->position;
    cullGroup->bound.radius = 20.0f;
    cullGroup->addChild(pointLight);
    scene->addChild(cullGroup);
}
#endif