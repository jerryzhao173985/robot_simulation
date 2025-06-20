#include "Visualizer.h"
#include "VisualizerHexapodConfig.h"
#include <iostream>

// Debug control
// #define DEBUG_VISUALIZER


 #ifdef USE_OPENGL_FALLBACK
    #include <cmath>
    #include <algorithm>
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

#include "PositionUtils.h"

Visualizer::Visualizer(uint32_t width, uint32_t height) 
    : windowWidth(width), windowHeight(height) {
    
    // Initialize camera
    camera.position = vsg_vec3(10.0f, 10.0f, 10.0f);
    camera.target = vsg_vec3(0.0f, 0.0f, 0.0f);
    camera.up = vsg_vec3(0.0f, 0.0f, 1.0f);
    camera.fov = 60.0f;
    camera.nearPlane = 0.1f;
    camera.farPlane = 1000.0f;
    camera.followRobot = false;
    camera.followDistance = 10.0f;
    camera.followHeight = 5.0f;

    // // Initialize camera: front-right 45° view, Z-up coordinate
    // camera.position       = vsg_vec3(2.0f, -2.0f, 1.5f);
    // camera.target         = vsg_vec3(0.0f,  0.0f,  0.3f);
    // camera.up             = vsg_vec3(0.0f,  0.0f,  1.0f);
    // camera.fov            = 45.0f;
    // camera.nearPlane      = 0.1f;
    // camera.farPlane       = 100.0f;
    // camera.followRobot    = false;
    // camera.followDistance = 5.0f;
    // camera.followHeight   = 2.0f;
    
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

        // Initialize OpenGL settings
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LIGHTING);
        glEnable(GL_LIGHT0);
        float lightPos[]   = {1.0f, 1.0f, 1.0f, 0.0f};
        float lightColor[] = {1.0f, 1.0f, 1.0f, 1.0f};
        glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
        glLightfv(GL_LIGHT0, GL_DIFFUSE,  lightColor);
        glClearColor(0.2f, 0.3f, 0.4f, 1.0f);

        // Setup simple mouse-driven arcball for free view
        glfwSetWindowUserPointer(window, this);
        glfwSetMouseButtonCallback(window,
            [](GLFWwindow* w, int button, int action, int) {
                auto viz = static_cast<Visualizer*>(glfwGetWindowUserPointer(w));
                if (button == GLFW_MOUSE_BUTTON_LEFT) {
                    if (action == GLFW_PRESS) {
                        viz->mouseDragging = true;
                        glfwGetCursorPos(w, &viz->dragLastX, &viz->dragLastY);
                    } else {
                        viz->mouseDragging = false;
                    }
                }
            }
        );
        glfwSetCursorPosCallback(window,
            [](GLFWwindow* w, double xpos, double ypos) {
                auto viz = static_cast<Visualizer*>(glfwGetWindowUserPointer(w));
                if (!viz->mouseDragging) return;
                double dx = xpos - viz->dragLastX;
                double dy = ypos - viz->dragLastY;
                viz->dragLastX = xpos;
                viz->dragLastY = ypos;
                viz->freeAzimuth   += static_cast<float>(dx * 0.005);
                viz->freeElevation += static_cast<float>(dy * 0.005);
                viz->freeElevation = std::clamp(viz->freeElevation, -1.5f, 1.5f);
            }
        );

        // Create scene root
        sceneRoot = ref_ptr<Group>(new Group());
        
        return true;
//  Modern VSG initialization using patterns from vsghelloworld example
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
        double radius = vsg::length(computeBounds.bounds.max - computeBounds.bounds.min) * 0.8;
        
        // 7. Setup camera (using patterns from examples)
        lookAt = vsg::LookAt::create(
            centre + vsg::dvec3(-radius * 2.0, -radius * 2.5, radius * 1.5), // Better angle for hexapod
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
        
        // IMPORTANT: viewer->update() must be called AFTER all transforms are updated
        // The transforms should be updated before this render() call
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
    // Fallback free-camera controls (arrow keys + +/- zoom)
#ifdef USE_OPENGL_FALLBACK
    // Free-camera controls only in fallback mode
    if (cameraMode == 0) {
        const float azStep = 0.02f;
        const float elStep = 0.02f;
        const float zoomStep = 0.2f;
        if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS)      freeAzimuth -= azStep;
        if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS)     freeAzimuth += azStep;
        freeElevation = std::clamp(
            freeElevation + (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS ? elStep : 0.0f)
            - (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS ? elStep : 0.0f),
            -1.5f, 1.5f);
        if (glfwGetKey(window, GLFW_KEY_EQUAL) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_ADD) == GLFW_PRESS)
            freeDistance = std::max(1.0f, freeDistance - zoomStep);
        if (glfwGetKey(window, GLFW_KEY_MINUS) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_KP_SUBTRACT) == GLFW_PRESS)
            freeDistance += zoomStep;
        // Z-up free-camera: elevation moves along Z; horizontal rotation in X-Y plane
        camera.position.x = freeDistance * cos(freeElevation) * sin(freeAzimuth);
        camera.position.y = freeDistance * cos(freeElevation) * cos(freeAzimuth);
        camera.position.z = freeDistance * sin(freeElevation);
        camera.target      = vsg_vec3(0.0f, 0.0f, 0.0f);
        return;
    }
#endif

    // Follow mode
    if (cameraMode == 1) {
        vsg_vec3 offset(
            -camera.followDistance * cos(frameTime * 0.1f),
            -camera.followDistance * sin(frameTime * 0.1f),
            camera.followHeight
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
            radius * sin(angle),
            camera.position.z - camera.target.z
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

void Visualizer::updateRobotTransform(const vsg_vec3& position, const vsg_quat& orientation) {
#ifndef USE_OPENGL_FALLBACK
    if (robotTransform) {
        // Convert quaternion to rotation matrix
        vsg::dquat q(orientation.x, orientation.y, orientation.z, orientation.w);
        vsg::dmat4 rotMatrix = vsg::rotate(q);
        
        // Create transform matrix: translation * rotation
        vsg::dmat4 transMatrix = vsg::translate(vsg::dvec3(position.x, position.y, position.z));
        robotTransform->matrix = transMatrix * rotMatrix;
        
        // Force VSG to update by marking as modified
        // In VSG, transforms are automatically updated when matrix changes
        
        // Debug output - every frame for the first 180 frames to see what's happening
        static int frameCount = 0;
        if (frameCount < 180 || frameCount % 60 == 0) {
            std::cout << "Frame " << frameCount << " - Robot transform updated:" << std::endl;
            std::cout << "  Position: (" << position.x << ", " << position.y << ", " << position.z << ")" << std::endl;
            std::cout << "  Orientation: (" << orientation.x << ", " << orientation.y << ", " << orientation.z << ", " << orientation.w << ")" << std::endl;
            std::cout << "  Transform ptr: " << robotTransform.get() << " Matrix ptr: " << &robotTransform->matrix << std::endl;
            
            // Print first few values of the matrix to verify it's being set
            auto& m = robotTransform->matrix;
            std::cout << "  Matrix[3]: (" << m[3][0] << ", " << m[3][1] << ", " << m[3][2] << ")" << std::endl;
        }
        frameCount++;
    } else {
        static bool warned = false;
        if (!warned) {
            std::cout << "WARNING: robotTransform is null in updateRobotTransform!" << std::endl;
            warned = true;
        }
    }
#endif
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
    
    // Set up shader set for proper rendering
    auto shaderSet = vsg::createPhongShaderSet(options);
    if (!shaderSet) {
        std::cerr << "Warning: Failed to create Phong shader set. Make sure VSG_FILE_PATH is set." << std::endl;
        // Try to create a basic shader set
        shaderSet = vsg::ShaderSet::create();
    }
    builder->shaderSet = shaderSet;
    
    auto scene = vsg::Group::create();
    
    // ------------------------------
    // Ground plane (quad lying on X-Y plane, Z-up)
    // ------------------------------
    vsg::GeometryInfo geomInfo;
    vsg::StateInfo stateInfo;
    
    geomInfo.position.set(0.0f, 0.0f, -0.01f);     // slightly below Z=0
    geomInfo.dx.set(10.0f, 0.0f, 0.0f);            // X extent
    geomInfo.dy.set(0.0f, 10.0f, 0.0f);            // Y extent
    geomInfo.color = vsg::vec4(0.7f, 0.7f, 0.7f, 1.0f); // Light gray
    
    // Ground appearance
    stateInfo.lighting = true;
    auto groundNode = builder->createQuad(geomInfo, stateInfo);
    scene->addChild(groundNode);

    // ------------------------------
    // Robot body (box) – properly sized hexapod body
    // ------------------------------
    const double bodyLength = 1.2;      // realistic hexapod body length
    const double bodyWidth  = 0.6;      // body width
    const double bodyHeight = 0.3;      // body height
    
    // Calculate initial Z position to match Robot.cpp physics initialization
    const double coxaLength = 0.25;
    const double femurLength = 0.35;
    const double tibiaLength = 0.4;
    const double totalLegLength = coxaLength + femurLength + tibiaLength;
    // Match physics initial position - body at 1.0 for safe settling
    const double bodyZ = 1.0;  // Matches Robot::createBody() initZ
    
    // Create robot transform that will be updated with physics position
    robotTransform = vsg::MatrixTransform::create();
    // Start at the expected physics position to ensure it's visible from the start
    robotTransform->matrix = vsg::translate(0.0, 0.0, bodyZ);
    
    std::cout << "Created robotTransform at initial Z=" << bodyZ << std::endl;
    std::cout << "Initial matrix translation: (" << robotTransform->matrix[3][0] << ", " 
              << robotTransform->matrix[3][1] << ", " << robotTransform->matrix[3][2] << ")" << std::endl;
    
    geomInfo.position.set(0.0f, 0.0f, 0.0f);  // Body at robot's local origin
    geomInfo.dx.set(static_cast<float>(bodyLength * 0.5), 0.0f, 0.0f);     // half-length
    geomInfo.dy.set(0.0f, static_cast<float>(bodyWidth * 0.5), 0.0f);      // half-width  
    geomInfo.dz.set(0.0f, 0.0f, static_cast<float>(bodyHeight * 0.5));     // half-height
    geomInfo.color = vsg::vec4(0.3f, 0.3f, 0.8f, 1.0f); // Blue-gray robot body
    
    // Create body state for proper material with bright color
    vsg::StateInfo bodyStateInfo;
    bodyStateInfo.lighting = true;
    bodyStateInfo.wireframe = false;
    bodyStateInfo.two_sided = true; // Render both sides in case of culling issues
    
    auto bodyNode = builder->createBox(geomInfo, bodyStateInfo);
    robotTransform->addChild(bodyNode);
    
    // Reset transform to identity after body creation
    geomInfo.transform = vsg::dmat4();

    // ------------------------------
    // Hexapod legs - proper 3-segment design with natural angles
    // ------------------------------
    // Using leg dimensions already defined above for consistency
    const double segmentRadius = 0.04;  // consistent segment thickness

    // Define leg and joint colors
    const auto legColor   = vsg::vec4(0.2f, 0.2f, 0.2f, 1.0f);
    const auto jointColor = vsg::vec4(0.15f, 0.15f, 0.15f, 1.0f);

    // Material states (lighting enabled, wireframe off)
    vsg::StateInfo legState;
    legState.lighting  = true;
    legState.wireframe = false;

    vsg::StateInfo jointState;
    jointState.lighting  = true;
    jointState.wireframe = false;
    
    // Leg attachment points - 3 pairs along body sides
    std::vector<double> legXPositions = { 
        -bodyLength * 0.3,   // front legs
         0.0,                // middle legs  
         bodyLength * 0.3    // rear legs
    };
    
    for (int legPair = 0; legPair < 3; ++legPair)
    {
        double legX = legXPositions[legPair];
        
        for (int side = -1; side <= 1; side += 2) // left (-1) and right (+1)
        {
            double sideSign = static_cast<double>(side);
            
            // Leg root positioned at body edge, slightly below center
            auto legRoot = vsg::MatrixTransform::create();
            double rootX = legX;
            double rootY = sideSign * (bodyWidth * 0.5);  // exactly at body edge
            double rootZ = -bodyHeight * 0.5;             // attach legs at body bottom
            legRoot->matrix = vsg::translate(rootX, rootY, rootZ);
            
            // Calculate leg segment orientations for natural hexapod stance
            // Using configuration for spider-like appearance
            double coxaAngle = sideSign * vsg::radians(HexapodVisualConfig::COXA_SPREAD_ANGLE);
            
            // ======= COXA (hip segment) =======
            // Calculate coxa direction: slightly upward and outward for natural stance
            double coxaUpAngle = vsg::radians(HexapodVisualConfig::COXA_ELEVATION);
            vsg::dvec3 coxaDir(
                cos(coxaAngle) * cos(coxaUpAngle),
                sin(coxaAngle) * cos(coxaUpAngle),
                sin(coxaUpAngle)  // Slight upward component
            );
            vsg::dvec3 coxaStart(0.0, 0.0, 0.0);
            vsg::dvec3 coxaEnd = coxaStart + coxaDir * coxaLength;
            vsg::dvec3 coxaCenter = (coxaStart + coxaEnd) * 0.5;
            
            // Calculate rotation from Z-axis to coxa direction
            vsg::dvec3 zAxis(0.0, 0.0, 1.0);
            vsg::dvec3 rotAxis = vsg::cross(zAxis, coxaDir);
            double rotAngle = acos(vsg::dot(zAxis, coxaDir));
            
            // Set up coxa geometry with proper transform
            geomInfo.position.set(0.0f, 0.0f, 0.0f);
            geomInfo.dx.set(static_cast<float>(segmentRadius), 0.0f, 0.0f);
            geomInfo.dy.set(0.0f, static_cast<float>(segmentRadius), 0.0f);
            geomInfo.dz.set(0.0f, 0.0f, static_cast<float>(coxaLength));
            geomInfo.color = legColor;
            geomInfo.transform = vsg::rotate(rotAngle, rotAxis);
            
            auto coxaNode = builder->createCylinder(geomInfo, legState);
            auto coxaXform = vsg::MatrixTransform::create();
            coxaXform->matrix = vsg::translate(coxaCenter);
            coxaXform->addChild(coxaNode);
            legRoot->addChild(coxaXform);
            
            // ======= FEMUR (thigh segment) =======
            // Femur extends mostly horizontally with natural curve
            double femurDownAngle = vsg::radians(HexapodVisualConfig::FEMUR_DOWN_ANGLE);
            // Leg-pair specific angles for natural spider-like pose
            double legPairOffset = 0.0;
            if (legPair == 0) legPairOffset = vsg::radians(HexapodVisualConfig::FRONT_LEG_FORWARD);
            else if (legPair == 1) legPairOffset = vsg::radians(HexapodVisualConfig::MIDDLE_LEG_OFFSET);
            else if (legPair == 2) legPairOffset = vsg::radians(HexapodVisualConfig::REAR_LEG_BACKWARD);
            double femurRadialAngle = coxaAngle + legPairOffset;
            vsg::dvec3 femurDir(
                cos(femurRadialAngle) * cos(femurDownAngle),  // X: radial direction * horizontal component
                sin(femurRadialAngle) * cos(femurDownAngle),  // Y: radial direction * horizontal component
                -sin(femurDownAngle)                           // Z: slight downward component
            );
            vsg::dvec3 femurStart = coxaEnd;
            vsg::dvec3 femurEnd = femurStart + femurDir * femurLength;
            vsg::dvec3 femurCenter = (femurStart + femurEnd) * 0.5;
            
            // Calculate rotation for femur
            rotAxis = vsg::cross(zAxis, femurDir);
            if (vsg::length(rotAxis) > 0.001) {
                rotAxis = vsg::normalize(rotAxis);
                rotAngle = acos(vsg::dot(zAxis, femurDir));
            } else {
                // Parallel or anti-parallel
                rotAxis = vsg::dvec3(1.0, 0.0, 0.0);
                rotAngle = (femurDir.z < 0) ? vsg::PI : 0.0;
            }
            
            // Set up femur geometry
            geomInfo.position.set(0.0f, 0.0f, 0.0f);
            geomInfo.dx.set(static_cast<float>(segmentRadius), 0.0f, 0.0f);
            geomInfo.dy.set(0.0f, static_cast<float>(segmentRadius), 0.0f);
            geomInfo.dz.set(0.0f, 0.0f, static_cast<float>(femurLength));
            geomInfo.color = legColor;
            geomInfo.transform = vsg::rotate(rotAngle, rotAxis);
            
            auto femurNode = builder->createCylinder(geomInfo, legState);
            auto femurXform = vsg::MatrixTransform::create();
            femurXform->matrix = vsg::translate(femurCenter);
            femurXform->addChild(femurNode);
            legRoot->addChild(femurXform);
            
            // ======= TIBIA (shin segment) =======
            // Tibia angles down steeply to reach ground
            double tibiaTotalDownAngle = vsg::radians(HexapodVisualConfig::TIBIA_DOWN_ANGLE);
            // Tibia maintains femur's radial direction but goes down steeply
            vsg::dvec3 tibiaDir(
                cos(femurRadialAngle) * cos(tibiaTotalDownAngle),  // X: same radial as femur
                sin(femurRadialAngle) * cos(tibiaTotalDownAngle),  // Y: same radial as femur
                -sin(tibiaTotalDownAngle)                           // Z: steep downward
            );
            vsg::dvec3 tibiaStart = femurEnd;
            vsg::dvec3 tibiaEnd = tibiaStart + tibiaDir * tibiaLength;
            vsg::dvec3 tibiaCenter = (tibiaStart + tibiaEnd) * 0.5;
            
            // Calculate rotation for tibia
            rotAxis = vsg::cross(zAxis, tibiaDir);
            if (vsg::length(rotAxis) > 0.001) {
                rotAxis = vsg::normalize(rotAxis);
                rotAngle = acos(vsg::dot(zAxis, tibiaDir));
            } else {
                rotAxis = vsg::dvec3(1.0, 0.0, 0.0);
                rotAngle = (tibiaDir.z < 0) ? vsg::PI : 0.0;
            }
            
            // Set up tibia geometry
            geomInfo.position.set(0.0f, 0.0f, 0.0f);
            geomInfo.dx.set(static_cast<float>(segmentRadius), 0.0f, 0.0f);
            geomInfo.dy.set(0.0f, static_cast<float>(segmentRadius), 0.0f);
            geomInfo.dz.set(0.0f, 0.0f, static_cast<float>(tibiaLength));
            geomInfo.color = legColor;
            geomInfo.transform = vsg::rotate(rotAngle, rotAxis);
            
            auto tibiaNode = builder->createCylinder(geomInfo, legState);
            auto tibiaXform = vsg::MatrixTransform::create();
            tibiaXform->matrix = vsg::translate(tibiaCenter);
            tibiaXform->addChild(tibiaNode);
            legRoot->addChild(tibiaXform);
            
            // ======= JOINT SPHERES for visual connection =======
            // Reset transform to identity for spheres
            geomInfo.transform = vsg::dmat4();
            
            // Smaller joint spheres
            float jointRadius = static_cast<float>(segmentRadius * 0.8);
            geomInfo.position.set(0.0f, 0.0f, 0.0f);
            geomInfo.dx.set(jointRadius, 0.0f, 0.0f);
            geomInfo.dy.set(0.0f, jointRadius, 0.0f);
            geomInfo.dz.set(0.0f, 0.0f, jointRadius);
            
            // Hip joint at leg root
            geomInfo.color = jointColor;
            auto hipJoint = builder->createSphere(geomInfo, jointState);
            legRoot->addChild(hipJoint);
            
            // Knee joint at coxa-femur connection
            auto kneeJointXform = vsg::MatrixTransform::create();
            kneeJointXform->matrix = vsg::translate(coxaEnd);
            geomInfo.color = jointColor;
            auto kneeJoint = builder->createSphere(geomInfo, jointState);
            kneeJointXform->addChild(kneeJoint);
            legRoot->addChild(kneeJointXform);
            
            // Ankle joint at femur-tibia connection
            auto ankleJointXform = vsg::MatrixTransform::create();
            ankleJointXform->matrix = vsg::translate(femurEnd);
            geomInfo.color = jointColor;
            auto ankleJoint = builder->createSphere(geomInfo, jointState);
            ankleJointXform->addChild(ankleJoint);
            legRoot->addChild(ankleJointXform);
            
            // Foot at end of tibia
            auto footXform = vsg::MatrixTransform::create();
            footXform->matrix = vsg::translate(tibiaEnd);
            geomInfo.color = vsg::vec4(0.8f, 0.2f, 0.2f, 1.0f); // Red foot
            float footRadius = static_cast<float>(segmentRadius * 2.0);
            geomInfo.dx.set(footRadius, 0.0f, 0.0f);
            geomInfo.dy.set(0.0f, footRadius, 0.0f);
            geomInfo.dz.set(0.0f, 0.0f, footRadius);
            auto foot = builder->createSphere(geomInfo, jointState);
            footXform->addChild(foot);
            legRoot->addChild(footXform);
            
            robotTransform->addChild(legRoot);
            
#ifdef DEBUG_VISUALIZER
            // Debug output for leg geometry verification
            int legIdx = legPair * 2 + (side == -1 ? 0 : 1);
            std::cout << "Leg " << legIdx << " visual geometry:" << std::endl;
            std::cout << "  Root (world): (" << rootX << ", " << rootY << ", " << bodyZ + rootZ << ")" << std::endl;
            std::cout << "  Angles: coxa=" << vsg::degrees(coxaAngle) 
                      << "°, femur down=" << vsg::degrees(femurDownAngle) 
                      << "°, tibia down=" << vsg::degrees(tibiaTotalDownAngle) << "°" << std::endl;
            std::cout << "  Segment ends (local to root):" << std::endl;
            std::cout << "    Coxa: " << coxaEnd << std::endl;
            std::cout << "    Femur: " << femurEnd << std::endl;
            std::cout << "    Tibia: " << tibiaEnd << std::endl;
            std::cout << "  Foot height (world): " << (bodyZ + rootZ + tibiaEnd.z) << std::endl;
            
            // Verify direction vectors are normalized
            std::cout << "  Direction vector magnitudes:" << std::endl;
            std::cout << "    Coxa dir: " << vsg::length(coxaDir) << " (should be 1.0)" << std::endl;
            std::cout << "    Femur dir: " << vsg::length(femurDir) << " (should be 1.0)" << std::endl;
            std::cout << "    Tibia dir: " << vsg::length(tibiaDir) << " (should be 1.0)" << std::endl;
            
            // Verify segment connections
            double coxaGap = vsg::length(femurStart - coxaEnd);
            double femurGap = vsg::length(tibiaStart - femurEnd);
            std::cout << "  Segment gaps (should be ~0):" << std::endl;
            std::cout << "    Coxa-Femur gap: " << coxaGap << std::endl;
            std::cout << "    Femur-Tibia gap: " << femurGap << std::endl;
#endif
        }
    }
    
    // Add the complete robot (body + legs) to the scene
    scene->addChild(robotTransform);
    
    // NOTE: Architectural Issue - Visual-Physics Transform Sync
    // The Robot.cpp creates placeholder transforms (legNodes) expecting to update them,
    // but this Visualizer creates static geometry that doesn't reference those transforms.
    // For proper dynamic simulation, the visual leg segments should reference the
    // transforms managed by Robot.cpp, not create independent static geometry.
    // Current implementation creates visually correct but static hexapod geometry.
    
    // Debug: Add coordinate axes at origin
    auto axesGroup = vsg::Group::create();
    
    // X-axis (red)
    geomInfo.position.set(0.5f, 0.0f, 0.0f);
    geomInfo.dx.set(0.5f, 0.0f, 0.0f);
    geomInfo.dy.set(0.0f, 0.02f, 0.0f);
    geomInfo.dz.set(0.0f, 0.0f, 0.02f);
    geomInfo.color = vsg::vec4(1.0f, 0.0f, 0.0f, 1.0f);
    axesGroup->addChild(builder->createBox(geomInfo, bodyStateInfo));
    
    // Y-axis (green)
    geomInfo.position.set(0.0f, 0.5f, 0.0f);
    geomInfo.dx.set(0.02f, 0.0f, 0.0f);
    geomInfo.dy.set(0.0f, 0.5f, 0.0f);
    geomInfo.dz.set(0.0f, 0.0f, 0.02f);
    geomInfo.color = vsg::vec4(0.0f, 1.0f, 0.0f, 1.0f);
    axesGroup->addChild(builder->createBox(geomInfo, bodyStateInfo));
    
    // Z-axis (blue)
    geomInfo.position.set(0.0f, 0.0f, 0.5f);
    geomInfo.dx.set(0.02f, 0.0f, 0.0f);
    geomInfo.dy.set(0.0f, 0.02f, 0.0f);
    geomInfo.dz.set(0.0f, 0.0f, 0.5f);
    geomInfo.color = vsg::vec4(0.0f, 0.0f, 1.0f, 1.0f);
    axesGroup->addChild(builder->createBox(geomInfo, bodyStateInfo));
    
    scene->addChild(axesGroup);
    
    // Debug output to verify robot visual was created
    std::cout << "Robot visual created:" << std::endl;
    std::cout << "  robotTransform ptr: " << robotTransform.get() << std::endl;
    std::cout << "  robotTransform children: " << robotTransform->children.size() << std::endl;
    std::cout << "  Expected initial physics Z: " << bodyZ << std::endl;
    std::cout << "  Scene children: " << scene->children.size() << std::endl;
    
    // Verify the robot is actually in the scene
    bool foundRobot = false;
    for (auto& child : scene->children) {
        if (child == robotTransform) {
            foundRobot = true;
            std::cout << "  ✓ Robot transform found in scene!" << std::endl;
            break;
        }
    }
    if (!foundRobot) {
        std::cout << "  ✗ ERROR: Robot transform NOT found in scene!" << std::endl;
    }
    
    // Add several test objects to verify visibility
    // Test 1: Large bright sphere at origin for visibility test
    auto test1 = vsg::MatrixTransform::create();
    test1->matrix = vsg::translate(0.0, 0.0, 2.0); // Above ground
    geomInfo.position.set(0.0f, 0.0f, 0.0f);
    geomInfo.dx.set(1.0f, 0.0f, 0.0f);
    geomInfo.dy.set(0.0f, 1.0f, 0.0f);
    geomInfo.dz.set(0.0f, 0.0f, 1.0f);
    geomInfo.color = vsg::vec4(1.0f, 1.0f, 0.0f, 1.0f); // Bright yellow
    auto sphere1 = builder->createSphere(geomInfo, bodyStateInfo);
    test1->addChild(sphere1);
    scene->addChild(test1);
    
    std::cout << "Added large yellow test sphere at (0, 0, 2)" << std::endl;
    
    // Test 2: Static box at robot height
    auto test2 = vsg::MatrixTransform::create();
    test2->matrix = vsg::translate(-2.0, 0.0, bodyZ);
    geomInfo.position.set(0.0f, 0.0f, 0.0f);
    geomInfo.dx.set(0.5f, 0.0f, 0.0f);
    geomInfo.dy.set(0.0f, 0.5f, 0.0f);
    geomInfo.dz.set(0.0f, 0.0f, 0.5f);
    geomInfo.color = vsg::vec4(0.0f, 1.0f, 0.0f, 1.0f); // Green
    auto box2 = builder->createBox(geomInfo, bodyStateInfo);
    test2->addChild(box2);
    scene->addChild(test2);
    
    std::cout << "Added test objects: Red sphere at (2,0,0.5), Green box at (-2,0," << bodyZ << ")" << std::endl;
    std::cout << "Robot visual model should be at (0,0," << bodyZ << ")" << std::endl;
    std::cout << "RobotTransform pointer: " << robotTransform.get() << std::endl;
    
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