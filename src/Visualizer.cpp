#include "Visualizer.h"
#include "VisualizerHexapodConfig.h"
#include "DebugOutput.h"
#include "NoiseManager.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>

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

// Internal InputHandler removed - using external InputHandler from main.cpp

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
        // macOS requires forward compatibility for OpenGL 3.2+
#ifdef __APPLE__
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#else
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);
#endif
        
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
        windowTraits->fullscreen = false;
        
        // Window traits configured
        
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
        // DISABLED: Trackball might be intercepting key events
        // viewer->addEventHandler(vsg::Trackball::create(vsgCamera));
        
        // Input handler is now added externally from main.cpp
        
        // 9. Create command graph using modern simplified approach
        auto commandGraph = vsg::createCommandGraphForView(window, vsgCamera, sceneRoot);
        viewer->assignRecordAndSubmitTaskAndPresentation({commandGraph});
        
        // 10. Load font and create text overlay
        textOptions = vsg::Options::create();
        textOptions->paths = options->paths;
        
        auto fontPath = vsg::findFile("fonts/times.vsgb", options->paths);
        if (fontPath.empty()) {
            // Try common font locations
            fontPath = "vsgExamples/data/fonts/times.vsgb";
        }
        
        font = vsg::read_cast<vsg::Font>(fontPath, textOptions);
        if (font) {
            createTextOverlay();
            std::cout << "Text overlay initialized" << std::endl;
        } else {
            std::cout << "Warning: Could not load font from " << fontPath << std::endl;
        }
        
        // 11. Compile everything
        viewer->compile();
        
        std::cout << "VSG initialization complete with modern API patterns" << std::endl;
        std::cout << "Window extent: " << window->extent2D().width << "x" << window->extent2D().height << std::endl;
        std::cout << "Viewer active: " << viewer->active() << std::endl;
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
    
#ifndef USE_OPENGL_FALLBACK
    // Update text visibility by modifying the color alpha
    if (statsText && statsText->layout) {
        if (auto layout = statsText->layout.cast<vsg::StandardLayout>()) {
            layout->color.a = showStatistics ? 0.9f : 0.0f;
            statsText->setup();
        }
    }
    if (controlsText && controlsText->layout) {
        if (auto layout = controlsText->layout.cast<vsg::StandardLayout>()) {
            layout->color.a = showControlsOverlay ? 0.9f : 0.0f;
            controlsText->setup();
        }
    }
#endif
    
    // Frame
    static int debugFrameCount = 0;
    static auto lastDebugTime = std::chrono::high_resolution_clock::now();
    
    auto beforeAdvance = std::chrono::high_resolution_clock::now();
    bool advanced = viewer->advanceToNextFrame();
    auto afterAdvance = std::chrono::high_resolution_clock::now();
    
    double advanceTime = std::chrono::duration<double>(afterAdvance - beforeAdvance).count();
    if (advanceTime > 0.1) {  // Log if it takes more than 100ms
        std::cout << "[VSG] advanceToNextFrame took " << advanceTime << "s" << std::endl;
    }
    
    if (advanced) {
        viewer->handleEvents();
        
        // IMPORTANT: viewer->update() must be called AFTER all transforms are updated
        // The transforms should be updated before this render() call
        viewer->update();
        
        viewer->recordAndSubmit();
        viewer->present();
        
        debugFrameCount++;
        if (debugFrameCount % 10 == 0) {
            auto now = std::chrono::high_resolution_clock::now();
            double elapsed = std::chrono::duration<double>(now - lastDebugTime).count();
            std::cout << "[VSG RENDER] " << debugFrameCount << " frames, " 
                      << (10.0/elapsed) << " FPS" << std::endl;
            lastDebugTime = now;
        }
    } else {
        static bool warnedOnce = false;
        if (!warnedOnce) {
            std::cout << "WARNING: viewer->advanceToNextFrame() returned false!" << std::endl;
            std::cout << "Viewer active: " << viewer->active() << std::endl;
            warnedOnce = true;
        }
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

void Visualizer::close() {
#ifdef USE_OPENGL_FALLBACK
    if (window) {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    }
#else
    if (viewer) {
        viewer->close();
    }
#endif
}

#ifndef USE_OPENGL_FALLBACK
void Visualizer::addEventHandler(vsg::ref_ptr<vsg::Visitor> handler) {
    if (viewer && handler) {
        viewer->addEventHandler(handler);
    }
}
#endif

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
    if (currentCameraMode == CameraMode::FREE) {
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

    // Camera mode handling
#ifndef USE_OPENGL_FALLBACK
    if (!lookAt) return;
    
    // Get current robot position (stored from last updateRobotTransform)
    static vsg_vec3 lastRobotPos(0, 0, 1);
    if (robotTransform) {
        // Extract position from transform matrix
        auto matrix = robotTransform->matrix;
        lastRobotPos = vsg_vec3(matrix[3][0], matrix[3][1], matrix[3][2]);
    }
    
    if (currentCameraMode == CameraMode::FOLLOW) {
        // Follow robot from behind and above
        vsg::dvec3 robotPos(lastRobotPos.x, lastRobotPos.y, lastRobotPos.z);
        
        // Clamp robot position to reasonable bounds
        robotPos.z = std::clamp(robotPos.z, 0.5, 5.0);
        
        vsg::dvec3 offset(-camera.followDistance, 0, camera.followHeight);
        vsg::dvec3 eyePos = robotPos + offset;
        
        // Ensure camera doesn't go below ground
        eyePos.z = std::max(eyePos.z, 1.0);
        
        lookAt->eye = eyePos;
        lookAt->center = robotPos;
        lookAt->up = vsg::dvec3(0.0, 0.0, 1.0);
    }
    else if (currentCameraMode == CameraMode::ORBIT) {
        // Orbit around robot
        static double orbitAngle = 0.0;
        orbitAngle += 0.01; // Slow rotation
        
        vsg::dvec3 robotPos(lastRobotPos.x, lastRobotPos.y, lastRobotPos.z);
        double radius = camera.followDistance;
        lookAt->eye = robotPos + vsg::dvec3(
            radius * cos(orbitAngle),
            radius * sin(orbitAngle),
            camera.followHeight
        );
        lookAt->center = robotPos;
        lookAt->up = vsg::dvec3(0.0, 0.0, 1.0);
    }
    else if (currentCameraMode == CameraMode::TOP) {
        // Top-down view
        vsg::dvec3 robotPos(lastRobotPos.x, lastRobotPos.y, lastRobotPos.z);
        
        // Clamp robot position
        robotPos.z = std::clamp(robotPos.z, 0.5, 5.0);
        
        lookAt->eye = robotPos + vsg::dvec3(0.0, 0.0, 20.0);
        lookAt->center = robotPos;
        lookAt->up = vsg::dvec3(0.0, 1.0, 0.0);
    }
    else if (currentCameraMode == CameraMode::FRONT) {
        // Front view
        vsg::dvec3 robotPos(lastRobotPos.x, lastRobotPos.y, lastRobotPos.z);
        lookAt->eye = robotPos + vsg::dvec3(0.0, -15.0, 5.0);
        lookAt->center = robotPos;
        lookAt->up = vsg::dvec3(0.0, 0.0, 1.0);
    }
    else if (currentCameraMode == CameraMode::SIDE) {
        // Side view
        vsg::dvec3 robotPos(lastRobotPos.x, lastRobotPos.y, lastRobotPos.z);
        lookAt->eye = robotPos + vsg::dvec3(15.0, 0.0, 5.0);
        lookAt->center = robotPos;
        lookAt->up = vsg::dvec3(0.0, 0.0, 1.0);
    }
    // "free" mode uses trackball and doesn't update automatically
#else
    // Follow mode
    if (currentCameraMode == CameraMode::FOLLOW) {
        vsg_vec3 offset(
            -camera.followDistance * cos(frameTime * 0.1f),
            -camera.followDistance * sin(frameTime * 0.1f),
            camera.followHeight
        );
        
        camera.position = camera.target + offset;
    }
    
    // Orbit mode  
    else if (currentCameraMode == CameraMode::ORBIT) {
        float angle = frameTime * 0.2f;
        vsg_vec3 diff = camera.position - camera.target;
        float radius = diff.length();
        
        camera.position = camera.target + vsg_vec3(
            radius * cos(angle),
            radius * sin(angle),
            camera.position.z - camera.target.z
        );
    }
#endif
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
    
    if (showStatistics) {
        renderTime = frameTime * 1000.0; // Convert to milliseconds
        double fps = frameTime > 0.0 ? 1.0 / frameTime : 0.0;
        
        // Update stats text overlay
        std::stringstream stats;
        stats << "FPS: " << std::fixed << std::setprecision(1) << fps << "\n";
        stats << "Frame time: " << std::fixed << std::setprecision(2) << renderTime << " ms\n";
        stats << "Camera: " << getCameraModeString() << "\n";
        if (manualControlEnabled) {
            stats << "Manual Control: ACTIVE\n";
            stats << "Velocity: (" << std::fixed << std::setprecision(2) 
                  << manualControlVelocity.x << ", " 
                  << manualControlVelocity.y << ", "
                  << manualControlVelocity.z << ")";
        } else {
            stats << "Manual Control: OFF";
        }
        
        // Update text overlay would go here
        // Currently text updates are handled from main.cpp
        
        if (frameCount % 60 == 0) {
            std::cout << "Frame: " << frameCount << ", FPS: " << fps << std::endl;
        }
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
        
        // Debug output disabled - use V key to enable debug mode if needed
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
    
    geomInfo.position.set(0.0f, 0.0f, 0.0f);       // exactly at Z=0 to match physics
    geomInfo.dx.set(50.0f, 0.0f, 0.0f);            // X extent - increased from 10
    geomInfo.dy.set(0.0f, 50.0f, 0.0f);            // Y extent - increased from 10
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
    
    // Add the robot body transform to the scene (legs will be added dynamically)
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

// Input handling implementation
void Visualizer::handleKeyPress(int key) {
    // Update manual control velocity based on key presses
    if (manualControlEnabled) {
        const float speed = 2.0f;
        
        // Reset velocity
        manualControlVelocity = vsg_vec3(0.0f, 0.0f, 0.0f);
        
        // Forward/backward
        if (keyStates['w'] || keyStates['W']) manualControlVelocity.x += speed;
        if (keyStates['s'] || keyStates['S']) manualControlVelocity.x -= speed;
        
        // Left/right
        if (keyStates['a'] || keyStates['A']) manualControlVelocity.y += speed;
        if (keyStates['d'] || keyStates['D']) manualControlVelocity.y -= speed;
        
        // Up/down (for testing)
        if (keyStates['q'] || keyStates['Q']) manualControlVelocity.z -= speed * 0.5f;
        if (keyStates['e'] || keyStates['E']) manualControlVelocity.z += speed * 0.5f;
    }
}

void Visualizer::handleKeyRelease(int key) {
    // Update velocity when keys are released
    handleKeyPress(key); // Recalculate velocity
}

void Visualizer::setCameraMode(CameraMode mode) {
    currentCameraMode = mode;
    
    // Reset camera parameters based on mode
    switch (mode) {
        case CameraMode::FOLLOW:
            camera.followRobot = true;
            camera.followDistance = 5.0f;
            camera.followHeight = 3.0f;
            break;
            
        case CameraMode::FREE:
            camera.followRobot = false;
            // Keep current position
            break;
            
        case CameraMode::ORBIT:
            camera.followRobot = true;
            camera.followDistance = 8.0f;
            camera.followHeight = 4.0f;
            camera.orbitAngle = 0.0f;
            break;
            
        case CameraMode::TOP:
            camera.followRobot = true;
            camera.position = robotPosition + vsg_vec3(0, 0, 10);
            camera.target = robotPosition;
            camera.up = vsg_vec3(0, 1, 0);
            break;
            
        case CameraMode::FRONT:
            camera.followRobot = true;
            camera.position = robotPosition + vsg_vec3(8, 0, 2);
            camera.target = robotPosition;
            break;
            
        case CameraMode::SIDE:
            camera.followRobot = true;
            camera.position = robotPosition + vsg_vec3(0, 8, 2);
            camera.target = robotPosition;
            break;
    }
    
    std::cout << "Camera mode: " << getCameraModeString() << std::endl;
}

void Visualizer::setCameraMode(const std::string& mode) {
    if (mode == "follow") setCameraMode(CameraMode::FOLLOW);
    else if (mode == "free") setCameraMode(CameraMode::FREE);
    else if (mode == "orbit") setCameraMode(CameraMode::ORBIT);
    else if (mode == "top") setCameraMode(CameraMode::TOP);
    else if (mode == "front") setCameraMode(CameraMode::FRONT);
    else if (mode == "side") setCameraMode(CameraMode::SIDE);
}

void Visualizer::cycleCamera() {
    int nextMode = (static_cast<int>(currentCameraMode) + 1) % 6;
    setCameraMode(static_cast<CameraMode>(nextMode));
}

void Visualizer::adjustCameraDistance(float delta) {
    if (currentCameraMode == CameraMode::FREE || 
        currentCameraMode == CameraMode::FOLLOW || 
        currentCameraMode == CameraMode::ORBIT) {
        camera.followDistance = std::clamp(camera.followDistance + delta, 2.0f, 20.0f);
        
        // Also adjust free camera distance
        if (currentCameraMode == CameraMode::FREE) {
            vsg_vec3 dir = camera.position - camera.target;
            float currentDist = vsg::length(dir);
            if (currentDist > 0.1f) {
                dir = vsg::normalize(dir);
                camera.position = camera.target + dir * (currentDist + delta);
            }
        }
    }
}

#ifndef USE_OPENGL_FALLBACK
void Visualizer::createTextOverlay() {
    if (!font || !scene) return;
    
    // Create text group for overlay
    textGroup = vsg::Group::create();
    
    // Create stats text with transform for positioning (top-right)
    {
        statsTransform = vsg::MatrixTransform::create();
        statsTransform->matrix = vsg::translate(vsg::dvec3(windowWidth - 280.0, 20.0, 0.0));
        
        auto layout = vsg::StandardLayout::create();
        layout->position = vsg::vec3(0.0f, 0.0f, 0.0f);
        layout->horizontal = vsg::vec3(14.0f, 0.0f, 0.0f);
        layout->vertical = vsg::vec3(0.0f, 18.0f, 0.0f);
        layout->color = vsg::vec4(0.9f, 0.9f, 0.9f, 0.85f);
        layout->outlineWidth = 0.2f;
        layout->outlineColor = vsg::vec4(0.1f, 0.1f, 0.1f, 0.8f);
        
        statsText = vsg::Text::create();
        statsText->text = vsg::stringValue::create("Initializing...");
        statsText->font = font;
        statsText->layout = layout;
        statsText->technique = vsg::GpuLayoutTechnique::create();
        statsText->setup(1024); // Allocate space for dynamic updates
        
        statsTransform->addChild(statsText);
        textGroup->addChild(statsTransform);
    }
    
    // Create controls text with transform
    {
        controlsTransform = vsg::MatrixTransform::create();
        controlsTransform->matrix = vsg::translate(vsg::dvec3(windowWidth - 300.0, 20.0, 0.0));
        
        auto layout = vsg::StandardLayout::create();
        layout->position = vsg::vec3(0.0f, 0.0f, 0.0f);
        layout->horizontal = vsg::vec3(14.0f, 0.0f, 0.0f);
        layout->vertical = vsg::vec3(0.0f, 18.0f, 0.0f);
        layout->color = vsg::vec4(0.9f, 0.9f, 1.0f, 0.9f);
        layout->outlineWidth = 0.15f;
        layout->outlineColor = vsg::vec4(0.0f, 0.0f, 0.0f, 1.0f);
        
        controlsText = vsg::Text::create();
        controlsText->text = vsg::stringValue::create(
            "=== CONTROLS ===\n"
            "WASD - Move robot\n"
            "Q/E  - Up/Down\n"
            "Alt+A/D - Strafe\n"
            "Shift - Sprint\n"
            "Space - Manual control\n"
            "C - Camera mode\n"
            "V - Debug verbosity\n"
            "N/M - Increase/Decrease noise\n"
            "H - Toggle help\n"
            "1/2/3 - AI modes\n"
            "R - Reset robot\n"
            "F1 - Stats\n"
            "F2 - Shadows\n"
            "F3 - SSAO\n"
            "ESC - Exit"
        );
        controlsText->font = font;
        controlsText->layout = layout;
        controlsText->technique = vsg::GpuLayoutTechnique::create();
        controlsText->setup(0, textOptions);
        
        controlsTransform->addChild(controlsText);
        textGroup->addChild(controlsTransform);
    }
    
    // Initially hide controls
    controlsTransform->children.clear();
    
    // Add text group to scene
    scene->addChild(textGroup);
    
    // Make stats visible by default
    statsVisible = true;
}

void Visualizer::updateStatsText(float fps, float frameTime, const vsg_vec3& robotPos, 
                                const vsg_vec3& robotVel, bool stable, const std::string& controlMode,
                                const vsg_vec3& moveCommand, float rotCommand,
                                int footContacts, const vsg_vec3& angularVelocity, float avgJointVel) {
    if (!statsText || !statsText->text) return;
    
    if (auto text_string = statsText->text.cast<vsg::stringValue>()) {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(1);
        ss << "=== ROBOT STATUS ===" << std::endl;
        ss << "FPS: " << fps << " (" << frameTime << "ms)" << std::endl;
        ss << std::endl;
        
        ss << "Position: (" << robotPos.x << ", " << robotPos.y << ", " << robotPos.z << ")" << std::endl;
        ss << "Velocity: " << vsg::length(robotVel) << " m/s" << std::endl;
        ss << "Stable: " << (stable ? "YES" : "NO") << std::endl;
        ss << std::endl;
        
        ss << "Control: " << controlMode << std::endl;
        if (controlMode.find("MANUAL") != std::string::npos) {
            ss << "Move Cmd: ";
            if (moveCommand.x != 0 || moveCommand.y != 0 || rotCommand != 0) {
                ss << "Fwd=" << moveCommand.x << " Turn=" << rotCommand;
            } else {
                ss << "None";
            }
            ss << std::endl;
        }
        ss << std::endl;
        
        ss << "=== SENSORS ===" << std::endl;
        ss << "Foot Contacts: " << footContacts << "/6" << std::endl;
        
        // Add ground collision warning
        if (robotPos.z < 0.2f) {
            ss << "*** GROUND COLLISION ***" << std::endl;
            ss << "Body Height: " << robotPos.z << "m (TOO LOW!)" << std::endl;
        }
        
        ss << "Angular Vel: (" << std::setprecision(2) 
           << angularVelocity.x << ", " << angularVelocity.y << ", " << angularVelocity.z << ")" << std::endl;
        ss << "Avg Joint Vel: " << avgJointVel << " rad/s" << std::endl;
        ss << std::endl;
        
        ss << std::setprecision(1);  // Reset precision
        ss << "Camera: " << getCameraModeString() << std::endl;
        
        // Add debug level
        std::string debugLevel = "NONE";
        switch(Debug::currentLevel) {
            case Debug::NONE: debugLevel = "NONE"; break;
            case Debug::ERROR: debugLevel = "ERROR"; break;
            case Debug::WARNING: debugLevel = "WARNING"; break;
            case Debug::INFO: debugLevel = "INFO"; break;
            case Debug::VERBOSE: debugLevel = "VERBOSE"; break;
            case Debug::ALL: debugLevel = "ALL"; break;
        }
        ss << "Debug: " << debugLevel << std::endl;
        
        // Add noise level
        float noiseLevel = NoiseManager::getInstance().getNoiseLevel();
        ss << "Noise Level: " << std::fixed << std::setprecision(1) << noiseLevel << std::endl;
        
        text_string->value() = ss.str();
        statsText->setup();
    }
}

void Visualizer::updateControlsText(bool visible) {
    if (!controlsTransform) return;
    
    if (visible && controlsTransform->children.empty()) {
        controlsTransform->addChild(controlsText);
    } else if (!visible && !controlsTransform->children.empty()) {
        controlsTransform->children.clear();
    }
}

void Visualizer::setTextVisible(bool stats, bool controls) {
    statsVisible = stats;
    controlsVisible = controls;
    
    if (statsTransform) {
        if (stats && statsTransform->children.empty()) {
            statsTransform->addChild(statsText);
        } else if (!stats && !statsTransform->children.empty()) {
            statsTransform->children.clear();
        }
    }
    
    updateControlsText(controls);
}

std::string Visualizer::getCameraModeString() const {
    switch (currentCameraMode) {
        case CameraMode::FOLLOW: return "Follow";
        case CameraMode::FREE: return "Free";
        case CameraMode::ORBIT: return "Orbit";
        case CameraMode::TOP: return "Top";
        case CameraMode::FRONT: return "Front";
        case CameraMode::SIDE: return "Side";
        default: return "Unknown";
    }
}
#else
// Stub implementations for OpenGL fallback
void Visualizer::createTextOverlay() {}

void Visualizer::updateStatsText(float fps, float frameTime, const vsg_vec3& robotPos, 
                                 const vsg_vec3& robotVel, bool manualControl, 
                                 const std::string& controlMode, const vsg_vec3& cameraPos, 
                                 float renderTime, int triangleCount, const vsg_vec3& lightDir, 
                                 float noiseLevel) {
    // Text overlay not implemented in OpenGL fallback
}

void Visualizer::updateControlsText(bool visible) {
    // Text overlay not implemented in OpenGL fallback
}

void Visualizer::setTextVisible(bool stats, bool controls) {
    // Text overlay not implemented in OpenGL fallback
}

std::string Visualizer::getCameraModeString() const {
    switch (currentCameraMode) {
        case CameraMode::FOLLOW: return "Follow";
        case CameraMode::FREE: return "Free";
        case CameraMode::ORBIT: return "Orbit";
        case CameraMode::TOP: return "Top";
        case CameraMode::FRONT: return "Front";
        case CameraMode::SIDE: return "Side";
        default: return "Unknown";
    }
}
#endif

void Visualizer::addSkybox(const std::string& skyboxPath) {
#ifndef USE_OPENGL_FALLBACK
    if (!sceneRoot) return;
    
    try {
        // Load skybox texture (typically a cube map)
        auto skyboxData = vsg::read_cast<vsg::Data>(skyboxPath, textOptions);
        if (!skyboxData) {
            std::cout << "Warning: Could not load skybox from " << skyboxPath << std::endl;
            return;
        }
        
        // Create skybox geometry (large inverted cube)
        auto builder = vsg::Builder::create();
        builder->options = textOptions;
        
        vsg::GeometryInfo geomInfo;
        geomInfo.position.set(0.0f, 0.0f, 0.0f);
        geomInfo.dx.set(500.0f, 0.0f, 0.0f);  // Very large
        geomInfo.dy.set(0.0f, 500.0f, 0.0f);
        geomInfo.dz.set(0.0f, 0.0f, 500.0f);
        
        vsg::StateInfo stateInfo;
        stateInfo.image = skyboxData;
        stateInfo.two_sided = true;  // Render both sides
        stateInfo.lighting = false;  // No lighting on skybox
        
        auto skybox = builder->createBox(geomInfo, stateInfo);
        
        // Make it render first (background)
        auto absoluteTransform = vsg::AbsoluteTransform::create();
        absoluteTransform->addChild(skybox);
        
        sceneRoot->addChild(absoluteTransform);
        std::cout << "Skybox added successfully" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Failed to add skybox: " << e.what() << std::endl;
    }
#else
    // OpenGL fallback - could implement simple gradient background
    std::cout << "Skybox not supported in OpenGL fallback mode" << std::endl;
#endif
}

void Visualizer::setEnvironmentLighting(float intensity, const vsg_vec3& direction) {
#ifndef USE_OPENGL_FALLBACK
    if (!sceneRoot) return;
    
    // Add or update environment lighting
    auto envLight = vsg::DirectionalLight::create();
    envLight->name = "environment";
    envLight->color.set(0.9f, 0.9f, 1.0f); // Slightly blue tint
    envLight->intensity = intensity;
    envLight->direction = vsg::normalize(vsg::vec3(direction.x, direction.y, direction.z));
    
    sceneRoot->addChild(envLight);
#endif
}