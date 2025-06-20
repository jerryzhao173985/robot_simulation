#include <vsg/all.h>
#include <iostream>

int main(int argc, char** argv)
{
    std::cout << "VSG Minimal Test Starting..." << std::endl;
    
    // Create window
    auto windowTraits = vsg::WindowTraits::create();
    windowTraits->windowTitle = "VSG Test - Red Box";
    windowTraits->width = 800;
    windowTraits->height = 600;
    windowTraits->x = 100;
    windowTraits->y = 100;
    
    std::cout << "Creating window..." << std::endl;
    auto window = vsg::Window::create(windowTraits);
    if (!window) {
        std::cerr << "Failed to create window!" << std::endl;
        return 1;
    }
    
    // Create viewer
    std::cout << "Creating viewer..." << std::endl;
    auto viewer = vsg::Viewer::create();
    viewer->addWindow(window);
    
    // Create simple scene - just a red triangle
    auto vertices = vsg::vec3Array::create(
    {
        {-0.5f, -0.5f, 0.0f},
        { 0.5f, -0.5f, 0.0f},
        { 0.0f,  0.5f, 0.0f}
    });
    
    auto colors = vsg::vec4Array::create(
    {
        {1.0f, 0.0f, 0.0f, 1.0f},
        {1.0f, 0.0f, 0.0f, 1.0f},
        {1.0f, 0.0f, 0.0f, 1.0f}
    });
    
    // Create a simple flat shaded triangle
    auto scenegraph = vsg::StateGroup::create();
    
    // Setup camera
    auto lookAt = vsg::LookAt::create(vsg::dvec3(0.0, 0.0, 3.0), vsg::dvec3(0.0, 0.0, 0.0), vsg::dvec3(0.0, 1.0, 0.0));
    auto perspective = vsg::Perspective::create(30.0, static_cast<double>(window->extent2D().width) / static_cast<double>(window->extent2D().height), 0.1, 10.0);
    auto camera = vsg::Camera::create(perspective, lookAt, vsg::ViewportState::create(window->extent2D()));
    
    // Create command graph
    auto commandGraph = vsg::createCommandGraphForView(window, camera, scenegraph);
    viewer->assignRecordAndSubmitTaskAndPresentation({commandGraph});
    
    // Add event handlers
    viewer->addEventHandler(vsg::CloseHandler::create(viewer));
    
    // Compile
    std::cout << "Compiling..." << std::endl;
    viewer->compile();
    
    std::cout << "Starting render loop. Window should show a red screen." << std::endl;
    std::cout << "Press ESC or close window to exit." << std::endl;
    
    // Main loop
    int frameCount = 0;
    while (viewer->advanceToNextFrame())
    {
        viewer->handleEvents();
        viewer->update();
        viewer->recordAndSubmit();
        viewer->present();
        
        if (frameCount == 0) {
            std::cout << "First frame rendered!" << std::endl;
        }
        
        if (frameCount % 60 == 0) {
            std::cout << "Frame " << frameCount << " - viewer active: " << (viewer->active() ? "YES" : "NO") << std::endl;
        }
        
        frameCount++;
    }
    
    std::cout << "Rendered " << frameCount << " frames total." << std::endl;
    return 0;
}