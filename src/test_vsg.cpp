#include <vsg/all.h>
#include <vsgXchange/all.h>
#include <iostream>

int main(int argc, char** argv)
{
    // Setup exactly like vsgbuilder example
    auto options = vsg::Options::create();
    options->paths = vsg::getEnvPaths("VSG_FILE_PATH");
    options->sharedObjects = vsg::SharedObjects::create();
    options->add(vsgXchange::all::create());

    auto windowTraits = vsg::WindowTraits::create();
    windowTraits->windowTitle = "VSG Test";
    windowTraits->width = 800;
    windowTraits->height = 600;

    auto builder = vsg::Builder::create();
    builder->options = options;

    // Create simple scene
    auto scene = vsg::Group::create();
    
    vsg::GeometryInfo geomInfo;
    vsg::StateInfo stateInfo;
    stateInfo.lighting = true;
    
    // Red box at origin
    geomInfo.position.set(0.0f, 0.0f, 0.0f);
    geomInfo.dx.set(1.0f, 0.0f, 0.0f);
    geomInfo.dy.set(0.0f, 1.0f, 0.0f);
    geomInfo.dz.set(0.0f, 0.0f, 1.0f);
    geomInfo.color = vsg::vec4(1.0f, 0.0f, 0.0f, 1.0f);
    
    auto box = builder->createBox(geomInfo, stateInfo);
    if (box) {
        std::cout << "Box created successfully!" << std::endl;
        scene->addChild(box);
    } else {
        std::cout << "Failed to create box!" << std::endl;
    }
    
    // Create viewer
    auto viewer = vsg::Viewer::create();
    auto window = vsg::Window::create(windowTraits);
    if (!window) {
        std::cout << "Could not create window." << std::endl;
        return 1;
    }
    viewer->addWindow(window);
    
    // Setup camera
    vsg::dvec3 centre(0.0, 0.0, 0.0);
    double radius = 3.0;
    auto lookAt = vsg::LookAt::create(centre + vsg::dvec3(0.0, -radius * 3.5, 0.0), centre, vsg::dvec3(0.0, 0.0, 1.0));
    auto perspective = vsg::Perspective::create(30.0, static_cast<double>(window->extent2D().width) / static_cast<double>(window->extent2D().height), 0.1, 10.0);
    auto camera = vsg::Camera::create(perspective, lookAt, vsg::ViewportState::create(window->extent2D()));
    
    // Create command graph
    auto commandGraph = vsg::createCommandGraphForView(window, camera, scene);
    viewer->assignRecordAndSubmitTaskAndPresentation({commandGraph});
    
    // Add handlers
    viewer->addEventHandler(vsg::CloseHandler::create(viewer));
    viewer->addEventHandler(vsg::Trackball::create(camera));
    
    // Compile and run
    viewer->compile();
    
    std::cout << "Starting render loop. You should see a red box." << std::endl;
    
    int frameCount = 0;
    while (viewer->advanceToNextFrame() && frameCount < 300) {
        viewer->handleEvents();
        viewer->update();
        viewer->recordAndSubmit();
        viewer->present();
        
        if (frameCount % 60 == 0) {
            std::cout << "Frame " << frameCount << std::endl;
        }
        frameCount++;
    }
    
    return 0;
}