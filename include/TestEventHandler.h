#pragma once
#include <vsg/all.h>
#include <iostream>

class TestEventHandler : public vsg::Inherit<vsg::Visitor, TestEventHandler>
{
public:
    TestEventHandler() {
        std::cout << "[TestEventHandler] Created" << std::endl;
    }
    
    // Remove the generic Event handler - VSG doesn't have a base Event class
    // We'll just handle specific event types
    
    void apply(vsg::KeyPressEvent& keyPress) override {
        std::cout << "[TestEventHandler] KeyPress: key=" << keyPress.keyBase 
                  << " keyModified=" << keyPress.keyModified << std::endl;
    }
    
    void apply(vsg::KeyReleaseEvent& keyRelease) override {
        std::cout << "[TestEventHandler] KeyRelease: key=" << keyRelease.keyBase << std::endl;
    }
    
    void apply(vsg::ButtonPressEvent& buttonPress) override {
        std::cout << "[TestEventHandler] ButtonPress: button=" << buttonPress.button << std::endl;
    }
    
    void apply(vsg::MoveEvent& moveEvent) override {
        // Don't spam with mouse move events
        static int moveCount = 0;
        if (++moveCount % 100 == 0) {
            std::cout << "[TestEventHandler] Mouse moved (100 events)" << std::endl;
        }
    }
};