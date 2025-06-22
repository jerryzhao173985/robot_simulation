#pragma once

#include <vsg/all.h>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <map>
#include <string>
#include <sstream>

// Comprehensive event debugger that captures and analyzes all VSG events
class ComprehensiveEventDebugger : public vsg::Inherit<vsg::Visitor, ComprehensiveEventDebugger>
{
public:
    ComprehensiveEventDebugger() : 
        start_time(std::chrono::high_resolution_clock::now()),
        event_count(0) {
        std::cout << "[EventDebugger] Initialized at " << getTimestamp() << std::endl;
    }

    // Override all event handlers to capture everything
    void apply(vsg::Object& object) override {
        std::cout << "[EventDebugger] Unknown object type: " << typeid(object).name() << std::endl;
        object.traverse(*this);
    }

    void apply(vsg::UIEvent& event) override {
        logEvent("UIEvent", event);
        event.traverse(*this);
    }

    void apply(vsg::WindowEvent& event) override {
        logEvent("WindowEvent", event);
        event.traverse(*this);
    }

    void apply(vsg::KeyEvent& event) override {
        std::ostringstream oss;
        oss << "keyBase=" << event.keyBase << "(" << getKeyName(event.keyBase) << ")"
            << " keyModified=" << event.keyModified << "(" << getKeyName(event.keyModified) << ")"
            << " modifier=" << event.keyModifier << "(" << getModifierString(event.keyModifier) << ")"
            << " repeatCount=" << event.repeatCount;
        logEvent("KeyEvent", event, oss.str());
        event.traverse(*this);
    }

    void apply(vsg::KeyPressEvent& event) override {
        keyPressCount[event.keyBase]++;
        
        std::ostringstream oss;
        oss << "KEY='" << getKeyName(event.keyBase) << "' (" << event.keyBase << ")"
            << " Modified='" << getKeyName(event.keyModified) << "' (" << event.keyModified << ")"
            << " Modifiers=" << getModifierString(event.keyModifier)
            << " Repeat=" << event.repeatCount
            << " TotalPresses=" << keyPressCount[event.keyBase];
        
        logEvent("KeyPressEvent", event, oss.str());
        
        // Track timing between key presses
        auto now = std::chrono::high_resolution_clock::now();
        if (lastKeyPressTime.count(event.keyBase) > 0) {
            auto delta = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastKeyPressTime[event.keyBase]);
            std::cout << "    -> Time since last press: " << delta.count() << "ms" << std::endl;
        }
        lastKeyPressTime[event.keyBase] = now;
    }

    void apply(vsg::KeyReleaseEvent& event) override {
        auto now = std::chrono::high_resolution_clock::now();
        
        std::ostringstream oss;
        oss << "KEY='" << getKeyName(event.keyBase) << "' (" << event.keyBase << ")";
        
        // Calculate key hold duration
        if (lastKeyPressTime.count(event.keyBase) > 0) {
            auto holdDuration = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastKeyPressTime[event.keyBase]);
            oss << " HoldDuration=" << holdDuration.count() << "ms";
        }
        
        logEvent("KeyReleaseEvent", event, oss.str());
    }

    void apply(vsg::ButtonPressEvent& event) override {
        std::ostringstream oss;
        oss << "button=" << event.button 
            << " mask=" << event.mask
            << " x=" << event.x << " y=" << event.y;
        logEvent("ButtonPressEvent", event, oss.str());
    }

    void apply(vsg::ButtonReleaseEvent& event) override {
        std::ostringstream oss;
        oss << "button=" << event.button 
            << " mask=" << event.mask
            << " x=" << event.x << " y=" << event.y;
        logEvent("ButtonReleaseEvent", event, oss.str());
    }

    void apply(vsg::MoveEvent& event) override {
        // Only log every Nth move event to avoid spam
        static int moveCount = 0;
        if (++moveCount % 100 == 0) {
            std::ostringstream oss;
            oss << "x=" << event.x << " y=" << event.y 
                << " mask=" << event.mask << " (100 events)";
            logEvent("MoveEvent", event, oss.str());
        }
    }

    void apply(vsg::ScrollWheelEvent& event) override {
        std::ostringstream oss;
        oss << "delta=(" << event.delta.x << "," << event.delta.y << ")";
        logEvent("ScrollWheelEvent", event, oss.str());
    }

    void apply(vsg::ExposeWindowEvent& event) override {
        logEvent("ExposeWindowEvent", event);
    }

    void apply(vsg::ConfigureWindowEvent& event) override {
        // Skip logging window resize events to avoid spam
        static int configCount = 0;
        configCount++;
        if (configCount % 100 == 0) {
            std::ostringstream oss;
            oss << "x=" << event.x << " y=" << event.y 
                << " width=" << event.width << " height=" << event.height
                << " (100 events)";
            logEvent("ConfigureWindowEvent", event, oss.str());
        }
    }

    void apply(vsg::CloseWindowEvent& event) override {
        logEvent("CloseWindowEvent", event);
    }

    void apply(vsg::FocusInEvent& event) override {
        logEvent("FocusInEvent", event);
    }

    void apply(vsg::FocusOutEvent& event) override {
        logEvent("FocusOutEvent", event);
    }

    void apply(vsg::TerminateEvent& event) override {
        logEvent("TerminateEvent", event);
    }

    // Helper methods
    void printSummary() const {
        std::cout << "\n=== Event Summary ===" << std::endl;
        std::cout << "Total events: " << event_count << std::endl;
        std::cout << "Event types:" << std::endl;
        for (const auto& [type, count] : eventTypeCounts) {
            std::cout << "  " << type << ": " << count << std::endl;
        }
        std::cout << "\nKey press counts:" << std::endl;
        for (const auto& [key, count] : keyPressCount) {
            std::cout << "  " << getKeyName(key) << " (" << key << "): " << count << " presses" << std::endl;
        }
    }

private:
    std::chrono::high_resolution_clock::time_point start_time;
    std::chrono::high_resolution_clock::time_point lastEventTime;
    std::map<vsg::KeySymbol, std::chrono::high_resolution_clock::time_point> lastKeyPressTime;
    std::map<vsg::KeySymbol, int> keyPressCount;
    std::map<std::string, int> eventTypeCounts;
    int event_count;

    std::string getTimestamp() const {
        auto now = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time);
        
        std::ostringstream oss;
        oss << std::setfill('0') << std::setw(6) << elapsed.count() << "ms";
        return oss.str();
    }

    void logEvent(const std::string& type, const vsg::UIEvent& event, const std::string& details = "") {
        event_count++;
        eventTypeCounts[type]++;
        
        auto now = std::chrono::high_resolution_clock::now();
        auto deltaFromLast = lastEventTime.time_since_epoch().count() > 0 ?
            std::chrono::duration_cast<std::chrono::microseconds>(now - lastEventTime).count() : 0;
        lastEventTime = now;
        
        std::cout << "[" << getTimestamp() << "] " 
                  << std::setw(20) << std::left << type;
        
        if (!details.empty()) {
            std::cout << " " << details;
        }
        
        if (deltaFromLast > 0) {
            std::cout << " (+" << deltaFromLast << "µs)";
        }
        
        std::cout << std::endl;
    }

    static std::string getKeyName(vsg::KeySymbol key) {
        switch(key) {
            // Letters
            case vsg::KEY_a: return "a";
            case vsg::KEY_b: return "b";
            case vsg::KEY_c: return "c";
            case vsg::KEY_d: return "d";
            case vsg::KEY_e: return "e";
            case vsg::KEY_f: return "f";
            case vsg::KEY_g: return "g";
            case vsg::KEY_h: return "h";
            case vsg::KEY_i: return "i";
            case vsg::KEY_j: return "j";
            case vsg::KEY_k: return "k";
            case vsg::KEY_l: return "l";
            case vsg::KEY_m: return "m";
            case vsg::KEY_n: return "n";
            case vsg::KEY_o: return "o";
            case vsg::KEY_p: return "p";
            case vsg::KEY_q: return "q";
            case vsg::KEY_r: return "r";
            case vsg::KEY_s: return "s";
            case vsg::KEY_t: return "t";
            case vsg::KEY_u: return "u";
            case vsg::KEY_v: return "v";
            case vsg::KEY_w: return "w";
            case vsg::KEY_x: return "x";
            case vsg::KEY_y: return "y";
            case vsg::KEY_z: return "z";
            
            // Capital letters
            case vsg::KEY_A: return "A";
            case vsg::KEY_B: return "B";
            case vsg::KEY_C: return "C";
            case vsg::KEY_D: return "D";
            case vsg::KEY_W: return "W";
            case vsg::KEY_S: return "S";
            
            // Special keys
            case vsg::KEY_Space: return "Space";
            case vsg::KEY_Escape: return "Escape";
            case vsg::KEY_Return: return "Return";
            case vsg::KEY_Tab: return "Tab";
            case vsg::KEY_BackSpace: return "BackSpace";
            case vsg::KEY_Delete: return "Delete";
            
            // Modifiers
            case vsg::KEY_Shift_L: return "Shift_L";
            case vsg::KEY_Shift_R: return "Shift_R";
            case vsg::KEY_Control_L: return "Control_L";
            case vsg::KEY_Control_R: return "Control_R";
            case vsg::KEY_Alt_L: return "Alt_L";
            case vsg::KEY_Alt_R: return "Alt_R";
            
            // Function keys
            case vsg::KEY_F1: return "F1";
            case vsg::KEY_F2: return "F2";
            case vsg::KEY_F3: return "F3";
            case vsg::KEY_F4: return "F4";
            
            // Navigation
            case vsg::KEY_Left: return "Left";
            case vsg::KEY_Right: return "Right";
            case vsg::KEY_Up: return "Up";
            case vsg::KEY_Down: return "Down";
            case vsg::KEY_Home: return "Home";
            case vsg::KEY_End: return "End";
            case vsg::KEY_Page_Up: return "Page_Up";
            case vsg::KEY_Page_Down: return "Page_Down";
            
            default: {
                std::ostringstream oss;
                oss << "Unknown(" << static_cast<int>(key) << ")";
                return oss.str();
            }
        }
    }

    static std::string getModifierString(vsg::KeyModifier mod) {
        std::ostringstream oss;
        oss << "[";
        bool first = true;
        
        if (mod & vsg::MODKEY_Shift) { oss << "Shift"; first = false; }
        if (mod & vsg::MODKEY_CapsLock) { if (!first) oss << "+"; oss << "CapsLock"; first = false; }
        if (mod & vsg::MODKEY_Control) { if (!first) oss << "+"; oss << "Control"; first = false; }
        if (mod & vsg::MODKEY_Alt) { if (!first) oss << "+"; oss << "Alt"; first = false; }
        if (mod & vsg::MODKEY_NumLock) { if (!first) oss << "+"; oss << "NumLock"; first = false; }
        if (mod & vsg::MODKEY_Meta) { if (!first) oss << "+"; oss << "Meta"; }
        
        if (first) oss << "None";
        oss << "]";
        return oss.str();
    }
};