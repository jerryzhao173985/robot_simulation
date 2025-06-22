# Window Configuration Settings

## Current Settings

- **Window Size**: 800x600 pixels (reduced from 1920x1080)
- **MSAA Samples**: 1x (reduced from 4x for better performance)
- **VSync**: Disabled (for maximum frame rate)
- **Window Mode**: Windowed (not fullscreen)
- **Decoration**: True (window has title bar and borders)

## To Change Window Size

Edit line 55 in `src/main.cpp`:
```cpp
visualizer = std::make_unique<Visualizer>(800, 600);
```

Common sizes:
- Small: 640x480
- Medium: 800x600 (current)
- Large: 1024x768
- HD: 1280x720
- Full HD: 1920x1080

## Performance Notes

Smaller windows generally provide:
- Better frame rates
- Less GPU memory usage
- Easier window management
- No need to resize after launch

The window resize storm issue may be related to macOS retina display scaling.