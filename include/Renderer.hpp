#pragma once

#include <arp/Autopilot.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <SDL2/SDL.h>
#include <arp/cameras/PinholeCamera.hpp>
#include <arp/cameras/RadialTangentialDistortion.hpp>

namespace gui {

constexpr int WINDOW_WIDTH = 1280;
constexpr int WINDOW_HEIGHT = 720;

// not templated in distortion for now
class Renderer {
   using camera_t = arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion>;
 private:
    SDL_Event event;
    SDL_Window * window;
    SDL_Renderer * renderer;
    SDL_Texture * texture;
    camera_t camera;
    bool undistortBeforeRender;
 public:
    /// @brief Setup rendering.
    Renderer(camera_t camera, bool undistortBeforeRender = true);
    /// @brief Clean up rendering.
    ~Renderer();
    /// @brief Display the image with the respective overlay.
    /// @param image Image to be displayed by the renderer.
    /// @param droneStatus Status of the drone for the overlay.
    /// @param batteryLevel Battery level of the drone for the overlay.
    void render(cv::Mat& image, arp::Autopilot::DroneStatus droneStatus, float batteryLevel, bool isAutomatic);
    /// @brief Toggles whether the renderer undistorts the image received from the camera before showing it.
    void toggleUndistortionBeforeRender();
    /// @brief Returns true if SDL_Window is closed (SDL_QUIT event). 
    /// @return 
    bool checkQuit();
};

}