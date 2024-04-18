#include <Renderer.hpp>
#include <Overlay.hpp>

namespace gui {

Renderer::Renderer(camera_t camera, bool undistortBeforeRender)
    : camera{camera}, undistortBeforeRender{undistortBeforeRender}
{
    SDL_Init(SDL_INIT_VIDEO);
    window = SDL_CreateWindow("Hello AR Drone", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, WINDOW_WIDTH, WINDOW_HEIGHT, 0);
    renderer = SDL_CreateRenderer(window, -1, 0);
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_RenderClear(renderer);
    SDL_RenderPresent(renderer);
}

Renderer::~Renderer()
{
    // cleanup
    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
}

void Renderer::render(cv::Mat& image, arp::Autopilot::DroneStatus droneStatus, float batteryLevel, bool isAutomatic)
{

    // undistort before creating texture
    cv::Mat undistortedImage = image;

    if (undistortBeforeRender) {
        camera.undistortImage(image, undistortedImage);
    }

    Overlay::displayInstructions(undistortedImage);
    Overlay::displayBattery(undistortedImage, batteryLevel);
    Overlay::displayDroneStatus(undistortedImage, droneStatus);
    Overlay::displayControlMode(undistortedImage, isAutomatic);
    
    // https://stackoverflow.com/questions/22702630/converting-cvmat-to-sdl-texture
    // I'm using SDL_TEXTUREACCESS_STREAMING because it's for a video player, you should
    // pick whatever suits you most: https://wiki.libsdl.org/SDL_TextureAccess
    // remember to pick the right SDL_PIXELFORMAT_* !

    texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_BGR24, SDL_TEXTUREACCESS_STREAMING, undistortedImage.cols, undistortedImage.rows);
    SDL_UpdateTexture(texture, NULL, (void*)undistortedImage.data, undistortedImage.step1());
    SDL_RenderClear(renderer);
    SDL_RenderCopy(renderer, texture, NULL, NULL);
    // Show our drawing
    SDL_RenderPresent(renderer);
    // cleanup (only after you're done displaying. you can repeatedly call UpdateTexture without destroying it)
    SDL_DestroyTexture(texture);
}

void Renderer::toggleUndistortionBeforeRender() {
    undistortBeforeRender = !undistortBeforeRender;
}

bool Renderer::checkQuit()
{
    SDL_PollEvent(&event);
    if (event.type == SDL_QUIT) {
        return true;
    }
    return false;
}

} // namespace gui