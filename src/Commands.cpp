#include <Commands.hpp>

#include <SDL2/SDL.h>

namespace Commands {

// Prints the command to be executed + drone status
void printCommand(const std::string& text, const arp::Autopilot::DroneStatus droneStatus)
{
    std::cout << text << droneStatus;
}

// Prints whether the command was successful or not
void printStatus(const bool success)
{
    if(success) {
        std::cout << " [ OK ]" << std::endl;
    } else {
        std::cout << " [FAIL]" << std::endl;
    }
}

void checkKeysForCommand(arp::Autopilot& autopilot, 
                         gui::Renderer& renderer,
                         arp::VisualInertialTracker& visualInertialTracker,
                         Task& task)
{
    //Multiple Key Capture Begins
    const Uint8 *state = SDL_GetKeyboardState(NULL);

    // check states!
    // command
    auto droneStatus = autopilot.droneStatus();

    if (state[SDL_SCANCODE_ESCAPE]) {
      printCommand("ESTOP PRESSED, SHUTTING OFF ALL MOTORS status=", droneStatus);
      bool success = autopilot.estopReset();
      printStatus(success);
    }

    if (state[SDL_SCANCODE_T]) {
      printCommand("Taking off...                          status=", droneStatus);
      bool success = autopilot.takeoff();
      printStatus(success);
    }

    if (state[SDL_SCANCODE_L]) {
      printCommand("Going to land...                       status=", droneStatus);
      bool success = autopilot.land();
      printStatus(success);
    }
    if (state[SDL_SCANCODE_C]) {
      printCommand("Requesting flattrim calibration...     status=", droneStatus);
      bool success = autopilot.flattrimCalibrate();
      printStatus(success);
    }

    // toggle distortion mode on key-up of 'K'
    static bool distortionTogglePending = false;
    if (state[SDL_SCANCODE_K]) {
      distortionTogglePending = true;
    } else if (distortionTogglePending) {
      std::cout << "Toggling camera (un)distortion." << std::endl;
      renderer.toggleUndistortionBeforeRender();
      distortionTogglePending = false;
    }
    
    // enable fusion mode on key-up of 'E'
    static bool fusionTogglePending = false;
    if (state[SDL_SCANCODE_E]) {
      fusionTogglePending = true;
    } else if (fusionTogglePending) {
      if(visualInertialTracker.isFusionEnabled()){
        visualInertialTracker.enableFusion(false);
        std::cout << "EKF fusion disabled." << std::endl;
      } else {
        visualInertialTracker.enableFusion(true);
        std::cout << "EKF fusion enabled." << std::endl;
      }
      fusionTogglePending = false;
    }

    // if (state[SDL_SCANCODE_RCTRL]) {
    //   printCommand("Entering automatic control mode...     status=", droneStatus);
    //   autopilot.setAutomatic();
    // }

    if (state[SDL_SCANCODE_SPACE]) {
      printCommand("Entering manual control mode...        status=", droneStatus);
      task.pause();
      printStatus(true);
    }

    if (state[SDL_SCANCODE_P]) {
      printCommand("Entering challenge mode...        status=", droneStatus);
      printStatus(true);
      task.execute();
    }
    
    if (!autopilot.isAutomatic() && (droneStatus == autopilot.Flying || 
                                     droneStatus == autopilot.Hovering || 
                                     droneStatus == autopilot.Flying2)) {
        // Compute manual move values.
        double forward = state[SDL_SCANCODE_UP]   - state[SDL_SCANCODE_DOWN];
        double left    = state[SDL_SCANCODE_LEFT] - state[SDL_SCANCODE_RIGHT];
        double up      = state[SDL_SCANCODE_W]    - state[SDL_SCANCODE_S];
        double yawLeft = state[SDL_SCANCODE_A]    - state[SDL_SCANCODE_D];
        bool success = autopilot.manualMove(forward, left, up, yawLeft);
        
        // Only print non-zero move commands.
        if (forward != 0 || left != 0 || up != 0 || yawLeft != 0) {
            // printCommand("Sending move command...               status=", droneStatus);
            printCommand("Sending twist {u=" + std::to_string(static_cast<int>(up)) + 
                        ", f=" + std::to_string(static_cast<int>(forward)) +
                        ", l=" + std::to_string(static_cast<int>(left)) +
                        ", yL=" + std::to_string(static_cast<int>(yawLeft)) +
                        "}... status=", droneStatus);
            printStatus(success);
        } 
    }
}
}