#pragma once

#include <arp/Autopilot.hpp>
#include <Renderer.hpp>
#include <Task.h>
#include <arp/VisualInertialTracker.hpp>

namespace Commands {

/// @brief Check all associated keys for a command.
/// @param autopilot 
/// @param renderer
/// @param visualInertialTracker
void checkKeysForCommand(arp::Autopilot& autopilot, 
                         gui::Renderer& renderer,
                         arp::VisualInertialTracker& visualInertialTracker,
                         Task &task);

}