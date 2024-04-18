#pragma once

#include <deque>
#include <arp/Autopilot.hpp>
#include <Eigen/Core>
#include <OccupancyMap.h>

namespace Planner {

std::deque<arp::Autopilot::Waypoint> planFlight(const Eigen::Vector3d& start, 
                                                const Eigen::Vector3d& goal, 
                                                const OccupancyMap& occupancyMap);

}

// ------------------------------------------------------------------------------------------------------
// #include <ros/ros.h>
// #include <opencv2/highgui/highgui.hpp>
// #include <iostream>
// #include <utility>
// #include <fstream>
// #include <string>
// #include <arp/Autopilot.hpp>
// #include <Eigen/Core>

// #include <opencv2/core.hpp>
// #include <opencv2/imgcodecs.hpp>
// #include <opencv2/highgui.hpp>
// #include <opencv2/core/types.hpp>

// #include <cmath>

