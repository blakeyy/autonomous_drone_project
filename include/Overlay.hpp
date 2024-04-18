#pragma once

#include <opencv2/highgui/highgui.hpp>
#include <arp/Autopilot.hpp>

namespace Overlay {

///\brief Writes drone teleoperation instructions to the supplied image.
void displayInstructions(cv::Mat& image);

///\brief Writes the battery level to the supplied image, rounded to integers. 
void displayBattery(cv::Mat& image, float battery);
    
///\brief Writes the drone status (in prose) to the supplied image.
void displayDroneStatus(cv::Mat& image, arp::Autopilot::DroneStatus droneStatus);

/// @brief Writes the current control mode to the supplied image.
void displayControlMode(cv::Mat& image, bool isAutomatic);

}