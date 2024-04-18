#include "Overlay.hpp"
#include <cv_bridge/cv_bridge.h>

namespace Overlay {

namespace Color {
const cv::Scalar Red{CV_RGB(255, 0, 0)};
const cv::Scalar Green{CV_RGB(0, 255, 0)};
const cv::Scalar Blue{CV_RGB(0, 0, 255)};
const cv::Scalar White{CV_RGB(255, 255, 255)};
}

namespace Font {
const cv::HersheyFonts Family = cv::FONT_HERSHEY_DUPLEX;
const float Scale = .6;
const int Thickness = 1;
}

/// Writes text to the image with preconfigured font family, scale, and thickness.
void putText(cv::Mat& image, const std::string& text, const cv::Point& point, const cv::Scalar& color)
{
    cv::putText(image, text, point, Font::Family, Font::Scale, color, Font::Thickness);
}

void displayInstructions(cv::Mat& image)
{
     // draw w-a-s-d control
      putText(image, "[W]/[S]: up/down         ", cv::Point(5, image.rows - 100), Color::Blue);
      putText(image, "[A]/[D]: yaw left/right  ", cv::Point(5, image.rows - 75), Color::Blue);

      // draw arrow control
      putText(image, "[^]/[v]: forward/backward", cv::Point(5, image.rows - 35), Color::Blue);
      putText(image, "[<]/[>]: left/right      ", cv::Point(5, image.rows - 10), Color::Blue);
      
      // draw t,l,e control
      putText(image, "[T]/[L]: takeoff/landing ", cv::Point(image.cols - 260, image.rows - 35), Color::Green);
      putText(image, "[Esc]: shut-off motors   ", cv::Point(image.cols - 260, image.rows - 10), Color::Red);

      // draw enable/disable undistortion command
      putText(image, "[K]: undistortion on/off ", cv::Point(image.cols - 260, image.rows - 60), Color::White);
      
      // draw enable/disable fusion command
      putText(image, "[E]: fusion enable/disable ", cv::Point(image.cols - 260, image.rows - 85), Color::White);

      // draw enter manual mode command
      putText(image, "[SPACE]: manual mode ", cv::Point(image.cols - 260, image.rows - 110), Color::White);
      
      // draw enter automatic mode command
      putText(image, "[RCTRL]: automatic mode ", cv::Point(image.cols - 260, image.rows - 135), Color::White);
}

void displayBattery(cv::Mat& image, float battery)
{
    // make battery red if SoC < 25% (otherwise white)
    cv::Scalar colorBattery;
    if (battery < 25) {
        colorBattery = Color::Red;
    } else {
        colorBattery = Color::White;
    }
    std::string text = "Battery: " + std::to_string(static_cast<int>(battery)) + "%";
    putText(image, text, cv::Point(image.cols - 125, 15), colorBattery);
}

void displayDroneStatus(cv::Mat& image, arp::Autopilot::DroneStatus droneStatus)
{
    using DroneStatus = arp::Autopilot::DroneStatus;
    std::string statusAsString;
    switch (droneStatus)
    {
    case DroneStatus::Unknown: statusAsString = "Unknown"; break;
    case DroneStatus::Inited: statusAsString = "Inited"; break;
    case DroneStatus::Landed: statusAsString = "Landed"; break;
    case DroneStatus::Flying: statusAsString = "Flying"; break;
    case DroneStatus::Hovering: statusAsString = "Hovering"; break;
    case DroneStatus::Test: statusAsString = "Test"; break;
    case DroneStatus::TakingOff: statusAsString = "TakingOff"; break;
    case DroneStatus::Flying2: statusAsString = "Flying2"; break;
    case DroneStatus::Landing: statusAsString = "Landing"; break;
    case DroneStatus::Looping: statusAsString = "Looping"; break;
    default: statusAsString = "Unknown"; break;
    }
    std::string text = "Status: " + statusAsString;
    putText(image, text, cv::Point(5, 15), Color::White);
}

void displayControlMode(cv::Mat& image, bool isAutomatic)
{
    if (isAutomatic) {
        putText(image, "Control mode: AUTOMATIC", cv::Point(5, 40), Color::White);
    } else {
        putText(image, "Control mode: MANUAL", cv::Point(5, 40), Color::White);
    }
}

}