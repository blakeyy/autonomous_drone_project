#include <Task.h>
#include <Planner.h>
#include <thread>
#include <chrono>
#include <iostream>

Task::Task(ros::NodeHandle& nh, arp::Autopilot& autopilot, arp::ViEkf& viEkf, Eigen::Vector3d goal, const OccupancyMap& occupancyMap) : 
        autopilot_{autopilot}, viEkf_{viEkf}, goal_{goal}, occupancyMap_{occupancyMap}
{
    autopilot_.onDestinationReached(std::bind(&Task::goalReachedCallback, std::ref(*this)));
    timer_ = nh.createSteadyTimer(
        ros::WallDuration(3/*seconds*/),
        &Task::timerCallback,
        this, // receiver object
        true, // oneshot
        false // autostart
    );
}

void Task::timerCallback(const ros::SteadyTimerEvent& event)
{
    startJourney(false);
}

void Task::execute()
{
    if (running_) return;
    running_ = true;

    if (!initialized_) {
        // the current position is the start position, point A
        start_ = viEkf_.getPositionEstimate();
        if (!autopilot_.isFlying()) {
            // set Z to 1 if the start point is set while landed.. 
            // otherwise the drone kamikazes
            start_(2) = 1;
        }
        std::cout << "Setting start point to: [" << start_(0) << "," << start_(1) << "," << start_(2) << "]" << std::endl;
        initialized_ = true;
    }
    startJourney();
}

void Task::pause()
{
    if (!running_) return;
    running_ = false;
    autopilot_.setManual();
}

void Task::startJourney(bool checkFlying)
{
    if (checkFlying && !autopilot_.isFlying()) {
        autopilot_.takeoff();
        // wait for the takeoff to finish
        timer_.setPeriod(ros::WallDuration(1.5), true); // needs to be reset because of how ros::(Steady)Timer is implemented.
        timer_.start();
        return; // timer_.start() will call startJourney again once we have taken off
    }
    auto dest = currentJourney_ == Journey::ToGoal ? goal_ : start_;
    autopilot_.flyPath(Planner::planFlight(viEkf_.getPositionEstimate(), dest, occupancyMap_));
    autopilot_.setAutomatic();
}

void Task::goalReachedCallback()
{
    std::cout << "Goal reached." << std::endl;
    autopilot_.land();
    std::this_thread::sleep_for(std::chrono::seconds(5)); // wait for the landing to finish
    currentJourney_ = Journey::ToStart;
    autopilot_.onDestinationReached(std::bind(&Task::startReachedCallback, std::ref(*this)));
    startJourney();
}

void Task::startReachedCallback()
{
    autopilot_.clearDestinationReachedCallback();
    std::cout << "Start reached." << std::endl;
    autopilot_.land();
}