/*
 * Autopilot.cpp
 *
 *  Created on: 10 Jan 2017
 *      Author: sleutene
 */

#include <arp/Autopilot.hpp>
#include <arp/kinematics/operators.hpp>
#include <Planner.h>

namespace arp {

Autopilot::Autopilot(ros::NodeHandle& nh)
    : nh_(&nh)
{
  isAutomatic_ = false; // always start in manual mode  

  // receive navdata
  subNavdata_ = nh.subscribe("ardrone/navdata", 50, &Autopilot::navdataCallback,
                             this);

  // commands
  pubReset_ = nh_->advertise<std_msgs::Empty>("/ardrone/reset", 1);
  pubTakeoff_ = nh_->advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
  pubLand_ = nh_->advertise<std_msgs::Empty>("/ardrone/land", 1);
  pubMove_ = nh_->advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  // flattrim service
  srvFlattrim_ = nh_->serviceClient<std_srvs::Empty>(
      nh_->resolveName("ardrone/flattrim"), 1);

  // Initialize PID-Controller parameters
  // Get ros parameters 
  bool success = true;
  success &= nh_->getParam("/ardrone_driver/euler_angle_max", euler_angle_max_);
  success &= nh_->getParam("/ardrone_driver/control_vz_max", control_vz_max_);
  success &= nh_->getParam("/ardrone_driver/control_yaw", control_yaw_);
  arp::PidController::Parameters controllerParameters_xy, controllerParameters_z, controllerParameters_yaw;
  success &= nh_->getParam("/ardrone_driver/pid_xy_kp", controllerParameters_xy.k_p);
  success &= nh_->getParam("/ardrone_driver/pid_xy_ki", controllerParameters_xy.k_i);
  success &= nh_->getParam("/ardrone_driver/pid_xy_kd", controllerParameters_xy.k_d);
  success &= nh_->getParam("/ardrone_driver/pid_z_kp", controllerParameters_z.k_p);
  success &= nh_->getParam("/ardrone_driver/pid_z_ki", controllerParameters_z.k_i);
  success &= nh_->getParam("/ardrone_driver/pid_z_kd", controllerParameters_z.k_d);
  success &= nh_->getParam("/ardrone_driver/pid_yaw_kp", controllerParameters_yaw.k_p);
  success &= nh_->getParam("/ardrone_driver/pid_yaw_ki", controllerParameters_yaw.k_i);
  success &= nh_->getParam("/ardrone_driver/pid_yaw_kd", controllerParameters_yaw.k_d);
  if (!success) {
    ROS_ERROR("Error reading ROS parameters.");
    return;
  }
  // Convert control_vz_max from mm/s to m/s.
  control_vz_max_ *= 1.0e-3;

  double eps = 1.0e-6;
  if (euler_angle_max_ < eps || control_vz_max_ < eps || control_yaw_ < eps) // Check if limits are too small before dividing!
  {
    limitsTooSmall_ = true;
    ROS_ERROR("Output limits of controller close to zero (division by zero). Move command won't be sent");
    return;
  }
  
  // Set the output limits of the contollers.
  x_controller_.setOutputLimits(-euler_angle_max_, euler_angle_max_);
  y_controller_.setOutputLimits(-euler_angle_max_, euler_angle_max_);
  z_controller_.setOutputLimits(-control_vz_max_, control_vz_max_);
  yaw_controller_.setOutputLimits(-control_yaw_, control_yaw_);

  // Set the control parameters.
  x_controller_.setParameters(controllerParameters_xy);
  y_controller_.setParameters(controllerParameters_xy);
  z_controller_.setParameters(controllerParameters_z);
  yaw_controller_.setParameters(controllerParameters_yaw);
}

void Autopilot::navdataCallback(const ardrone_autonomy::NavdataConstPtr& msg)
{
  std::lock_guard<std::mutex> l(navdataMutex_);
  lastNavdata_ = *msg;
}

// Get the drone status.
Autopilot::DroneStatus Autopilot::droneStatus()
{
  ardrone_autonomy::Navdata navdata;
  {
    std::lock_guard<std::mutex> l(navdataMutex_);
    navdata = lastNavdata_;
  }
  return DroneStatus(navdata.state);
}

// Get the battery level
float Autopilot::getBatteryLevel()
{
  float batteryLevel;
  {
    std::lock_guard<std::mutex> l(navdataMutex_);
      batteryLevel = lastNavdata_.batteryPercent;
  }
  return batteryLevel;
}

// Request flattrim calibration.
bool Autopilot::flattrimCalibrate()
{
  DroneStatus status = droneStatus();
  if (status != DroneStatus::Landed) {
    return false;
  }
  // ARdrone -> flattrim calibrate
  std_srvs::Empty flattrimServiceRequest;
  srvFlattrim_.call(flattrimServiceRequest);
  return true;
}

// Takeoff.
bool Autopilot::takeoff()
{
  DroneStatus status = droneStatus();
  if (status != DroneStatus::Landed) {
    return false;
  }
  // ARdrone -> take off
  std_msgs::Empty takeoffMsg;
  pubTakeoff_.publish(takeoffMsg);
  return true;
}

// Land.
bool Autopilot::land()
{
  DroneStatus status = droneStatus();
  if (status != DroneStatus::Landed && status != DroneStatus::Landing
      && status != DroneStatus::Looping) {
    // ARdrone -> land
    std_msgs::Empty landMsg;
    pubLand_.publish(landMsg);
    return true;
  }
  return false;
}

// Turn off all motors and reboot.
bool Autopilot::estopReset()
{
  // ARdrone -> Emergency mode
  std_msgs::Empty resetMsg;
  pubReset_.publish(resetMsg);
  return true;
}

// Move the drone manually.
bool Autopilot::manualMove(double forward, double left, double up,
                           double rotateLeft)
{
  // Check for manual mode
  if (isAutomatic()) {
    return false;
  }
  return move(forward, left, up, rotateLeft);
}

// Move the drone.
bool Autopilot::move(double forward, double left, double up, double rotateLeft)
{
  // Check for valid drone status.
  if (!isFlying()) {
    return false;
  }

  //check the boundaries
  if(fabs(forward) > 1.0 || fabs(left) > 1.0 || fabs(up) > 1.0 || fabs(rotateLeft) > 1.0){
    std::cout << std::endl << "desired command is out of boundary, it must be in the range [-1, 1]  "  << std::endl;
    return false;
  }
  
  geometry_msgs::Twist moveMsg;
  moveMsg.linear.x = forward; 
  moveMsg.linear.y = left; 
  moveMsg.linear.z = up; 
  moveMsg.angular.x = 0;
  moveMsg.angular.y = 0;
  moveMsg.angular.z = rotateLeft;
  pubMove_.publish(moveMsg);
  return true;
}

// Set to automatic control mode.
void Autopilot::setManual()
{
  isAutomatic_ = false;
}

// Set to manual control mode.
void Autopilot::setAutomatic()
{
  x_controller_.resetIntegrator();
  y_controller_.resetIntegrator();
  z_controller_.resetIntegrator();
  yaw_controller_.resetIntegrator();
  isAutomatic_ = true;
}

// Move the drone automatically.
bool Autopilot::setPoseReference(double x, double y, double z, double yaw)
{
  std::lock_guard<std::mutex> l(refMutex_);
  ref_x_ = x;
  ref_y_ = y;
  ref_z_ = z;
  ref_yaw_ = yaw;
  return true;
}

bool Autopilot::getPoseReference(double& x, double& y, double& z, double& yaw) {
  std::lock_guard<std::mutex> l(refMutex_);
  x = ref_x_;
  y = ref_y_;
  z = ref_z_;
  yaw = ref_yaw_;
  return true;
}

/// The callback from the estimator that sends control outputs to the drone
void Autopilot::controllerCallback(uint64_t timeMicroseconds,
                                  const arp::kinematics::RobotState& x)
{
  // only do anything here, if automatic
  if (!isAutomatic_) {
    // keep resetting this to make sure we use the current state as reference as soon as sent to automatic mode
    const double yaw = kinematics::yawAngle(x.q_WS);
    setPoseReference(x.t_WS[0], x.t_WS[1], x.t_WS[2], yaw);
    return;
  }

  // Only enable when in flight
  if (!isFlying()) {
    return;
  }

  Eigen::Vector3d positionReference;
  double yaw_ref;
  bool lastWaypointPopped = false;

  // Get waypoint list, if available
  { // <- manual scope for mutex lock
    std::lock_guard<std::mutex> l(waypointMutex_);
    if(!waypoints_.empty()) {
      // setPoseReference() from current waypoint
      const Waypoint& currentWp = waypoints_.front();
      setPoseReference(currentWp.x, currentWp.y, currentWp.z, currentWp.yaw); // TODO is this really necessary for every loop iteration? maybe implement a check
      getPoseReference(positionReference[0], positionReference[1], positionReference[2], yaw_ref);
      // remove current waypoint, if position error below tolerance.
      if ((x.t_WS - positionReference).norm() < currentWp.posTolerance) {
        waypoints_.pop_front();
        std::cout << "REACHED A WAYPOINT: " << x.t_WS[0] << " " << x.t_WS[1] << " " <<x.t_WS[2] << std::endl;
        if (waypoints_.empty()) {
          lastWaypointPopped = true;
        }
      }
    } else {
      // This is the original line of code:
      getPoseReference(positionReference[0], positionReference[1],positionReference[2], yaw_ref);
    }
  }

  // has to be handled outside the lock scope because otherwise there
  // will be a deadlock:
  // > controllerCallback locks waypoints
  // > calls destinationReachedCallback which wants to set waypoints (!!!)
  if (lastWaypointPopped && destinationReached_) {
    destinationReached_();
  }

  // Compute position error
  Eigen::Matrix3d R_SW = x.q_WS.toRotationMatrix().transpose();
  Eigen::Vector3d position_error = R_SW * (positionReference - x.t_WS);
  
  // Compute yaw error
  double yaw_estimated = arp::kinematics::yawAngle(x.q_WS);
  double yaw_error = yaw_ref - yaw_estimated;
  // Ensure that yaw error is within the limits of [-pi,pi]
  yaw_error += M_PI;
  double num_shifts = floor(yaw_error / (2*M_PI));
  yaw_error += -num_shifts * 2*M_PI;
  yaw_error -= M_PI;
  
  // Compute the approximated time derivatives of the error signals
  Eigen::Vector3d position_error_dot = -R_SW * x.v_W;
  double yaw_error_dot = 0.0;

  double output_x = x_controller_.control(timeMicroseconds, position_error(0), position_error_dot(0));
  double output_y = y_controller_.control(timeMicroseconds, position_error(1), position_error_dot(1));
  double output_z = z_controller_.control(timeMicroseconds, position_error(2), position_error_dot(2));
  double output_yaw = yaw_controller_.control(timeMicroseconds, yaw_error, yaw_error_dot);

  if (!limitsTooSmall_) {
    // Scale the controller outputs for the move command
    output_x /= euler_angle_max_;
    output_y /= euler_angle_max_;
    output_z /= control_vz_max_;
    output_yaw /= control_yaw_;

    // Command the drone to move
    move(output_x, output_y, output_z, output_yaw);
  }
}

}  // namespace arp

