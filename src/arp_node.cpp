#include <memory>
#include <unistd.h>
#include <stdlib.h>
#include <fstream>

#include <SDL2/SDL.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_srvs/Empty.h>

#include <arp/Autopilot.hpp>
#include <arp/cameras/PinholeCamera.hpp>
#include <arp/cameras/RadialTangentialDistortion.hpp>
#include <arp/VisualInertialTracker.hpp>
#include <arp/StatePublisher.hpp>

#include <Commands.hpp>
#include <Renderer.hpp>
#include <OccupancyMap.h>
#include <Task.h>
#include <arp/Frontend.hpp>

class Subscriber
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Subscriber(arp::VisualInertialTracker* visualInertialTrackerPtr){
    visualInertialTrackerPtr_ = visualInertialTrackerPtr;
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    // Get time stamp
    uint64_t timeMicroseconds = uint64_t(msg->header.stamp.sec) * 1000000ll + msg->header.stamp.nsec / 1000;
    // -- for later use
    std::lock_guard<std::mutex> l(imageMutex_);
    lastImage_ = cv_bridge::toCvShare(msg, "bgr8")->image;
    // Feed image to VisualIntertialTracker
    visualInertialTrackerPtr_->addImage(timeMicroseconds, lastImage_);
  }

  bool getLastImage(cv::Mat& image)
  {
    std::lock_guard<std::mutex> l(imageMutex_);
    if (lastImage_.empty())
      return false;
    image = lastImage_.clone();
    lastImage_ = cv::Mat();  // clear, only get same image once.
    return true;
  }

  void imuCallback(const sensor_msgs::ImuConstPtr& msg)
  {
    // Get time stamp
    uint64_t timeMicroseconds = uint64_t(msg->header.stamp.sec) * 1000000ll + msg->header.stamp.nsec / 1000;
    // Feed IMU measurement to VisualIntertialTracker
    Eigen::Vector3d omega_S, acc_S;
    omega_S[0] = msg->angular_velocity.x;
    omega_S[1] = msg->angular_velocity.y;
    omega_S[2] = msg->angular_velocity.z;
    acc_S[0] = msg->linear_acceleration.x;
    acc_S[1] = msg->linear_acceleration.y;
    acc_S[2] = msg->linear_acceleration.z;
    visualInertialTrackerPtr_->addImuMeasurement(timeMicroseconds, omega_S, acc_S);
  }

 private:
  cv::Mat lastImage_;
  std::mutex imageMutex_;
  arp::VisualInertialTracker* visualInertialTrackerPtr_;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "arp_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  // set up autopilot
  arp::Autopilot autopilot(nh);

  // set up camera model
  bool success = true;
  double k1, k2, p1, p2, fu, fv, cu, cv;
  int imageWidth, imageHeight;
  success &= nh.getParam("/arp_node/k1", k1);
  success &= nh.getParam("/arp_node/k2", k2);
  success &= nh.getParam("/arp_node/p1", p1);
  success &= nh.getParam("/arp_node/p2", p2);
  success &= nh.getParam("/arp_node/fu", fu);
  success &= nh.getParam("/arp_node/fv", fv);
  success &= nh.getParam("/arp_node/cu", cu);
  success &= nh.getParam("/arp_node/cv", cv);
  success &= nh.getParam("/arp_node/image_width", imageWidth);
  success &= nh.getParam("/arp_node/image_height", imageHeight);
  std::cout << "k^T = [" << k1 << ", " << k2 << ", " << p1 << ", " << p2 << ", 0]" << std::endl;
  std::cout << "camera: fu="  << fu << ", fv=" << fv << ", cu=" << cu << ", cv=" << cv << std::endl;
  if (!success) {
    ROS_ERROR("Error reading camera parameters.");
    return -1;
  }
  arp::cameras::RadialTangentialDistortion distortion{k1, k2, p1, p2};
  arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> phc{
    imageWidth, imageHeight, fu, fv, cu, cv, distortion};
  phc.initialiseUndistortMaps(imageWidth, imageHeight, fu, fv, cu, cv);

  // set up frontend
  success = true;
  double mapCamFocalLength;
  arp::FrontendThresholds thresholds;
  arp::FeatureDetectionParams featureDetectionParams;
  success &= nh.getParam("/arp_node/map_camera_focal_length", mapCamFocalLength);
  // frame matching
  success &= nh.getParam("/arp_node/min_matches", thresholds.minMatches);
  success &= nh.getParam("/arp_node/max_bow_results", thresholds.maxBoWResults);
  // feature detection
  success &= nh.getParam("/arp_node/uniformity_radius", featureDetectionParams.uniformityRadius);
  success &= nh.getParam("/arp_node/octaves", featureDetectionParams.octaves);
  success &= nh.getParam("/arp_node/absolute_threshold", featureDetectionParams.absoluteThreshold);
  success &= nh.getParam("/arp_node/max_num_kpt", featureDetectionParams.maxNumKpt);
  std::cout << "frontend: min_num_matches="  << thresholds.minMatches 
            << ", num_dbow_results=" << thresholds.maxBoWResults << std::endl;
  std::cout << "detector: uniformity_radius="  << featureDetectionParams.uniformityRadius 
            << ", octaves=" << featureDetectionParams.octaves 
            << ", absolute_threshold=" << featureDetectionParams.absoluteThreshold 
            << ", max_num_kpt=" << featureDetectionParams.maxNumKpt << std::endl;
  if (!success) {
    ROS_ERROR("Error reading frontend parameters.");
    return -1;
  }
  arp::Frontend frontend(imageWidth, imageHeight, 
                         fu, fv, cu, cv, k1, k2, p1, p2,
                         mapCamFocalLength, 
                         thresholds, featureDetectionParams);

  // load map
  std::string path = ros::package::getPath("ardrone_practicals");
  std::string mapFile;
  if(!nh.getParam("arp_node/map", mapFile)) ROS_FATAL("error loading parameter");
  std::string mapPath = path+"/maps/"+mapFile;
  if(!frontend.loadMap(mapPath)) ROS_FATAL_STREAM("could not load map from " << mapPath << " !");

  // load occupancy map
  if (!nh.getParam("arp_node/occupancymap", mapFile)) ROS_FATAL("Could not find occupancy map path parameter.");
  mapPath = path+"/maps/"+mapFile;
  auto occupancyMap = std::make_shared<OccupancyMap>(mapPath);
  
  // load DBoW2 vocabulary
  std::string vocPath = path+"/maps/small_voc.yml.gz";
  if(!frontend.loadDBoW2Voc(vocPath)) ROS_FATAL_STREAM("could not load DBoW2 voc. from " << vocPath << " !");
  if(!frontend.buildDBoW2Database()) ROS_FATAL_STREAM("Could not build DBoW2 database from vocabulary.");

  // state publisher -- provided for rviz visualisation of drone pose:
  arp::StatePublisher pubState(nh);
  
  // set up EKF
  arp::ViEkf viEkf;
  Eigen::Matrix4d T_SC_mat;
  std::vector<double> T_SC_array;
  if(!nh.getParam("arp_node/T_SC", T_SC_array))
  ROS_FATAL("error loading parameter");
  T_SC_mat <<
  T_SC_array[0], T_SC_array[1], T_SC_array[2], T_SC_array[3],
  T_SC_array[4], T_SC_array[5], T_SC_array[6], T_SC_array[7],
  T_SC_array[8], T_SC_array[9], T_SC_array[10], T_SC_array[11],
  T_SC_array[12], T_SC_array[13], T_SC_array[14], T_SC_array[15];
  arp::kinematics::Transformation T_SC(T_SC_mat);
  viEkf.setCameraExtrinsics(T_SC);
  viEkf.setCameraIntrinsics(frontend.camera());

  // set up visual-inertial tracking
  arp::VisualInertialTracker visualInertialTracker;
  visualInertialTracker.setFrontend(frontend);
  visualInertialTracker.setEstimator(viEkf);

  // set up visualisation: publish poses to topic ardrone/vi_ekf_pose
  visualInertialTracker.setVisualisationCallback(std::bind(&arp::StatePublisher::publish, &pubState, std::placeholders::_1, std::placeholders::_2));
  
  // set up controller callback (estimator needs to call the autopilot, when it's done computing the state)
  visualInertialTracker.setControllerCallback(std::bind(&arp::Autopilot::controllerCallback, &autopilot, std::placeholders::_1, std::placeholders::_2));
      
  // setup inputs
  Subscriber subscriber(&visualInertialTracker);
  image_transport::Subscriber subImage = it.subscribe(
      "ardrone/front/image_raw", 2, &Subscriber::imageCallback, &subscriber);
  ros::Subscriber subImu = nh.subscribe("ardrone/imu", 50,
                                        &Subscriber::imuCallback, &subscriber);

  // setup rendering
  gui::Renderer renderer{phc};

  // load goal location
  std::vector<double> pointB;
  if (!nh.getParam("arp_node/pointB", pointB)) ROS_FATAL("Couldn't load goal point.");
  // autopilot.setGoal(Eigen::Vector3d{pointB.data()});

  Task task{nh, autopilot, viEkf, Eigen::Vector3d{pointB.data()}, *occupancyMap};

  // enter main event loop
  std::cout << "===== Hello AR Drone ====" << std::endl;
  // cv::Mat image;
  cv::Mat imageWithKeypoints;
  while (ros::ok()) {
    ros::spinOnce();
    ros::Duration dur(0.04);
    dur.sleep();
    if(renderer.checkQuit()) {
      break;
    }

    // render image, if there is a new one available
    // if(subscriber.getLastImage(image)) {
    //   renderer.render(image, autopilot.droneStatus(), autopilot.getBatteryLevel());
    // }
    if(visualInertialTracker.getLastVisualisationImage(imageWithKeypoints)) {
      renderer.render(imageWithKeypoints, autopilot.droneStatus(), autopilot.getBatteryLevel(), autopilot.isAutomatic());
    }

    // Check if keys are pressed and execute associated commands
    Commands::checkKeysForCommand(autopilot, renderer, visualInertialTracker, task);
  }

  // make sure to land the drone...
  success = autopilot.land();
  return success;
}

