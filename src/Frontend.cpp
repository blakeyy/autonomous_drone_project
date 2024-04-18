/*
 * Frontend.cpp
 *
 *  Created on: 9 Dec 2020
 *      Author: sleutene
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <ros/ros.h>

#include <brisk/brisk.h>

#include <arp/Frontend.hpp>

#ifndef CV_AA
#define CV_AA cv::LINE_AA // maintains backward compatibility with older OpenCV
#endif

#ifndef CV_BGR2GRAY
#define CV_BGR2GRAY cv::COLOR_BGR2GRAY // maintains backward compatibility with older OpenCV
#endif

namespace arp {

void drawMatchedKeypointsOnImage(const DetectionVec& detections, cv::Mat& visualisationImage)
{  
  // Draw all matched (and inlier) keypoints in green over the red ones.
  std::vector<cv::KeyPoint> matchedKeypoints;
  for (const auto& detection : detections){
    // cv::drawMarker(visualisationImage, cv::Point2d{detection.keypoint[0],detection.keypoint[1]}, cv::Scalar(0,255,0));
    cv::KeyPoint matchedKeypoint(detection.keypoint[0], detection.keypoint[1], 1.0);
    matchedKeypoints.push_back(matchedKeypoint);
  }
  cv::drawKeypoints(visualisationImage, matchedKeypoints, visualisationImage, cv::Scalar(0,255,0));
}
  

Frontend::Frontend(int imageWidth, int imageHeight,
                                   double focalLengthU, double focalLengthV,
                                   double imageCenterU, double imageCenterV,
                                   double k1, double k2, double p1, double p2,
                                   double mapCamFocalLength,
                                   FrontendThresholds thresholds,
                                   FeatureDetectionParams featureDetectionParams) :
  camera_(imageWidth, imageHeight, focalLengthU, focalLengthV, imageCenterU,
          imageCenterV,
          arp::cameras::RadialTangentialDistortion(k1, k2, p1, p2)),
  thresholds_(std::move(thresholds)),
  featureDetectionParams_(std::move(featureDetectionParams))
{
  camera_.initialiseUndistortMaps();

  // also save for OpenCV RANSAC later
  cameraMatrix_ = cv::Mat::zeros(3, 3, CV_64FC1);
  cameraMatrix_.at<double>(0,0) = focalLengthU;
  cameraMatrix_.at<double>(1,1) = focalLengthV;
  cameraMatrix_.at<double>(0,2) = imageCenterU;
  cameraMatrix_.at<double>(1,2) = imageCenterV;
  cameraMatrix_.at<double>(2,2) = 1.0;
  distCoeffs_ = cv::Mat::zeros(1, 4, CV_64FC1);
  distCoeffs_.at<double>(0) = k1;
  distCoeffs_.at<double>(1) = k2;
  distCoeffs_.at<double>(2) = p1;
  distCoeffs_.at<double>(3) = p2;
  
  // BRISK detector and descriptor
  // detector_.reset(new brisk::ScaleSpaceFeatureDetector<brisk::HarrisScoreCalculator>(35, 5, 70, 750));
  detector_.reset(new brisk::ScaleSpaceFeatureDetector<brisk::HarrisScoreCalculator>(
    featureDetectionParams_.uniformityRadius, 
    featureDetectionParams_.octaves, 
    featureDetectionParams_.absoluteThreshold,
    featureDetectionParams_.maxNumKpt));
  extractor_.reset(new brisk::BriskDescriptorExtractor(true, false));
  
  // leverage camera-aware BRISK (caution: needs the *_new* maps...)
  cv::Mat rays = cv::Mat(imageHeight, imageWidth, CV_32FC3);
  cv::Mat imageJacobians = cv::Mat(imageHeight, imageWidth, CV_32FC(6));
  for (int v=0; v<imageHeight; ++v) {
    for (int u=0; u<imageWidth; ++u) {
      Eigen::Vector3d ray;
      Eigen::Matrix<double, 2, 3> jacobian;
      if(camera_.backProject(Eigen::Vector2d(u,v), &ray)) {
        ray.normalize();
      } else {
        ray.setZero();
      }
      rays.at<cv::Vec3f>(v,u) = cv::Vec3f(ray[0],ray[1],ray[2]);
      Eigen::Vector2d pt;
      if(camera_.project(ray, &pt, &jacobian)
         ==cameras::ProjectionStatus::Successful) {
        cv::Vec6f j;
        j[0]=jacobian(0,0);
        j[1]=jacobian(0,1);
        j[2]=jacobian(0,2);
        j[3]=jacobian(1,0);
        j[4]=jacobian(1,1);
        j[5]=jacobian(1,2);
        imageJacobians.at<cv::Vec6f>(v,u) = j;
      }
    }
  }
  std::static_pointer_cast<cv::BriskDescriptorExtractor>(extractor_)->setCameraProperties(rays, imageJacobians, mapCamFocalLength);
}

bool  Frontend::loadMap(std::string path) {
  std::ifstream mapfile(path);
  if(!mapfile.good()) {
    return false;
  }
  
  // read each line
  std::string line;
  std::set<uint64_t> lmIds;
  uint64_t poseId = 0;
  LandmarkVec landmarks;
  while (std::getline(mapfile, line)) {

    // Convert to stringstream
    std::stringstream ss(line);
    
    if(0==line.compare(0, 7,"frame: ")) {
      // store previous set into map
      landmarks_[poseId] = landmarks;
      // get pose id:
      std::stringstream frameSs(line.substr(7,line.size()-1));
      frameSs >> poseId;
      if(!frameSs.eof()) {
        std::string covisStr;
        frameSs >> covisStr; // comma
        frameSs >> covisStr;
        if(0==covisStr.compare("covisibilities:")) {
          while(!frameSs.eof()) {
            uint64_t covisId;
            frameSs >> covisId;
            covisibilities_[poseId].insert(covisId);
          }
        }
      }
      // move to filling next set of landmarks
      landmarks.clear();
    } else {
      if(poseId>0) {
        Landmark landmark;
      
        // get keypoint idx
        size_t keypointIdx;
        std::string keypointIdxString;
        std::getline(ss, keypointIdxString, ',');
        std::stringstream(keypointIdxString) >> keypointIdx;
        
        // get landmark id
        uint64_t landmarkId;
        std::string landmarkIdString;
        std::getline(ss, landmarkIdString, ',');
        std::stringstream(landmarkIdString) >> landmarkId;
        landmark.landmarkId = landmarkId;
        
        // read 3d position
        for(int i=0; i<3; ++i) {
          std::string coordString;
          std::getline(ss, coordString, ',');
          double coord;
          std::stringstream(coordString) >> coord;
          landmark.point[i] = coord;
        }

        // Get descriptor
        std::string descriptorstring;
        std::getline(ss, descriptorstring);
        landmark.descriptor = cv::Mat(1,48,CV_8UC1);
        for(int col=0; col<48; ++col) {
          uint32_t byte;
          std::stringstream(descriptorstring.substr(2*col,2)) >> std::hex >> byte;
          landmark.descriptor.at<uchar>(0,col) = byte;
        }
        landmark.landmarkId = landmarkId;
        lmIds.insert(landmarkId);
        landmarks.push_back(landmark);
      }      
    } 
  }
  if(poseId>0) {
    // store into map
    landmarks_[poseId] = landmarks;
  }
  std::cout << "loaded " << lmIds.size() << " landmarks from " << landmarks_.size() << " poses." << std::endl;
  return lmIds.size() > 0;
}

bool Frontend::loadDBoW2Voc(std::string path) {
  std::cout << "Loading DBoW2 vocabulary from " << path << std::endl;
  dBowVocabulary_.load(path);
  // Hand over vocabulary to dataset. false = do not use direct index:
  dBowDatabase_.setVocabulary(dBowVocabulary_, false, 0);
  std::cout << "loaded DBoW2 vocabulary with " << dBowVocabulary_.size() << " words." << std::endl;
  return true; 
}

DBoW2::FBrisk::TDescriptor transformDescriptor(const cv::Mat& descriptor)
{
  DBoW2::FBrisk::TDescriptor newDescriptor{48};
  for (size_t i = 0; i < 48; i++) {
    newDescriptor.push_back(descriptor.at<unsigned char>(0, i));
  }
  return newDescriptor;
}

bool Frontend::buildDBoW2Database()
{
  // std::vector<DBoW2::FBrisk::TDescriptor> features;
  for (const std::pair<uint64_t, LandmarkVec>& lmByPose : landmarks_)  {
    std::vector<DBoW2::FBrisk::TDescriptor> features;
    for (const Landmark& lm : lmByPose.second) {
      features.push_back(transformDescriptor(lm.descriptor));
    }
    DBoW2::EntryId newId = dBowDatabase_.add(features);
    posesByDBoWEntry_.insert({newId, lmByPose.first});
  }

  return true;
}

int Frontend::detectAndDescribe(
    const cv::Mat &grayscaleImage, const Eigen::Vector3d &extractionDirection,
    std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors) const
{

  // run BRISK detector
  detector_->detect(grayscaleImage, keypoints);

  // run BRISK descriptor extractor
  // orient the keypoints according to the extraction direction:
  Eigen::Vector3d ep;
  Eigen::Vector2d reprojection;
  Eigen::Matrix<double, 2, 3> Jacobian;
  Eigen::Vector2d eg_projected;
  for (size_t k = 0; k < keypoints.size(); ++k) {
    cv::KeyPoint& ckp = keypoints[k];
    const Eigen::Vector2d kp(ckp.pt.x, ckp.pt.y);
    // project ray
    camera_.backProject(kp, &ep);
    // obtain image Jacobian
    camera_.project(ep+extractionDirection.normalized()*0.001, &reprojection);
    // multiply with gravity direction
    eg_projected = reprojection-kp;
    double angle = atan2(eg_projected[1], eg_projected[0]);
    // set
    ckp.angle = angle / M_PI * 180.0;
  }
  extractor_->compute(grayscaleImage, keypoints, descriptors);

  return keypoints.size();
}

bool Frontend::ransac(const std::vector<cv::Point3d>& worldPoints, 
                      const std::vector<cv::Point2d>& imagePoints, 
                      kinematics::Transformation & T_CW, std::vector<int>& inliers) const {
  if(worldPoints.size() != imagePoints.size()) {
    return false;
  }

  // this must stay in even if, theoretically, VI-EKF works with 2 matches
  // because cv::solvePnPRansac asserts npoints >= 4
  if(worldPoints.size()<5) {
    return false; // not realiable enough
  }

  inliers.clear();
  cv::Mat rvec, tvec;
  bool ransacSuccess = cv::solvePnPRansac(
      worldPoints, imagePoints, cameraMatrix_, distCoeffs_,
      rvec, tvec, false, 100, 5.0, 0.99, inliers, cv::SOLVEPNP_EPNP);	

  // set pose
  cv::Mat R = cv::Mat::zeros(3, 3, CV_64FC1);
  cv::Rodrigues(rvec, R);
  Eigen::Matrix4d T_CW_mat = Eigen::Matrix4d::Identity();
  for(int i=0; i<3; i++) {
    T_CW_mat(i,3) = tvec.at<double>(i);
    for(int j=0; j<3; j++) {
      T_CW_mat(i,j) = R.at<double>(i,j);
    }
  }
  T_CW = kinematics::Transformation(T_CW_mat);

  return ransacSuccess && (double(inliers.size())/double(imagePoints.size()) > 0.7);
}

/// \brief Helper function to wrap image and landmark data in a Detection struct.
Detection createDetectionFromMatchedData(const cv::Point2d& imagePoint, const cv::Point3d& landmark, size_t landmarkId)
{
  Detection detection;
  detection.keypoint[0] = imagePoint.x;
  detection.keypoint[1] = imagePoint.y;
  detection.landmark[0] = landmark.x;
  detection.landmark[1] = landmark.y;
  detection.landmark[2] = landmark.z;
  detection.landmarkId = landmarkId;
  return detection;
}

bool Frontend::detectAndMatch(const cv::Mat& image, const Eigen::Vector3d & extractionDirection, 
                              DetectionVec & detections, kinematics::Transformation & T_CW, 
                              cv::Mat & visualisationImage, bool needsReInitialisation)
{
  detections.clear(); // make sure empty

  // to gray:
  cv::Mat grayScale;
  cv::cvtColor(image, grayScale, CV_BGR2GRAY);

  // run BRISK detector and descriptor extractor:
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  detectAndDescribe(grayScale, extractionDirection, keypoints, descriptors);

  // Draw all found keypoints in red onto the image:
  cv::drawKeypoints(image, keypoints, visualisationImage, cv::Scalar(0,0,255));

  // IDs of keyframes to search
  std::vector<uint64_t> keyframeIds;

  if (lost_) {

    // find similar keyframes to only search potentially visible landmarks
    std::vector<DBoW2::FBrisk::TDescriptor> features;
    for (size_t i = 0; i < descriptors.rows; i++) {
      features.push_back(transformDescriptor(descriptors.row(i)));
    }

    // based on the features, run place recognition using BoW db query
    DBoW2::QueryResults dBoWResults;
    dBowDatabase_.query(features, dBoWResults, thresholds_.maxBoWResults);
    for (const DBoW2::Result& result : dBoWResults) {
      keyframeIds.push_back(posesByDBoWEntry_.at(result.Id));
    }

  }  else {

    // look at current keyframe and covisibilities
    keyframeIds.push_back(activeKeyframe_);
    const std::set<uint64_t>& covisibilities = covisibilities_.at(activeKeyframe_);
    keyframeIds.insert(std::end(keyframeIds), std::begin(covisibilities), std::end(covisibilities));
    
  }

  // Match against landmarks in gathered keyframe IDs and infer live frame from number of matches
  std::vector<cv::Point2d> matchedImagePoints;
  std::vector<cv::Point3d> matchedLandmarkPoints;
  std::vector<uint64_t> matchedLandmarkIDs;
  unsigned int mostMatches = 0;
  uint64_t frameWithMostMatches = 0;

  // go through all poses
  for (const auto& pose : keyframeIds) {
    const LandmarkVec relevantLandmarks = landmarks_.at(pose);
    unsigned int matches = 0;

    // go through all landmarks seen from this pose
    for(const auto& lm : relevantLandmarks) {

      // the pose prior T_CW is valid if needsReInitialisation is false
      if (!needsReInitialisation) {
        // "rule out potential matches if the landmarks are not projecting into the live frame"
        Eigen::Vector2d imagePoint;
        auto status = camera_.project(T_CW * lm.point, &imagePoint);
        if (status != arp::cameras::ProjectionStatus::Successful) { 
          continue;
        }
      }

      // go through all keypoints in the frame
      for(size_t k = 0; k < keypoints.size(); ++k) { 
        uchar* keypointDescriptor = descriptors.data + k*48; // descriptors are 48 bytes long
        const float dist = brisk::Hamming::PopcntofXORed(
              keypointDescriptor, lm.descriptor.data, 3); // compute desc. distance: 3 for 3x128bit (=48 bytes)
        
        // check if a match and add them to the "matched" vectors.
        if(dist < 60.0) {
          // match! add world point of landmark (+ ID) and image point of keypoint the respective vector to use them in ransac.
          matches += 1;
          cv::Point2d matchedImagePoint;
          matchedImagePoint.x = keypoints[k].pt.x;
          matchedImagePoint.y = keypoints[k].pt.y;
          matchedImagePoints.push_back(matchedImagePoint);
          cv::Point3d matchedLandmarkPoint;
          matchedLandmarkPoint.x = lm.point[0];
          matchedLandmarkPoint.y = lm.point[1];
          matchedLandmarkPoint.z = lm.point[2];
          matchedLandmarkPoints.push_back(matchedLandmarkPoint);
          matchedLandmarkIDs.push_back(lm.landmarkId);
        }
      }

    }

    if (matches > mostMatches) {
      mostMatches = matches;
      frameWithMostMatches = pose;
    }
  }

  // If we found a sufficient number of matches, set the new active keyframe. Otherwise, we 
  // must re-localize  using the BoW place recognition in the next call to detectAndMatch
  if (mostMatches > thresholds_.minMatches) {
    activeKeyframe_ = frameWithMostMatches;
    //std::cout << "Active key frame=" << activeKeyframe_ << ", matches=" << mostMatches << std::endl;
    lost_ = false;
  } else {
    lost_ = true;
    return false;
  }

  // run RANSAC (to remove outliers and get pose T_CW estimate)
  std::vector<int> inliers;
  bool ransacSuccess = ransac(matchedLandmarkPoints, matchedImagePoints, T_CW, inliers);

  if(ransacSuccess) {
    // set detections (only use inliers)
    for (const auto& ptID : inliers) {
      detections.push_back(createDetectionFromMatchedData(matchedImagePoints[ptID], matchedLandmarkPoints[ptID], matchedLandmarkIDs[ptID]));
    }
  } else {
    // set detections (all, because ransac failed but we have enough matched keypoints)
    for (int i = 0; i < matchedImagePoints.size(); i++) { 
      detections.push_back(createDetectionFromMatchedData(matchedImagePoints[i], matchedLandmarkPoints[i], matchedLandmarkIDs[i]));
    }
  }

  // visualise by painting matched keypoints into visualisationImage
  drawMatchedKeypointsOnImage(detections, visualisationImage);
  
  // We can use the RANSAC result without the outlier rejection,
  // but only if we don't need to reintialize.
  return !needsReInitialisation || ransacSuccess;
}

}  // namespace arp

