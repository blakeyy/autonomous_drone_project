#include <OccupancyMap.h>
#include <fstream>
#include <ros/ros.h>

OccupancyMap::OccupancyMap(const std::string& filename)
{
  // open the file:
  std::ifstream mapFile(filename, std::ios::in | std::ios::binary);
  if(!mapFile.is_open()) {
  ROS_FATAL_STREAM("could not open map file " << filename);
  }
  // first read the map size along all the dimensions:
  // int sizes[3];
  int* sizes = dimensions_.data();
  if(!mapFile.read((char*)sizes, 3*sizeof(int))) {
  ROS_FATAL_STREAM("could not read map file " << filename);
  }
  // now read the map data
  mapData_.reserve(sizes[0]*sizes[1]*sizes[2]);
  if(!mapFile.read((char*)mapData_.data(), sizes[0]*sizes[1]*sizes[2])) {
  ROS_FATAL_STREAM("could not read map file " << filename);
  }
  mapFile.close();
  // now wrap it with a cv::Mat for easier access:
  wrapped_ = cv::Mat{3, sizes, CV_8SC1, mapData_.data()};

  std::cout << "Loaded occupancy map \"" << filename << "\"." << std::endl;

  occupancy_map_ = cv::Mat(3, sizes, CV_64FC1);
  cv::Mat occupancy_map1_ = cv::Mat(3, sizes, CV_64FC1);

  for(int i=0; i<sizes[0]; i++ ){
    for(int j=0; j<sizes[1]; j++ ){
      for(int k=0; k<sizes[2]; k++ ){
        occupancy_map_.at<double>(i,j,k) =  static_cast<double>(wrapped_.at<char>(i,j,k));
        //std::cout << "map " <<  occupancy_map_.at<double>(i,j,k) << std::endl;
      }
    }
  }

  //inflate
  int inflation = 0;
  ros::NodeHandle nh; //is it safe?
  if (!nh.getParam("arp_node/inflation", inflation)) ROS_FATAL("Could not find inflation parameter.");

  for(int n=0; n<inflation; n++){
    std::cout << "inflating map" << std::endl;

    //inflate the occupancy
    for(int i=1; i<sizes[0]-1; i++ ){
      for(int j=1; j<sizes[1]-1; j++ ){
        for(int k=1; k<sizes[2]-1; k++ ){
          for(int dx=-1; dx<2; dx++){
            for(int dy=-1; dy<2; dy++){
              for(int dz=-1; dz<2; dz++){
                occupancy_map1_.at<double>(i,j,k) =  std::max(occupancy_map_.at<double>(i,j,k),occupancy_map_.at<double>(i+dx,j+dy,k+dz));
              }
            }
          }
        }
      }
    }
    //occupancy_map_ = occupancy_map1_.clone();

    for(int i=1; i<sizes[0]-1; i++ ){
      for(int j=1; j<sizes[1]-1; j++ ){
        for(int k=1; k<sizes[2]-1; k++ ){
          occupancy_map_.at<double>(i,j,k) = occupancy_map1_.at<double>(i,j,k);
        }
      }
    }

  }


}

OccupancyMap::operator const cv::Mat&() const
{
    return wrapped_;
}

const OccupancyMap::Dimensions& OccupancyMap::dimensions() const
{
  return dimensions_;
}

double OccupancyMap::at(int x, int y, int z) const
{
  // TODO pull out using data_.at<double>()?
  //return static_cast<double>(wrapped_.at<char>(x, y, z));
  return occupancy_map_.at<double>(x, y, z);
}