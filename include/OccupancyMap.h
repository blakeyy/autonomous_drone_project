#pragma once

#include <memory>
#include <vector>
#include <opencv2/core.hpp>

/// \brief Takes care of RAII of the occupancy map data.
///
/// Provided code used raw pointers that need to be deleted explicitely.
/// Instead, this class keeps a reference to a vector that will drop the
/// data on its own once the owning OccupancyMap instance goes out of scope.
class OccupancyMap {
public:
    using Dimensions = std::array<int,3>;
    explicit OccupancyMap(const std::string& filename);
    operator const cv::Mat&() const;
    const Dimensions& dimensions() const;
    double at(int x, int y, int z) const;
private:
    std::vector<char> mapData_; ///< Points to 3d occupancy map
    Dimensions dimensions_; ///< The dimensions of the data
    cv::Mat wrapped_; ///< Wraps map data for easy access
    cv::Mat occupancy_map_; //inflated occupancy map
};