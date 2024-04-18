#include <Planner.h>
#
namespace Planner
{

class Vertex {
public:
    Vertex(std::vector<int> a, float b) : ijk(a), total_cost(b){}
    //grid index
    std::vector<int> ijk; //or Eigen::Vector3i?
    float total_cost;
};

struct greater1 {
    bool operator()(const Vertex& a, const Vertex& b) const {
        return a.total_cost > b.total_cost;
    }
};

void addWayPoint(std::vector<int> gridPoint, 
                 std::deque<arp::Autopilot::Waypoint> &waypoints,
                 const OccupancyMap::Dimensions& dimensions)
{
    std::cout << "PATH: (" << gridPoint[0] << "," << gridPoint[1]  << "," << gridPoint[2] << ")" << std::endl;
    int const *const sizes_ = dimensions.data(); 
    arp::Autopilot::Waypoint p;
    p.x = (double(gridPoint[0]) - ((double(sizes_[0]) - 1) / 2)) / 10;
    p.y = (double(gridPoint[1]) - ((double(sizes_[1]) - 1) / 2)) / 10;
    p.z = (double(gridPoint[2]) - ((double(sizes_[2]) - 1) / 2)) / 10;
    p.yaw = 0.0;
    p.posTolerance = 0.1;
    waypoints.push_front(p);
}

std::deque<arp::Autopilot::Waypoint> planFlight(const Eigen::Vector3d &start,
                                                const Eigen::Vector3d &goal,
                                                const OccupancyMap &occupancyMap)
{
    
    std::cout << "planning flight " << std::endl;

    // creating and aliasing variables to glue together vest's and gümüs' code
    const auto& dimensions = occupancyMap.dimensions();
    int const *const sizes_ = dimensions.data();
    const auto pointA_ = start;
    const auto pointB_ = goal;
    std::vector<Vertex> open_vertex_vector_;

    cv::Mat dist_ = cv::Mat(3, sizes_, CV_64FC1, cv::Scalar::all(INFINITY));
    cv::Mat totalDistEst_ = cv::Mat(3, sizes_, CV_64FC1, cv::Scalar::all(INFINITY));
    // totalDistEst_ = cv::Mat(3, sizes_, CV_64FC1, INFINITY); ?
    // prev_ = cv::Mat(3, sizes_, CV_64FC1, cv::Scalar::all(NAN));
    // prev_[sizes_[0]][sizes_[1]][sizes_[2]];

    // std::cout <<  "prev_ value: "<< prev_.at<double>(0,0,0) << std::endl;

    // if(prev_.at<double>(0,0,0) == NAN){
    //   std::cout << "NAN" << std::endl;
    // }

    std::map<std::vector<int>, std::vector<int>> path;

    int start_i = std::round(pointA_[0] / 0.1 + double(sizes_[0] - 1) / 2.0);
    int start_j = std::round(pointA_[1] / 0.1 + double(sizes_[1] - 1) / 2.0);
    int start_k = std::round(pointA_[2] / 0.1 + double(sizes_[2] - 1) / 2.0);
    std::vector<int> start_ijk;
    start_ijk = {start_i, start_j, start_k};
    std::cout << "what: " << std::endl;

    int goal_i = std::round(pointB_[0] / 0.1 + double(sizes_[0] - 1) / 2.0);
    int goal_j = std::round(pointB_[1] / 0.1 + double(sizes_[1] - 1) / 2.0);
    int goal_k = std::round(pointB_[2] / 0.1 + double(sizes_[2] - 1) / 2.0);
    std::vector<int> goal_ijk;
    goal_ijk = {goal_i, goal_j, goal_k};

    dist_.at<double>(start_i, start_j, start_k) = 0;
    totalDistEst_.at<double>(start_i, start_j, start_k) = std::sqrt(std::pow((start_i - goal_i), 2) + std::pow((start_j - goal_j), 2) + std::pow((start_k - goal_k), 2));

    bool reached = false;

    Vertex start_vertex(start_ijk, 0);
    open_vertex_vector_.push_back(start_vertex);

    std::cout << "start: " << start_ijk[0] << " " << start_ijk[1] << " " << start_ijk[2] << " "
                << " goal: " << goal_ijk[0] << " " << goal_ijk[1] << " " << goal_ijk[2] << " " << std::endl;

    int occ_thr = -3;
    ros::NodeHandle nh; //is it safe?
    if (!nh.getParam("arp_node/occ_thr", occ_thr)) ROS_FATAL("Could not find occ_thr parameter.");

    if(occupancyMap.at(start_ijk[0], start_ijk[1], start_ijk[2]) >= occ_thr){
        ROS_ERROR("start is invalid, change the point or increase threshold");

    } 

    while (!open_vertex_vector_.empty())
    {
        // what if there is no path?

        Vertex u = open_vertex_vector_[0];
        std::pop_heap(open_vertex_vector_.begin(), open_vertex_vector_.end(), greater1());
        open_vertex_vector_.pop_back();

        std::cout << "pop vertex: " << u.ijk[0] << " " << u.ijk[1] << " " << u.ijk[2] << std::endl;

        if (u.ijk == goal_ijk)
        {
            std::cout << "path found" << std::endl;
            reached = true;
            break;
        }

        std::vector<int> v_ijk{0, 0, 0};
        double alt;
        // for each neighbour v of u:
        for (int dx = -1; dx < 2; dx++)
        {
            for (int dy = -1; dy < 2; dy++)
            {
                for (int dz = -1; dz < 2; dz++)
                {
                    if (dx != 0 || dy != 0 || dz != 0)
                    {
                        v_ijk[0] = u.ijk[0] + dx;
                        v_ijk[1] = u.ijk[1] + dy;
                        v_ijk[2] = u.ijk[2] + dz;
                        if (v_ijk[0] >= 0 && v_ijk[0] < sizes_[0] && v_ijk[1] >= 0 && v_ijk[1] < sizes_[1] && v_ijk[2] >= 0 && v_ijk[2] < sizes_[2] && occupancyMap.at(v_ijk[0], v_ijk[1], v_ijk[2]) < occ_thr // if it is unblocked TODO: there is a problem in cost map
                            //&& closedList_.at<double>(v_ijk[0], v_ijk[1], v_ijk[2]) == -1
                        )
                        {

                            if (occupancyMap.at(v_ijk[0], v_ijk[1], v_ijk[2]) > 0)
                            {
                                std::cout << "there is an obstacle TODO: skip this point" << std::endl;
                            }
                            //std::cout << "cost: " << occupancyMap.at(v_ijk[0], v_ijk[1], v_ijk[2]) << std::endl;

                            alt = dist_.at<double>(u.ijk[0], u.ijk[1], u.ijk[2]) + std::sqrt(dx * dx + dy * dy + dz * dz);

                            if (alt < dist_.at<double>(v_ijk[0], v_ijk[1], v_ijk[2]))
                            {
                                dist_.at<double>(v_ijk[0], v_ijk[1], v_ijk[2]) = alt;
                                totalDistEst_.at<double>(v_ijk[0], v_ijk[1], v_ijk[2]) = alt + std::sqrt(std::pow((v_ijk[0] - goal_i), 2) + std::pow((v_ijk[1] - goal_j), 2) + std::pow((v_ijk[2] - goal_k), 2));

                                Vertex v(v_ijk, totalDistEst_.at<double>(v_ijk[0], v_ijk[1], v_ijk[2]));
                                // v.parent_vertex = &u;
                                open_vertex_vector_.push_back(v);

                                std::push_heap(open_vertex_vector_.begin(), open_vertex_vector_.end(), greater1());
                                // prev[v] ← u
                                path[v_ijk] = u.ijk;
                            }
                        }
                    }
                }
            }
        }
    } // while end

    // create waypoints using A*
    std::deque<arp::Autopilot::Waypoint> waypoints;

    if (!reached) {
        std::cout << "NO PATH AVAILABLE" << std::endl;
        return waypoints;
    }

    reached = false;

    auto temp_vec = path[goal_ijk];
    std::cout << "waypoint: " << goal_ijk[0] << " " << goal_ijk[1] << " " << goal_ijk[2] << std::endl;
    std::cout << "waypoint: " << temp_vec[0] << " " << temp_vec[1] << " " << temp_vec[2] << std::endl;

    addWayPoint(goal_ijk, waypoints, dimensions);
    addWayPoint(temp_vec, waypoints, dimensions);

    while (!reached)
    {
        if (temp_vec == start_ijk)
        {
            reached = true;
            //std::cout << "reached 2" << std::endl;
            break;
        }
        temp_vec = path[temp_vec];
        std::cout << "waypoint: " << temp_vec[0] << " " << temp_vec[1] << " " << temp_vec[2] << std::endl;
        addWayPoint(temp_vec, waypoints, dimensions);
    }

    return waypoints;
}

}
