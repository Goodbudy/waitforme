#ifndef ASTAR_PLANNER_HPP
#define ASTAR_PLANNER_HPP   

#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <string>   
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class AstarPlanner : public rclcpp::Node {
    
    public:
    AstarPlanner();

    // ROS interfaces
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr     odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr           path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_marker_pub_;

    // Grid and pose data
    std::vector<std::vector<int>> grid_;
    double origin_x_, origin_y_, resolution_;
    size_t width_, height_;
    bool map_ready_;
    double current_x_, current_y_;
    double obstacle_buffer_radius_;

    private:
    struct Point;
    struct ComparePoint;
    float eclidDist(int x1, int y1, int x2, int y2);
    std::vector<std::pair<double, double>> convertGridToWorld(std::vector<AstarPlanner::Point *>);
    void applyObstacleBuffering(double buffer);
    void publishPath(std::vector<std::pair<double, double>>);
    void saveGridAsImage(const std::vector<std::vector<int>>& grid, const std::string& filename, const std::vector<Point*>& path);
    void publishPathToRViz(const std::vector<std::pair<double, double>> &);
    std::vector<Point*> aStarSearch(double startX, double startY, double goalX, double goalY);
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
};

#endif // PATHPLANNER_HPP