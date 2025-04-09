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

class AstarPlanner : public rclcpp::Node {
    
    public:
    AstarPlanner();
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_sub_;

    private:
    struct Point;
    struct ComparePoint;
    float eclidDist(int x1, int y1, int x2, int y2);
    void convertToBinaryGrid(const nav_msgs::msg::OccupancyGrid& map);
    void saveGridAsImage(const std::vector<std::vector<int>>& grid, const std::string& filename, const std::vector<Point*>& path);
    std::vector<Point*> aStarSearch(double startX, double startY, double goalX, double goalY, std::vector<std::vector<int>>& grid);
    double origin_x;
    double origin_y;
    double resolution;

};

#endif // PATHPLANNER_HPP