#include "astar_planner.h"
#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"

AstarPlanner::AstarPlanner() : Node("astarplanner")
{
    occupancy_grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, std::bind(&AstarPlanner::convertToBinaryGrid, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Node Started");
}

// Each node represents a point on the grid.
// The cost is the distance travelled so far from the start
// The heuristic is a function define later that calcualtes the eclidean distance to the goal
// Parent points to the previous node
// the total cost is how far it has travelled and how far it is from the goal. I.E, low score is good

struct AstarPlanner::Point
{
    int x, y;
    float cost, heuristic;
    Point *parent;

    Point(int x, int y, float cost, float heuristic, Point *parent = nullptr)
        : x(x), y(y), cost(cost), heuristic(heuristic), parent(parent) {}

    float totalCost() const { return cost + heuristic; }
};

// This function is what is used to figure out what is the highest priority to search the next node.
// I.E if this node is a low cost, search from there first
struct AstarPlanner::ComparePoint
{
    bool operator()(const Point *a, const Point *b)
    {
        return a->totalCost() > b->totalCost();
    }
};

// Eclidiean distace from point to point
float AstarPlanner::eclidDist(int x1, int y1, int x2, int y2)
{
    return std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

void AstarPlanner::convertToBinaryGrid(const nav_msgs::msg::OccupancyGrid &map)
{
    int width = map.info.width;
    int height = map.info.height;
    origin_x = map.info.origin.position.x;
    origin_y = map.info.origin.position.y;
    resolution = map.info.resolution;
    RCLCPP_INFO(this->get_logger(), "origin x = %.2f, y = %.2f, resolution = %.2f", origin_x, origin_y, resolution);


    std::vector<std::vector<int>> binaryGrid(height, std::vector<int>(width, 0));

    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            int index = y * width + x;
            int value = map.data[index];

            // Mark as 1 if occupied or unknown
            if (value == 100 || value == -1)
                binaryGrid[y][x] = 1;
            else
                binaryGrid[y][x] = 0;
        }
    }
    RCLCPP_INFO(this->get_logger(), "convert to binary");
    auto path = AstarPlanner::aStarSearch(0, 0.5, 0, -2, binaryGrid);
    AstarPlanner::saveGridAsImage(binaryGrid, "Binary Grid", path);
}


// currently image is flipped in the x and y axis. 
void AstarPlanner::saveGridAsImage(const std::vector<std::vector<int>> &grid, const std::string &filename, const std::vector<Point*>& path)
{
    RCLCPP_INFO(this->get_logger(), "image creating");
    int height = grid.size();
    int width = grid[0].size();
    
    // create an image in colour scale 
    cv::Mat image(height, width, CV_8UC3);

    cv::circle(image, cv::Point(0, 0), 3, cv::Scalar(255, 255, 0), -1); // Cyan dot for origin

    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            if (grid[y][x]) {
                // Obstacle (black)
                image.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0); // Black for obstacles
            } else {
                // Free space (white)
                image.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255); // White for free space
            }
        }
    }
    for (const auto& point : path)
    {
        RCLCPP_INFO(this->get_logger(), "Path point x: %d, Path point y: %d", point->x, point->y);
        image.at<cv::Vec3b>(point->y, point->x) = cv::Vec3b(0, 0, 255);  // Red (BGR format)
    }
    // // Draw start position in GREEN
    // cv::circle(image, cv::Point(startX, startY), 3, cv::Scalar(0, 255, 0), -1);

    // // Draw goal position in RED
    // cv::circle(image, cv::Point(goalX, goalY), 3, cv::Scalar(0, 0, 255), -1);
    RCLCPP_INFO(this->get_logger(), "image saved");
    std::string filePath = filename + ".png";
    cv::imwrite(filePath, image);
}

// Impliment A* algorithim. Take an initial start X and Y positon, a goal X and Y Position and a map
// NOTE: x and y positions are relative to the Rviz map, internal conversion are done to change positions to grid locations
std::vector<AstarPlanner::Point *> AstarPlanner::aStarSearch(double startX, double startY, double goalX, double goalY, std::vector<std::vector<int>> &grid)
{
    std::priority_queue<Point *, std::vector<Point *>, ComparePoint> openList;
    std::unordered_map<int, Point *> visited;

    int gridStartX = (startX - origin_x) / resolution;
    int gridStartY = (startY - origin_y) / resolution;
    int gridGoalX = (goalX - origin_x) / resolution;
    int gridGoalY = (goalY - origin_y) / resolution;
    int width = grid[0].size(); // Needed for visited key calculation
    RCLCPP_INFO(this->get_logger(), "start x = %.2f, y = %.2f, grid start x = %d, y = %d ", startX, startY, gridStartX, gridStartY);
    RCLCPP_INFO(this->get_logger(), "goal x = %.2f, y = %.2f, grid goal x = %d, y = %d ", goalX, goalY, gridGoalX, gridGoalY);
    RCLCPP_INFO(this->get_logger(), "Map Resolution = %.2f", resolution);

    // Check start and goal validity
    if (grid[gridStartY][gridStartX] != 0 || grid[gridGoalY][gridGoalX] != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Start or Goal is in an obstacle");
        return {};
    }

    // Create an initial node at the start position with a cost of zero as it hasn't moved yet
    Point *start = new Point(gridStartX, gridStartY, 0, eclidDist(gridStartX, gridStartY, gridGoalX, gridGoalY));
    openList.push(start);

    // Define the movements allowed, I.E. up down left and right.
    // This can be modified if more dirrections are required, I.E diagonal - if the resolution gets big enough, maybe this can be variable?
    std::vector<std::pair<int, int>> directions = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};

    // Start the A* loop, this will continue to run while there is nodes in the priority queue, as the initial node was pushed back above
    while (!openList.empty())
    {
        Point *current = openList.top();
        openList.pop();

        // Skip if already visited
        int currentKey = current->y * width + current->x;
        if (visited.count(currentKey))
            continue;

        visited[currentKey] = current;

        // check if we have reached the goal
        if (current->x == gridGoalX && current->y == gridGoalY)
        {

            // Reconstruct path
            std::vector<Point *> path;
            while (current)
            {
                RCLCPP_INFO(this->get_logger(), "New Node");
                path.push_back(current);
                current = current->parent;
            }
            return path;
        }
        // Generate Neighbour nodes
        for (auto [dx, dy] : directions)
        {
            int nx = current->x + dx;
            int ny = current->y + dy;
            // Check if the neighbour node is valid I.E within the bounds of the world and not an obstacle
            if (nx >= 0 && ny >= 0 && ny < static_cast<int>(grid.size()) && nx < static_cast<int>(grid[0].size()) && grid[ny][nx] == 0)
            {
                int neighborKey = ny * width + nx;
                if (!visited.count(neighborKey))
                {
                    float newCost = current->cost + 1;
                    Point *neighbor = new Point(nx, ny, newCost, eclidDist(nx, ny, gridGoalX, gridGoalY), current);
                    RCLCPP_ERROR(this->get_logger(), "Bad Node");
                    openList.push(neighbor);
                }
            }
        }
    }
    return {};
}

// Main function to test A*
int main(int argc, char *argv[])
{
    // std::vector<std::vector<int>> grid = {
    //     {0, 0, 0, 0, 1},
    //     {0, 1, 1, 0, 1},
    //     {0, 0, 0, 0, 0},
    //     {1, 1, 0, 1, 1},
    //     {0, 0, 0, 0, 0}
    // };

    // auto path = aStarSearch(0, 0, 4, 4, grid);

    // for (auto node : path) {
    //     std::cout << "(" << node->x << ", " << node->y << ") <- ";
    // }
    // std::cout << "Start\n";

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AstarPlanner>());
    rclcpp::shutdown();
    return 0;

    return 0;
}
