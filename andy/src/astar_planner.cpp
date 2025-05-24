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
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <yaml-cpp/yaml.h>
#include <opencv2/imgcodecs.hpp>
#include "rcpputils/filesystem_helper.hpp"

AstarPlanner::AstarPlanner() 
: Node("astarplanner", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
  map_ready_(false),
  current_x_(0.0),
  current_y_(0.0)
{
    RCLCPP_INFO(get_logger(), "start constructor: AstarPlanner node started");
    // QoS for goal subscription
    auto goal_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();

    std::string yaml_file;
    if (this->get_parameter("map_yaml_path", yaml_file)) {
        RCLCPP_INFO(this->get_logger(), "In constructor if statement: map_yaml_path param = %s", yaml_file.c_str());
        loadMapFromFile(yaml_file);
    } else {
        RCLCPP_ERROR(this->get_logger(), "No map_yaml_path provided.");
    }

    // Subscriptions and publishers — scoped by ROS namespace automatically
    // map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    //     "/map", 10,
    //     std::bind(&AstarPlanner::mapCallback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10,
        std::bind(&AstarPlanner::odomCallback, this, std::placeholders::_1));

    object_sub_ = create_subscription<visualization_msgs::msg::Marker>(
        "visualization_marker", 10,
        std::bind(&AstarPlanner::objectCallBack, this, std::placeholders::_1));

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "astar_goal", goal_qos,
        std::bind(&AstarPlanner::goalCallback, this, std::placeholders::_1));

    path_pub_ = create_publisher<nav_msgs::msg::Path>("planned_path", 10);
    path_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("astar_path", 10);

    // RCLCPP_INFO(get_logger(), "Waiting for map to become available...");
    // rclcpp::Rate wait_rate(10); // 10 Hz
    // int max_attempts = 50;
    // int attempts = 0;

    // while (rclcpp::ok() && !map_ready_ && attempts++ < max_attempts) {
    //     rclcpp::spin_some(this->get_node_base_interface());
    //     wait_rate.sleep();
    // }

    // if (map_ready_) {
    //     RCLCPP_INFO(get_logger(), "Map is ready. AstarPlanner fully initialized.");
    // } else {
    //     RCLCPP_WARN(get_logger(), "Map not received after waiting. AstarPlanner may fail to plan.");
    // }

    RCLCPP_INFO(get_logger(), "end cnostructor: AstarPlanner node started");
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

void AstarPlanner::loadMapFromFile(const std::string& yaml_file)
{
    RCLCPP_ERROR(this->get_logger(), "ENTER LOADING MAP");
    YAML::Node config = YAML::LoadFile(yaml_file);
    std::string image_file = config["image"].as<std::string>();
    // If image_file is relative, make it absolute based on the YAML path
    if (image_file[0] != '/') {
        auto yaml_dir = rcpputils::fs::path(yaml_file).parent_path();
        image_file = (yaml_dir / image_file).string();
    }

    resolution_ = config["resolution"].as<double>();
    origin_x_ = config["origin"][0].as<double>();
    origin_y_ = config["origin"][1].as<double>();

    cv::Mat image = cv::imread(image_file, cv::IMREAD_UNCHANGED);
    if (image.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load map image: %s", image_file.c_str());
        return;
    }

    width_ = image.cols;
    height_ = image.rows;

    grid_.resize(height_, std::vector<int>(width_, 0));
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            uint8_t pixel = image.at<uchar>(y, x);
            grid_[y][x] = (pixel < 250) ? 1 : 0;  // Threshold for obstacles
        }
    }

    original_grid_ = grid_;
    object_grid_ = grid_;
    map_ready_ = true;

    RCLCPP_ERROR(this->get_logger(), "Map loaded from file: %s (%zux%zu)", yaml_file.c_str(), width_, height_);
}

// void AstarPlanner::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
// {
//     RCLCPP_WARN(this->get_logger(), "Entered mapCallback");

//     width_ = msg->info.width;
//     height_ = msg->info.height;
//     origin_x_ = msg->info.origin.position.x;
//     origin_y_ = msg->info.origin.position.y;
//     resolution_ = msg->info.resolution;

//     grid_.assign(height_, std::vector<int>(width_, 0));
//     for (size_t y = 0; y < height_; ++y)
//     {
//         for (size_t x = 0; x < width_; ++x)
//         {
//             int idx = y * width_ + x;
//             int val = msg->data[idx];
//             grid_[y][x] = (val == 100 || val == -1) ? 1 : 0;
//         }
//     }
//     original_grid_ = grid_;
//     object_grid_ = grid_;
//     map_ready_ = true;
//     RCLCPP_ERROR(get_logger(), "Map received: %zux%zu", width_, height_);
//     // inflateObstacles();
//     // RCLCPP_INFO(get_logger(), "Inflated obstacles by %d cells", inflation_radius_cells_);
// }

void AstarPlanner::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    // RCLCPP_INFO(get_logger(), "Odom Callback Current x:  %.2f, Current y: %.2f", current_x_, current_y_);
}

void AstarPlanner::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // if (!map_ready_)
    // {
    //     RCLCPP_WARN(get_logger(), "Map not ready, cannot plan");
    //     return;
    // }

    double gx = msg->pose.position.x;
    double gy = msg->pose.position.y;
    RCLCPP_INFO(get_logger(),
                "Planning path from (%.2f, %.2f) → (%.2f, %.2f)",
                current_x_, current_y_, gx, gy);

    // 1) A* search on grid:
    // apply the buffer in m
    // pretend object is there
    // newObject(2,2.4,3);
    // obstacle_buffer_radius_ = 0.2;
    // applyObstacleBuffering(obstacle_buffer_radius_);
    auto grid_path = aStarSearch(current_x_, current_y_, gx, gy);

    // 2) Save occupancy + path to PNG:
    saveGridAsImage(grid_, "astar_map", grid_path);

    // 3) Convert and publish the world‐frame path:
    std::vector<std::pair<double, double>> world_path = convertGridToWorld(grid_path);
    std::reverse(world_path.begin(), world_path.end());
    publishPath(world_path);
    publishPathToRViz(world_path);
}

void AstarPlanner::objectCallBack(const visualization_msgs::msg::Marker msg)
{
    RCLCPP_INFO(this->get_logger(), "New Object Detected");
    // yes the grid x and y needs to be switched.
    int gridx = (msg.pose.position.x - origin_x_ - resolution_ / 2) / resolution_;
    int gridy = (msg.pose.position.y - origin_y_ - resolution_ / 2) / resolution_;
    RCLCPP_INFO(this->get_logger(), "gridx: %d , gridy: %d", gridx, gridy);
    // radius for cylinder is 0.15m
    if (msg.type == visualization_msgs::msg::Marker::CYLINDER)
    {
        int radius_grid = 0.15 / resolution_;
        int r2 = radius_grid * radius_grid;
        RCLCPP_ERROR(this->get_logger(), "Object Cylinder at X Pos: %.2f, YPos: %.2f", msg.pose.position.x, msg.pose.position.y);
        for (int dy = -radius_grid; dy <= radius_grid; ++dy)
        {
            for (int dx = -radius_grid; dx <= radius_grid; ++dx)
            {
                int nx = gridx + dx;
                int ny = gridy + dy;

                if (nx >= 0 && ny >= 0 && ny < height_ && nx < width_)
                {
                    if (dx * dx + dy * dy <= r2)
                    {
                        object_grid_[ny][nx] = 1;
                    }
                }
            }
        }
    }
    // radius roughly 0.2m
    else if (msg.type == visualization_msgs::msg::Marker::CUBE)
    {
        int radius_grid = 0.11 / resolution_;
        RCLCPP_ERROR(this->get_logger(), "Object Cube at X Pos: %.2f, YPos: %.2f", msg.pose.position.x, msg.pose.position.y);
        for (int dy = -radius_grid; dy <= radius_grid; ++dy)
        {
            for (int dx = -radius_grid; dx <= radius_grid; ++dx)
            {
                int nx = gridx + dx;
                int ny = gridy + dy;

                if (nx >= 0 && ny >= 0 && ny < height_ && nx < width_)
                {
                    object_grid_[ny][nx] = 1;
                }
            }
        }
    }
    else RCLCPP_ERROR(this->get_logger(), "Object is not a Cylinder or Cube");
    const std::vector<Point *> grid_path;
    saveGridAsImage(object_grid_, "astar_map", grid_path);
}
// Eclidiean distace from point to point
float AstarPlanner::eclidDist(int x1, int y1, int x2, int y2)
{
    return std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

// currently image is flipped in the x and y axis.
void AstarPlanner::saveGridAsImage(const std::vector<std::vector<int>> &grid, const std::string &filename, const std::vector<Point *> &path)
{
    RCLCPP_INFO(this->get_logger(), "image creating");

    // create an image in colour scale
    cv::Mat image(height_, width_, CV_8UC3);

    cv::circle(image, cv::Point(0, 0), 3, cv::Scalar(255, 255, 0), -1); // Cyan dot for origin

    for (size_t y = 0; y < height_; ++y)
    {
        for (size_t x = 0; x < width_; ++x)
        {
            if (grid[y][x])
            {
                // Obstacle (black)
                image.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0); // Black for obstacles
            }
            else
            {
                // Free space (white)
                image.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 255, 255); // White for free space
            }
        }
    }
    for (const auto &point : path)
    {
        // RCLCPP_INFO(this->get_logger(), "Path point x: %d, Path point y: %d", point->x, point->y);
        image.at<cv::Vec3b>(point->y, point->x) = cv::Vec3b(0, 0, 255); // Red (BGR format)ew
    }

    RCLCPP_INFO(this->get_logger(), "image saved");
    std::string filePath = filename + ".png";
    cv::Mat rotated;
    cv::Mat flip;
    cv::flip(image, flip, 0); // flip vertically to match map coordinates
    cv::rotate(flip, rotated, cv::ROTATE_90_COUNTERCLOCKWISE);
    cv::imwrite(filePath, rotated);
}

void AstarPlanner::publishPathToRViz(const std::vector<std::pair<double, double>> &world_path)
{
    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker line_strip;
    line_strip.header.frame_id = "/map";
    line_strip.header.stamp = rclcpp::Clock().now();
    line_strip.ns = "astar_path";
    line_strip.id = 0;
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_strip.action = visualization_msgs::msg::Marker::ADD;
    line_strip.scale.x = 0.025; // Thickness
    line_strip.color.a = 1.0;
    line_strip.color.r = 1.0;
    line_strip.color.g = 0.0;
    line_strip.color.b = 0.0;

    for (const auto &coord : world_path)
    {
        geometry_msgs::msg::Point p;
        p.x = coord.first;
        p.y = coord.second;
        p.z = 0.0;
        line_strip.points.push_back(p);
    }

    marker_array.markers.push_back(line_strip);

    path_marker_pub_->publish(marker_array);
    RCLCPP_INFO(this->get_logger(), "Published path as MarkerArray to RViz with %zu points", world_path.size());
}

std::vector<std::pair<double, double>> AstarPlanner::convertGridToWorld(std::vector<AstarPlanner::Point *> gridPath)
{
    std::vector<std::pair<double, double>> worldPath;
    for (const auto &path : gridPath)
    {
        double wx = path->x * resolution_ + origin_x_ + resolution_ / 2.0;
        double wy = path->y * resolution_ + origin_y_ + resolution_ / 2.0;
        worldPath.emplace_back(wx, wy);
        // RCLCPP_INFO(this->get_logger(), "world path x: %.2f, y: %.2f", wx, wy);
    }
    return worldPath;
}

void AstarPlanner::publishPath(std::vector<std::pair<double, double>> worldPath)
{
    nav_msgs::msg::Path rosPath;
    rosPath.header.stamp = this->get_clock()->now();
    rosPath.header.frame_id = "/map"; // Make sure it matches your TF

    for (const auto &[x, y] : worldPath)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = this->get_clock()->now();
        pose.header.frame_id = "/map"; // Consistency with the path header

        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;

        pose.pose.orientation.w = 1.0; // Neutral orientation

        rosPath.poses.push_back(pose);
    }

    // Publish the path
    path_pub_->publish(rosPath);
    RCLCPP_INFO(this->get_logger(), "Published world path with %zu points", rosPath.poses.size());
}

void AstarPlanner::applyObstacleBuffering(double buffer)
{
    int buffer_size = static_cast<int>(buffer / resolution_);
    auto grid = object_grid_;

    RCLCPP_INFO(this->get_logger(), "Buffering");
    RCLCPP_INFO(this->get_logger(), "Buffer size in cells: %d", buffer_size);
    RCLCPP_INFO(this->get_logger(), "Original grid cell (10,10) = %d", original_grid_[10][10]);

    int total_obstacles = 0;
    for (int y = 0; y < grid.size(); ++y)
        for (int x = 0; x < grid[0].size(); ++x)
            if (grid[y][x] == 0)
                total_obstacles++;

    RCLCPP_INFO(this->get_logger(), "Total obstacle cells: %d", total_obstacles);

    for (int y = 0; y < height_; ++y)
    {
        for (int x = 0; x < width_; ++x)
        {
            if (object_grid_[y][x] == 1) // Only expand around original obstacles and new obstacles
            {
                for (int dy = -buffer_size; dy <= buffer_size; ++dy)
                {
                    for (int dx = -buffer_size; dx <= buffer_size; ++dx)
                    {
                        int nx = x + dx;
                        int ny = y + dy;

                        if (nx >= 0 && ny >= 0 && ny < height_ && nx < width_)
                        {
                            grid_[ny][nx] = 1;
                        }
                    }
                }
            }
        }
    }
}
// Impliment A* algorithim. Take an initial start X and Y positon, a goal X and Y Position and a map
// NOTE: x and y positions are relative to the Rviz map, internal conversion are done to change positions to grid locations
std::vector<AstarPlanner::Point *> AstarPlanner::aStarSearch(double startX, double startY, double goalX, double goalY)
{
    std::priority_queue<Point *, std::vector<Point *>, ComparePoint> openList;
    std::unordered_map<int, Point *> visited;

    int gridStartX = (startX - origin_x_) / resolution_;
    int gridStartY = (startY - origin_y_) / resolution_;
    int gridGoalX = (goalX - origin_x_) / resolution_;
    int gridGoalY = (goalY - origin_y_) / resolution_;
    RCLCPP_INFO(this->get_logger(), "start x = %.2f, y = %.2f, grid start x = %d, y = %d ", startX, startY, gridStartX, gridStartY);
    RCLCPP_INFO(this->get_logger(), "goal x = %.2f, y = %.2f, grid goal x = %d, y = %d ", goalX, goalY, gridGoalX, gridGoalY);
    RCLCPP_INFO(this->get_logger(), "Map Resolution = %.2f", resolution_);

    // Check start and goal validity
    if (grid_[gridStartY][gridStartX] != 0 || grid_[gridGoalY][gridGoalX] != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Start or Goal is in an obstacle");
        return {};
    }

    // Create an initial node at the start position with a cost of zero as it hasn't moved yet
    Point *start = new Point(gridStartX, gridStartY, 0, eclidDist(gridStartX, gridStartY, gridGoalX, gridGoalY));
    openList.push(start);

    // Define the movements allowed, I.E. up down left and right and diagonals.
    std::vector<std::pair<int, int>> directions = {
        {0, 1},   // up
        {1, 0},   // right
        {0, -1},  // down
        {-1, 0},  // left
        {1, 1},   // top-right
        {1, -1},  // bottom-right
        {-1, -1}, // bottom-left
        {-1, 1}   // top-left
    };

    // Start the A* loop, this will continue to run while there is nodes in the priority queue, as the initial node was pushed back above
    while (!openList.empty())
    {
        Point *current = openList.top();
        openList.pop();

        // Skip if already visited
        int currentKey = current->y * width_ + current->x;
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
                // RCLCPP_INFO(this->get_logger(), "New Node");
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
            if (nx >= 0 && ny >= 0 && ny < static_cast<int>(height_) && nx < static_cast<int>(width_) && grid_[ny][nx] == 0)
            {
                if (dx != 0 && dy != 0)
                {
                    if (grid_[current->y][current->x + dx] != 0 || grid_[current->y + dy][current->x] != 0)
                    {
                        continue; // Don't allow diagonal if either adjacent cardinal cell is an obstacle
                    }
                }
                int neighborKey = ny * width_ + nx;
                if (!visited.count(neighborKey))
                {
                    // cost to move, regualr left right up and down cost 1, diagonals as they are a further away cost 1.41
                    float moveCost = (dx == 0 || dy == 0) ? 1.0f : std::sqrt(2.0f);
                    float newCost = current->cost + moveCost;
                    Point *neighbor = new Point(nx, ny, newCost, eclidDist(nx, ny, gridGoalX, gridGoalY), current);
                    // RCLCPP_ERROR(this->get_logger(), "Bad Node");
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
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AstarPlanner>());
    rclcpp::shutdown();
    return 0;
}
