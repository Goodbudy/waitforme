#include "detection.h"
#include <cmath>
#include <utility>
#include <tf2/utils.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <std_msgs/msg/color_rgba.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>



ObjDetect::ObjDetect() : Node("detection_node"), firstCent(true), ct_(0)
{
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&ObjDetect::scanCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&ObjDetect::odomCallback, this, std::placeholders::_1));
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/visualization_marker", 10);
}

void ObjDetect::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    auto segments = countSegments(scan);
}

void ObjDetect::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
    currentOdom = *odom;
}

std::vector<std::vector<geometry_msgs::msg::Point>> ObjDetect::countSegments(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    geometry_msgs::msg::Point current, previous;
    std::vector<std::vector<geometry_msgs::msg::Point>> segmentVector;
    std::vector<geometry_msgs::msg::Point> currentSegment;
    bool segStarted = false;

    for (size_t i = 1; i < scan->ranges.size(); i++)
    {
        if (std::isinf(scan->ranges.at(i - 1)) || std::isnan(scan->ranges.at(i - 1)))
            continue;

        while (i < scan->ranges.size() && (!std::isnan(scan->ranges.at(i))) && (scan->ranges.at(i) < scan->range_max))
        {
            float previousAngle = scan->angle_min + scan->angle_increment * (i - 1);
            previous.x = scan->ranges.at(i - 1) * cos(previousAngle);
            previous.y = scan->ranges.at(i - 1) * sin(previousAngle);

            float currentAngle = scan->angle_min + scan->angle_increment * i;
            current.x = scan->ranges.at(i) * cos(currentAngle);
            current.y = scan->ranges.at(i) * sin(currentAngle);

            float dist = hypot((current.x - previous.x), (current.y - previous.y));

            if (dist < 0.075)
            {
                if (!segStarted)
                {
                    segStarted = true;
                    currentSegment.clear();
                }
                currentSegment.push_back(localToGlobal(currentOdom, current));
            }
            else
            {
                /*
                if (segStarted && !currentSegment.empty())
                {
                    segmentVector.push_back(currentSegment);
                    if (!isThisAWall(currentSegment) && !isThisACorner(currentSegment))
                        detectCylinder(currentSegment);
                    else if (!isThisAWall(currentSegment) && isThis90(currentSegment))
                        detectSquare(currentSegment);
                    
                    segStarted = false;
                }
                */
                break;
            }
            i++;
        }
        /*
        if (segStarted && !currentSegment.empty())
        {
            segmentVector.push_back(currentSegment);
            rclcpp::sleep_for(std::chrono::milliseconds(10));
            if (!isThisAWall(currentSegment) && !isThisACorner(currentSegment))
                detectCylinder(currentSegment);
            else if (!isThisAWall(currentSegment) && isThis90(currentSegment))
                detectSquare(currentSegment);
            segStarted = false;
        }
        */
        if (segStarted && !currentSegment.empty())
        {
            segmentVector.push_back(currentSegment);
            rclcpp::sleep_for(std::chrono::milliseconds(10));
            if (!isThisAWall(currentSegment)) {
                if (isThis90(currentSegment)) {
                    detectSquare(currentSegment);
                } else if (!isThisACorner(currentSegment)) {
                    detectCylinder(currentSegment);
                }
            }
            segStarted = false;
        }

    }

    return segmentVector;
}

/*
void ObjDetect::detectCylinder(const std::vector<geometry_msgs::msg::Point> &segment)
{
    if (segment.size() >= 6)
    {
        const auto &p1 = segment.front();
        const auto &p2 = segment.at(segment.size() / 2);
        const auto &p3 = segment.back();

        double a = hypot(p2.x - p1.x, p2.y - p1.y);
        double b = hypot(p3.x - p2.x, p3.y - p2.y);
        double c = hypot(p3.x - p1.x, p3.y - p1.y);

        double cosTheta = (a * a + b * b - c * c) / (2 * a * b);
        double theta = acos(cosTheta);
        double R = c / (2 * fabs(sin(theta / 2)));

        double targetRadius = 0.15;
        double tolerance_ = 0.01;

        if (fabs(R - targetRadius) < tolerance_)
        {
            geometry_msgs::msg::Point centre = findCentre(p1, p3, targetRadius);
            if (!checkExisting(centre) || firstCent)
            {
                firstCent = false;
                centres.push_back(centre);
                RCLCPP_INFO(this->get_logger(), "Circle with radius ~%.2fm detected. Center: x = %.2f, y = %.2f, z = %.2f", targetRadius, centre.x, centre.y, centre.z);
                marker_pub_->publish(produceMarkerCylinder(centre, visualization_msgs::msg::Marker::CYLINDER, "black"));
            }
        }
    }
}
*/
void ObjDetect::detectCylinder(const std::vector<geometry_msgs::msg::Point> &segment)
{
    if (segment.size() >= 6 && !isThisACorner(segment))
    {
        const auto &p1 = segment.front();
        const auto &p2 = segment.at(segment.size() / 2);
        const auto &p3 = segment.back();

        double a = hypot(p2.x - p1.x, p2.y - p1.y);
        double b = hypot(p3.x - p2.x, p3.y - p2.y);
        double c = hypot(p3.x - p1.x, p3.y - p1.y);

        double cosTheta = (a * a + b * b - c * c) / (2 * a * b);
        double theta = acos(cosTheta);
        double R = c / (2 * fabs(sin(theta / 2)));

        double targetRadius = 0.15;
        double tolerance_ = 0.01;

        if (fabs(R - targetRadius) < tolerance_)
        {
            geometry_msgs::msg::Point centre = findCentre(p1, p3, targetRadius);
            
            if (std::isnan(centre.x) || std::isnan(centre.y) || std::isnan(centre.z)) {
                RCLCPP_WARN(this->get_logger(), "Skipping marker: detected NaN in center coordinates.");
                return;
            }

            // New: verify all points lie close to this radius
            bool allCloseToRadius = true;
            for (const auto &pt : segment)
            {
                double dist = hypot(pt.x - centre.x, pt.y - centre.y);
                if (fabs(dist - targetRadius) > tolerance_)
                {
                    allCloseToRadius = false;
                    break;
                }
            }

            if (allCloseToRadius && (!checkExisting(centre) || firstCent))
            {
                firstCent = false;
                centres.push_back(centre);
                RCLCPP_INFO(this->get_logger(), "Circle with radius ~%.2fm detected. Center: x = %.2f, y = %.2f, z = %.2f", targetRadius, centre.x, centre.y, centre.z);
                marker_pub_->publish(produceMarkerCylinder(centre, visualization_msgs::msg::Marker::CYLINDER, "black"));
            }
        }
    }
}
/*
void ObjDetect::detectSquare(const std::vector<geometry_msgs::msg::Point> &segment)
{
    if (segment.size() < 6)
        return;

    // Get points from the segment
    const auto &p1 = segment.front();
    const auto &p2 = segment.at(segment.size() / 2);
    const auto &p3 = segment.back();

    // Compute side lengths
    double sideA = hypot(p2.x - p1.x, p2.y - p1.y);
    double sideB = hypot(p3.x - p2.x, p3.y - p2.y);

    double targetLength = 0.3;
    double tolerance = 0.05;

    if (isThis90(segment) &&
        fabs(sideA - targetLength) < tolerance &&
        fabs(sideB - targetLength) < tolerance)
    {
        geometry_msgs::msg::Point corner = p2;
        if (std::isnan(corner.x) || std::isnan(corner.y) || std::isnan(corner.z)) {
            RCLCPP_WARN(this->get_logger(), "Skipping marker: detected NaN in square corner coordinates.");
            return;
        }
        
        // Optional: check if corner is already known to avoid duplicates
        if (!checkExisting(corner) || firstCent)
        {
            firstCent = false;
            centres.push_back(corner);

            RCLCPP_INFO(this->get_logger(), "Square corner detected at x = %.2f, y = %.2f", corner.x, corner.y);

            // Reuse the cylinder marker for now with a different color or style
            auto marker = produceMarkerCylinder(corner, visualization_msgs::msg::Marker::CUBE, "red");
            marker_pub_->publish(marker);
        }
    }
}
*/
//Line fitting method for detecting squares; includes fit line and anglebetweensegments
void ObjDetect::detectSquare(const std::vector<geometry_msgs::msg::Point> &segment)
{
    if (segment.size() < 6)
        return;

    size_t half = segment.size() / 2;
    std::vector<geometry_msgs::msg::Point> seg1(segment.begin(), segment.begin() + half);
    std::vector<geometry_msgs::msg::Point> seg2(segment.begin() + half, segment.end());

    float length1 = hypot(seg1.front().x - seg1.back().x, seg1.front().y - seg1.back().y);
    float length2 = hypot(seg2.front().x - seg2.back().x, seg2.front().y - seg2.back().y);

    float targetLength = 0.3;
    float tolerance = 0.05;

    if (fabs(length1 - targetLength) < tolerance && fabs(length2 - targetLength) < tolerance)
    {
        float angle = angleBetweenSegments(seg1, seg2);
        if (angle > 85 && angle < 95)
        {
            geometry_msgs::msg::Point corner = segment[half]; // approx corner
            if (std::isnan(corner.x) || std::isnan(corner.y))
                return;

            if (!checkExisting(corner) || firstCent)
            {
                firstCent = false;
                centres.push_back(corner);
                RCLCPP_INFO(this->get_logger(), "Fitted square corner at x=%.2f, y=%.2f", corner.x, corner.y);
                auto marker = produceMarkerCylinder(corner, visualization_msgs::msg::Marker::CUBE, "red");
                marker_pub_->publish(marker);
            }
        }
    }
}

std::pair<float, float> ObjDetect::fitLine(const std::vector<geometry_msgs::msg::Point>& segment)
{
    float sumX = 0.0, sumY = 0.0, sumXY = 0.0, sumX2 = 0.0;
    size_t n = segment.size();

    for (const auto& pt : segment)
    {
        sumX += pt.x;
        sumY += pt.y;
        sumXY += pt.x * pt.y;
        sumX2 += pt.x * pt.x;
    }

    float denom = (n * sumX2 - sumX * sumX);
    if (denom == 0.0) return {0.0f, 0.0f}; // fallback

    float slope = (n * sumXY - sumX * sumY) / denom;
    float intercept = (sumY - slope * sumX) / n;

    return {slope, intercept};
}

float ObjDetect::angleBetweenSegments(const std::vector<geometry_msgs::msg::Point>& seg1, const std::vector<geometry_msgs::msg::Point>& seg2)
{
    if (seg1.size() < 2 || seg2.size() < 2) return 0.0;

    auto p1 = seg1.front();
    auto p2 = seg1.back();
    auto p3 = seg2.front();
    auto p4 = seg2.back();

    float v1x = p2.x - p1.x;
    float v1y = p2.y - p1.y;
    float v2x = p4.x - p3.x;
    float v2y = p4.y - p3.y;

    float dot = v1x * v2x + v1y * v2y;
    float mag1 = hypot(v1x, v1y);
    float mag2 = hypot(v2x, v2y);

    float cosTheta = dot / (mag1 * mag2);
    cosTheta = std::clamp(cosTheta, -1.0f, 1.0f); // prevent NaN
    float theta = acos(cosTheta) * 180 / M_PI;
    return theta;
}


geometry_msgs::msg::Point ObjDetect::findCentre(geometry_msgs::msg::Point P1, geometry_msgs::msg::Point P2, double r)
{
    geometry_msgs::msg::Point centre;
    double mx = (P1.x + P2.x) / 2.0;
    double my = (P1.y + P2.y) / 2.0;
    double d = hypot(P2.x - P1.x, P2.y - P1.y);
    double h = sqrt(r * r - (d / 2.0) * (d / 2.0));
    double dx = -(P2.y - P1.y) / d;
    double dy = (P2.x - P1.x) / d;

    double c1x = mx + h * dx;
    double c1y = my + h * dy;
    double c2x = mx - h * dx;
    double c2y = my - h * dy;

    double d2 = hypot(currentOdom.pose.pose.position.x - c1x, currentOdom.pose.pose.position.y - c1y);
    double d3 = hypot(currentOdom.pose.pose.position.x - c2x, currentOdom.pose.pose.position.y - c2y);

    if (d2 > d3)
        {
            centre.x = c1x;
            centre.y = c1y;
            centre.z = currentOdom.pose.pose.position.z;
        }
    else
        {
            centre.x = c2x;
            centre.y = c2y;
            centre.z = currentOdom.pose.pose.position.z;
        }
    return centre;
}

bool ObjDetect::checkExisting(geometry_msgs::msg::Point centre)
{
    duplicate_threshold_ = 0.5;
    for (const auto &existing_centre : centres)
    {
        if (hypot(centre.x - existing_centre.x, centre.y - existing_centre.y) < duplicate_threshold_)
            return true;
    }
    return false;
}

void ObjDetect::visualizeSegment(const std::vector<geometry_msgs::msg::Point> &segment)
{
    visualization_msgs::msg::Marker line_marker;
    line_marker.header.frame_id = "map";
    line_marker.header.stamp = this->get_clock()->now();
    line_marker.ns = "cylinder_markers";
    line_marker.id = ct_++;
    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::msg::Marker::ADD;
    line_marker.scale.x = 0.05;
    line_marker.color.a = 0.8;
    line_marker.color.g = 1.0;

    for (const auto &point : segment)
        line_marker.points.push_back(point);

    marker_pub_->publish(line_marker);
}

visualization_msgs::msg::Marker ObjDetect::produceMarkerCylinder(geometry_msgs::msg::Point pt, int type, const std::string& colour)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "cylinder_markers";
    marker.id = ct_++;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = pt;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    if (colour == "black") {
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
    } else if (colour == "red") {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
    } else {
        // Default to black if unrecognized
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
    }
    return marker;
}

geometry_msgs::msg::Point ObjDetect::localToGlobal(const nav_msgs::msg::Odometry &global, const geometry_msgs::msg::Point &local)
{
    geometry_msgs::msg::Point pt;
    double yaw = tf2::getYaw(global.pose.pose.orientation);
    pt.x = global.pose.pose.position.x + (local.x * cos(yaw) - local.y * sin(yaw));
    pt.y = global.pose.pose.position.y + (local.x * sin(yaw) + local.y * cos(yaw));
    pt.z = 0;
    return pt;
}

bool ObjDetect::isThisAWall(const std::vector<geometry_msgs::msg::Point> &segment)
{
    return hypot(segment.front().x - segment.back().x, segment.front().y - segment.back().y) > 0.6;
}

bool ObjDetect::isThisACorner(const std::vector<geometry_msgs::msg::Point> &segment)
{
    float a = hypot(segment.back().x - segment[segment.size() / 2].x, segment.back().y - segment[segment.size() / 2].y);
    float b = hypot(segment.front().x - segment[segment.size() / 2].x, segment.front().y - segment[segment.size() / 2].y);
    float c = hypot(segment.back().x - segment.front().x, segment.back().y - segment.front().y);
    float cosTheta = (a * a + b * b - c * c) / (2 * a * b);
    float theta = acos(cosTheta) * 180 / M_PI;

    return theta < 120;
}
/*
bool ObjDetect::isThis90(const std::vector<geometry_msgs::msg::Point> &segment)
{
    float a = hypot(segment.back().x - segment[segment.size() / 2].x, segment.back().y - segment[segment.size() / 2].y);
    float b = hypot(segment.front().x - segment[segment.size() / 2].x, segment.front().y - segment[segment.size() / 2].y);
    float c = hypot(segment.back().x - segment.front().x, segment.back().y - segment.front().y);
    float cosTheta = (a * a + b * b - c * c) / (2 * a * b);
    float theta = acos(cosTheta) * 180 / M_PI;

    return 85 < theta < 95;
}
*/
/* 
//V2 fixing errors
bool ObjDetect::isThis90(const std::vector<geometry_msgs::msg::Point> &segment)
{
    float a = hypot(segment.back().x - segment[segment.size() / 2].x, segment.back().y - segment[segment.size() / 2].y);
    float b = hypot(segment.front().x - segment[segment.size() / 2].x, segment.front().y - segment[segment.size() / 2].y);
    float c = hypot(segment.back().x - segment.front().x, segment.back().y - segment.front().y);
    float cosTheta = (a * a + b * b - c * c) / (2 * a * b);
    float theta = acos(cosTheta) * 180 / M_PI;

    return (theta > 85.0 && theta < 95.0);
}
*/
/*
// using fitline and anglebetweenline to accurately test?
bool ObjDetect::isThis90(const std::vector<geometry_msgs::msg::Point>& segment)
{
    if (segment.size() < 6) return false; // Need enough points for fitting

    size_t mid = segment.size() / 2;
    std::vector<geometry_msgs::msg::Point> firstHalf(segment.begin(), segment.begin() + mid);
    std::vector<geometry_msgs::msg::Point> secondHalf(segment.begin() + mid, segment.end());

    cv::Vec4f line1 = fitLineToPoints(firstHalf);
    cv::Vec4f line2 = fitLineToPoints(secondHalf);

    float angle = angleBetweenLines(line1, line2);
    return (angle > 80.0 && angle < 100.0);
}
*/
/*
//using segment line fitting going through all points
// Function to check if a segment contains a 90-degree angle
bool ObjDetect::isThis90(const std::vector<geometry_msgs::msg::Point>& segment)
{
    if (segment.size() < 3)
    {
        return false; // Not enough points to check for 90-degree angle
    }

    // Function to calculate the angle between two lines using their slopes
    auto calculateAngleBetweenLines = [](const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2, const geometry_msgs::msg::Point& p3) -> float 
    {
        float dx1 = p2.x - p1.x;
        float dy1 = p2.y - p1.y;
        float dx2 = p3.x - p2.x;
        float dy2 = p3.y - p2.y;
        
        float dot = dx1 * dx2 + dy1 * dy2;
        float mag1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
        float mag2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
        
        float angle = std::acos(dot / (mag1 * mag2)) * 180.0 / M_PI;
        return angle;
    };

    // Check for 90-degree angle between consecutive line segments
    for (size_t i = 1; i < segment.size() - 1; ++i)
    {
        float angle = calculateAngleBetweenLines(segment[i-1], segment[i], segment[i+1]);

        if (fabs(angle - 90.0f) <= 10.0f) // Check if the angle is close to 90 degrees (within a tolerance)
        {
            std::cout << "90-degree corner detected at index " << i << " (angle = " << angle << "°)" << std::endl;
            return true; // 90-degree corner detected
        }
    }
    return false; // No 90-degree angle detected
}
*/
bool ObjDetect::isThis90(const std::vector<geometry_msgs::msg::Point>& segment)
{
    const size_t windowSize = 10;
    const float angleToleranceDeg = 10.0;

    if (segment.size() < 2 * windowSize)
        return false; // Not enough points to form two lines

    // Helper lambda to fit a line direction vector from a range of points
    auto fitLine = [](const std::vector<geometry_msgs::msg::Point>& points, size_t start, size_t count) {
        float sum_x = 0, sum_y = 0;
        for (size_t i = start; i < start + count; ++i) {
            sum_x += points[i].x;
            sum_y += points[i].y;
        }
        float mean_x = sum_x / count;
        float mean_y = sum_y / count;

        float num = 0, den = 0;
        for (size_t i = start; i < start + count; ++i) {
            num += (points[i].x - mean_x) * (points[i].y - mean_y);
            den += (points[i].x - mean_x) * (points[i].x - mean_x);
        }

        float slope = den != 0 ? num / den : 0.0;  // Avoid division by zero
        float dx = 1.0;
        float dy = slope;
        float mag = std::sqrt(dx * dx + dy * dy);
        return std::make_pair(dx / mag, dy / mag); // return unit vector
    };

    // Helper lambda to compute angle between two direction vectors
    auto angleBetween = [](std::pair<float, float> v1, std::pair<float, float> v2) {
        float dot = v1.first * v2.first + v1.second * v2.second;
        dot = std::clamp(dot, -1.0f, 1.0f); // Clamp for numerical safety
        return std::acos(dot) * 180.0 / M_PI;
    };

    // Slide a window through the segment and compare line directions
    for (size_t i = 0; i <= segment.size() - 2 * windowSize; ++i) {
        auto dir1 = fitLine(segment, i, windowSize);
        auto dir2 = fitLine(segment, i + windowSize, windowSize);
        float angle = angleBetween(dir1, dir2);

        if (std::fabs(angle - 90.0f) <= angleToleranceDeg) {
            std::cout << "90-degree corner detected at index " << i + windowSize
                      << " (angle = " << angle << "°)" << std::endl;
            return true;
        }
    }

    return false;
}



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjDetect>());
    rclcpp::shutdown();
    return 0;
}
