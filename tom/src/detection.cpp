#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <chrono>
#include <thread>  
/*
class Detection : public rclcpp::NODE {
public:
Detection() : Node("detection") {
    publisher_ = this->create_publisher<geometry_msgs::msg::TOBENAMED>("/TOBEFOUND", 10); //to publish objects and names.
    subscribe_ = this->create_subscriber< //subscribe to lidar
}

private:
//where i'm gonna write all this code.
/*
Function to detect shapes
have list of known shapes
    square = artwork
    circle = table
    other
        if square & moving = turtlebot
        if other = person?

when detected pass onto function which publishes position, and object.
Mark on rviz what object is with identifier. eg square = artwork
*/


//}


int main(int argc, char const *argv[])
{
    /* code */
    return 0;
}
