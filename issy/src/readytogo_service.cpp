#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

class ReadyToGoService : public rclcpp::Node {
public:
    ReadyToGoService() : Node("readytogo_service") {
        service_ = this->create_service<std_srvs::srv::Trigger>("readytogo",
            std::bind(&ReadyToGoService::handleRequest, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;

    void handleRequest(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        response->success = true;
        response->message = "Ready to go!";
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReadyToGoService>());
    rclcpp::shutdown();
    return 0;
}
