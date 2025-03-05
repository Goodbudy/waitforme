#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/string.hpp"

class GoalsService : public rclcpp::Node {
public:
    GoalsService() : Node("goals_service") {
        service_ = this->create_service<std_srvs::srv::Trigger>("goals",
            std::bind(&GoalsService::handleRequest, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;

    void handleRequest(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        response->success = true;
        response->message = "Goals service is active.";
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalsService>());
    rclcpp::shutdown();
    return 0;
}
