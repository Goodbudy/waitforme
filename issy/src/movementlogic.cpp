#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "issy/srv/add_goal.hpp"
#include "issy/srv/execute_goals.hpp"
#include <queue>
#include <vector>
#include <cmath>

enum class State {
  HOMING,
  IDLE,
  EXEC_GOAL_PATH,
  WAITING_AT_GOAL,
  RETURNING_HOME_PATH
};

static double normalizeAngle(double ang) {
  while (ang > M_PI)  ang -= 2.0*M_PI;
  while (ang < -M_PI) ang += 2.0*M_PI;
  return ang;
}

class MovementLogic : public rclcpp::Node
{
public:
  MovementLogic()
  : Node("movement_logic"),
    x_home_(3.5),
    y_home_(3.2),
    tolerance_(0.2),
    state_(State::HOMING),
    current_target_idx_(0),
    current_x_(0.0),
    current_y_(0.0),
    current_yaw_(0.0)
  {
    // make sure planner sees our initial home goal
    auto goal_qos = rclcpp::QoS(1).transient_local();
    goal_pub_    = create_publisher<geometry_msgs::msg::PoseStamped>("astar_goal", goal_qos);
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "planned_path", 10,
      std::bind(&MovementLogic::pathCallback, this, std::placeholders::_1)
    );

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 20,
      std::bind(&MovementLogic::odomCallback, this, std::placeholders::_1)
    );

    add_goal_srv_ = create_service<issy::srv::AddGoal>(
      "add_goal",
      std::bind(&MovementLogic::handleAddGoal, this,
                std::placeholders::_1, std::placeholders::_2)
    );

    exec_goals_srv_ = create_service<issy::srv::ExecuteGoals>(
      "execute_goals",
      std::bind(&MovementLogic::handleExecuteGoals, this,
                std::placeholders::_1, std::placeholders::_2)
    );

    follow_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&MovementLogic::followPath, this)
    );

    RCLCPP_INFO(get_logger(),
                "Startup: homing to (%.2f, %.2f)", x_home_, y_home_);
    publishGoal(x_home_, y_home_);
  }

private:
  // ROS interfaces
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr       cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr         path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr     odom_sub_;
  rclcpp::Service<issy::srv::AddGoal>::SharedPtr               add_goal_srv_;
  rclcpp::Service<issy::srv::ExecuteGoals>::SharedPtr          exec_goals_srv_;
  rclcpp::TimerBase::SharedPtr                                 follow_timer_;
  rclcpp::TimerBase::SharedPtr                                 wait_timer_;

  // State
  double x_home_, y_home_, tolerance_;
  State state_;
  std::queue<std::pair<double,double>> goal_queue_;
  std::vector<geometry_msgs::msg::PoseStamped> path_;
  size_t current_target_idx_;
  double current_x_, current_y_, current_yaw_;

  // Control gains & limits
  const double kp_lin_      = 0.5;
  const double kp_ang_      = 1.0;
  const double ang_tol_     = 0.1;   // rad before forward motion
  const double max_lin_vel_ = 0.2;
  const double max_ang_vel_ = 1.0;

  void publishGoal(double x, double y) {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp    = now();
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.orientation.w = 1.0;
    goal_pub_->publish(goal);
    RCLCPP_INFO(get_logger(),
                "[%d] Published goal (%.2f, %.2f)",
                static_cast<int>(state_), x, y);
  }

  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    if (msg->poses.empty()) {
      RCLCPP_WARN(get_logger(), "Received empty path");
      return;
    }
    path_ = msg->poses;
    current_target_idx_ = 0;
    RCLCPP_INFO(get_logger(),
                "Received path with %zu points", path_.size());
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    // extract yaw
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;
    double siny = 2.0*(qw*qz + qx*qy);
    double cosy = 1.0 - 2.0*(qy*qy + qz*qz);
    current_yaw_ = std::atan2(siny, cosy);
  }

  void followPath() {
    geometry_msgs::msg::Twist cmd{};
    // no path or done => stop
    if (path_.empty() || current_target_idx_ >= path_.size()) {
      cmd_vel_pub_->publish(cmd);
      return;
    }

    auto &t = path_[current_target_idx_].pose;
    double dx   = t.position.x - current_x_;
    double dy   = t.position.y - current_y_;
    double dist = std::hypot(dx, dy);

    // reached waypoint?
    if (dist < tolerance_) {
      current_target_idx_++;
      if (current_target_idx_ >= path_.size()) {
        RCLCPP_INFO(get_logger(),
                    "Completed path at state %d",
                    static_cast<int>(state_));
        cmd_vel_pub_->publish(cmd);
        onPathComplete();
      }
      return;
    }

    // steer
    double desired_yaw = std::atan2(dy, dx);
    double yaw_err     = normalizeAngle(desired_yaw - current_yaw_);
    double lin_vel     = 0.0;
    double ang_vel     = kp_ang_ * yaw_err;

    if (std::fabs(yaw_err) < ang_tol_) {
      lin_vel = kp_lin_ * dist;
    }

    lin_vel = std::clamp(lin_vel, -max_lin_vel_, max_lin_vel_);
    ang_vel = std::clamp(ang_vel, -max_ang_vel_, max_ang_vel_);

    cmd.linear.x  = lin_vel;
    cmd.angular.z = ang_vel;
    cmd_vel_pub_->publish(cmd);
  }

  void onPathComplete() {
    switch (state_) {
      case State::HOMING:
        state_ = State::IDLE;
        break;

      case State::EXEC_GOAL_PATH:
        state_ = State::WAITING_AT_GOAL;
        RCLCPP_INFO(get_logger(), "Waiting 5s at goal...");
        // one-shot 5s timer
        wait_timer_ = create_wall_timer(
          std::chrono::seconds(5),
          [this]() {
            wait_timer_->cancel();          // cancel after first fire
            RCLCPP_INFO(get_logger(), "5s elapsed, returning home");
            state_ = State::RETURNING_HOME_PATH;
            publishGoal(x_home_, y_home_);
          }
        );
        break;

      case State::RETURNING_HOME_PATH:
        state_ = State::IDLE;              // back at home, wait for exec_goals
        RCLCPP_INFO(get_logger(), "Home reached, idle");
        break;

      default:
        break;
    }
  }

  void handleAddGoal(
    const std::shared_ptr<issy::srv::AddGoal::Request> req,
    std::shared_ptr<issy::srv::AddGoal::Response>    res)
  {
    goal_queue_.emplace(req->x, req->y);
    res->success = true;
    res->message = "Goal queued";
    RCLCPP_INFO(get_logger(),
                "Added goal (%.2f, %.2f), queue size=%zu",
                req->x, req->y, goal_queue_.size());
  }

  void handleExecuteGoals(
    const std::shared_ptr<issy::srv::ExecuteGoals::Request>,
    std::shared_ptr<issy::srv::ExecuteGoals::Response> res)
  {
    if (state_ == State::IDLE && !goal_queue_.empty()) {
      auto [x,y] = goal_queue_.front();
      goal_queue_.pop();
      state_ = State::EXEC_GOAL_PATH;
      publishGoal(x, y);
      res->success = true;
      res->message = "Executing next goal";
      RCLCPP_INFO(get_logger(),
                  "Executing goal (%.2f, %.2f)", x, y);
    } else {
      res->success = false;
      res->message = "Cannot execute now";
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MovementLogic>());
  rclcpp::shutdown();
  return 0;
}