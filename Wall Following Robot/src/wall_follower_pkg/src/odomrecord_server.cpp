#include "custom_interfaces/action/odom_record.hpp"
#include "geometry_msgs/msg/point32.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <thread>

class OdomRecordActionServer : public rclcpp::Node {
public:
  // aliases
  using OdomRecord = custom_interfaces::action::OdomRecord;
  using GoalHandle_Odom = rclcpp_action::ServerGoalHandle<OdomRecord>;

  explicit OdomRecordActionServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("odom_record_action_server", options) {
    using namespace std::placeholders;

    // creating action server
    this->odomrec_actionserver_ = rclcpp_action::create_server<OdomRecord>(
        this, "record_odom",
        std::bind(&OdomRecordActionServer::handle_goal, this, _1, _2),
        std::bind(&OdomRecordActionServer::handle_cancel, this, _1),
        std::bind(&OdomRecordActionServer::handle_accepted, this, _1));

    // odom subscriber
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10,
        std::bind(&OdomRecordActionServer::odom_callback, this, _1));

    // class variables
    this->current_distance = 0.0;
    this->current_pos.x = 0.0;
    this->current_pos.y = 0.0;
    this->current_pos.z = 0.0;
    this->start_dist_error = true;
    this->lap = false;
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // callback to get odom data (for initial and current pos.
    this->current_pos.x = msg->pose.pose.position.x;
    this->current_pos.y = msg->pose.pose.position.y;
    this->current_pos.z = msg->pose.pose.position.z;
  }

  float calc_distance(const std::vector<geometry_msgs::msg::Point32> pos,
                      const int mode) {
    float dist = 0.0;
    if (mode == 1) {
      dist = sqrt(pow(pos.end()[-1].x - pos.end()[-2].x, 2) +
                  pow(pos.end()[-1].y - pos.end()[-2].y, 2));
    } else {
      dist = sqrt(pow(pos.end()[-1].x - pos.front().x, 2) +
                  pow(pos.end()[-1].y - pos.front().y, 2));
    }
    return dist;
  }

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const OdomRecord::Goal> goal) {
    // response to receiving goal
    RCLCPP_INFO(this->get_logger(), "Received goal to Record Odometry");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandle_Odom> goal_handle) {
    // what happens when goal is cancelled
    RCLCPP_INFO(this->get_logger(), "Cancelling goal.");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle_Odom> goal_handle) {
    using namespace std::placeholders;

    // returning quickly to prevent blocking executor, spinning new thread
    std::thread{std::bind(&OdomRecordActionServer::execute, this, _1),
                goal_handle}
        .detach();
  }

  void execute(const std::shared_ptr<GoalHandle_Odom> goal_handle) {
    // main execute method to do action every second
    RCLCPP_INFO(this->get_logger(), "Executing goal: Recording Odometry Data");
    rclcpp::Rate loop_rate(1);

    // initializations
    const auto goal = goal_handle->get_goal();                // goal variable
    auto feedback = std::make_shared<OdomRecord::Feedback>(); // feedback
    auto &message = feedback->current_total; // send to feedback name
    message = 0.0;                           // float 32 initial message
    auto result = std::make_shared<OdomRecord::Result>();

    // keep sending feedback until robot has done a full lap
    while (!this->lap && rclcpp::ok()) {
      // check if there is any cancellation request
      if (goal_handle->is_canceling()) {
        result->list_of_odoms = this->list_of_pos;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      // every sec, append new current pos
      list_of_pos.push_back(this->current_pos);

      // calc distance travelled every sec after 1st 2 positions
      if (list_of_pos.size() > 0) {
        // start at 0 distance travelled
        float dist;
        if (this->start_dist_error) {
          dist = calc_distance(list_of_pos, 1) - calc_distance(list_of_pos, 1);
          this->start_dist_error = false;
        } else {
          dist = calc_distance(list_of_pos, 1);
        }

        // update total current distance and set as message
        this->current_distance = this->current_distance + dist;
        message = this->current_distance;
      }

      // publishing feedback message
      // RCLCPP_INFO(this->get_logger(), "Sending accumulated distance.");
      goal_handle->publish_feedback(feedback);

      // check if robot has done a lap, stop service if so
      if (this->current_distance > 4.5) {
        // only start calculating after a certain distance
        float lap_dist = calc_distance(list_of_pos, 0);
        if (lap_dist <= 0.35) {
          this->lap = true;
        }
      }

      loop_rate.sleep(); // rate
    }

    // check if goal is done, end execute, stop robot
    if (rclcpp::ok()) {
      result->list_of_odoms = this->list_of_pos;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Odom Record Succeeded.");
    }
  }

  // Initializations
  rclcpp_action::Server<OdomRecord>::SharedPtr odomrec_actionserver_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  std::vector<geometry_msgs::msg::Point32> list_of_pos;
  geometry_msgs::msg::Point32 current_pos;
  float current_distance;
  bool start_dist_error;
  bool lap;
}; // Odom Record Server Class

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto OdomRecord_ActionServer = std::make_shared<OdomRecordActionServer>();

  // Running executor
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(OdomRecord_ActionServer);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}