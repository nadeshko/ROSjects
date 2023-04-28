#include "custom_interfaces/action/odom_record.hpp"
#include "custom_interfaces/srv/find_wall.hpp"
#include "geometry_msgs/msg/point32.h"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <chrono>
#include <cstdio>
#include <functional>
#include <future>
#include <memory>

using namespace std::chrono_literals;

class WallFollower : public rclcpp ::Node {
  // wall follower construct
public:
  WallFollower() : Node("wall_follower") {
    using namespace std::placeholders;

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&WallFollower::laser_callback, this, _1));
    timer_ = this->create_wall_timer(
        500ms, std::bind(&WallFollower::timer_callback, this));
    // similar to self from python
    this->laser_front = 0.0;
    this->laser_right = 0.0;
  }

private:
  // functions in class
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr laser) {
    this->laser_right = laser->ranges[180];
    this->laser_front = laser->ranges[360];
  }

  void timer_callback() {
    if (laser_front > 0.5) {
      if (laser_right > 0.26) {
        twist_msg.linear.x = 0.07;
        twist_msg.angular.z = -0.1;
        RCLCPP_INFO(this->get_logger(), "Adjusting Right");
      } else if (laser_right < 0.21) {
        twist_msg.linear.x = 0.07;
        twist_msg.angular.z = 0.1;
        RCLCPP_INFO(this->get_logger(), "Adjusting Left");
      } else {
        twist_msg.linear.x = 0.07;
        twist_msg.angular.z = 0;
        RCLCPP_INFO(this->get_logger(), "Straight");
      }
    } else if (laser_front < 0.5) {
      if (laser_right < 0.2) {
        twist_msg.linear.x = 0.0;
        twist_msg.angular.z = 0.2;
        RCLCPP_INFO(this->get_logger(), "Just Turning Left");
      } else {
        twist_msg.linear.x = 0.07;
        twist_msg.angular.z = 0.4;
        RCLCPP_INFO(this->get_logger(), "Turning Left Quickly");
      }
    }
    publisher_->publish(this->twist_msg); // publish velocity command
  }

  // initializations for class
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  geometry_msgs::msg::Twist twist_msg;
  float laser_front;
  float laser_left;
  float laser_right;
};

class OdomRecordActionClient : public rclcpp::Node {
public:
  // aliases
  using OdomRecord = custom_interfaces::action::OdomRecord;
  using GoalHandle_Odom = rclcpp_action::ClientGoalHandle<OdomRecord>;

  explicit OdomRecordActionClient(
      const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
      : Node("odom_record_action_client", node_options) {
    // action client
    this->odomrec_actionclient_ = rclcpp_action::create_client<OdomRecord>(
        this->get_node_base_interface(), this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(), "record_odom");

    // timer
    this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&OdomRecordActionClient::send_goal, this));
  }

  void send_goal() {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->odomrec_actionclient_) {
      // check if action server is running
      RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }

    if (!this->odomrec_actionclient_->wait_for_action_server(
            std::chrono::seconds(10))) {
      // wait for action server to start for 10 seconds
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      return;
    }

    auto goal_msg = OdomRecord::Goal();
    RCLCPP_INFO(this->get_logger(), "Sending goal: Record Odom");
    auto send_goal_options =
        rclcpp_action::Client<OdomRecord>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&OdomRecordActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&OdomRecordActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&OdomRecordActionClient::result_callback, this, _1);
    this->odomrec_actionclient_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  void goal_response_callback(
      std::shared_future<GoalHandle_Odom::SharedPtr> future) {
    // check goal_handle to see whether goal is accepted or rejected by server
    auto goal_handle = future.get(); // ROS2 Foxy version instead of Humble
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
      GoalHandle_Odom::SharedPtr,
      const std::shared_ptr<const OdomRecord::Feedback> feedback) {
    // distance feedback every second
    RCLCPP_INFO(this->get_logger(), "Current Distance Travelled: %f m",
                feedback->current_total);
  }

  void result_callback(const GoalHandle_Odom::WrappedResult &result) {
    // check result.code to see what happened to our goal and print results
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
    }

    // receive result from server
    list_of_odoms = result.result->list_of_odoms;
    // casting unsigned to signed int
    int list_of_odoms_size = (int)list_of_odoms.size();
    RCLCPP_INFO(this->get_logger(), "List of odoms:");
    for (int i = 0; i < list_of_odoms_size; i++) {
      RCLCPP_INFO(this->get_logger(), "\n---\nx: %f\ny: %f\ntheta: %f\n---",
                  list_of_odoms[i].x, list_of_odoms[i].y, list_of_odoms[i].z);
    }
  }

  rclcpp_action::Client<OdomRecord>::SharedPtr odomrec_actionclient_;
  std::vector<geometry_msgs::msg::Point32> list_of_odoms;
  rclcpp::TimerBase::SharedPtr timer_;
};

bool find_wall() {
  using FindWall = custom_interfaces::srv::FindWall;

  // Method for finding wall before following wall
  bool wall = false;

  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("findwall_service_client");
  rclcpp::Client<FindWall>::SharedPtr client =
      node->create_client<FindWall>("FindWall");

  auto req = std::make_shared<FindWall::Request>();
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "service not available, waiting...");
  }

  auto result_future = client->async_send_request(req);
  // Wait for Result
  if (rclcpp::spin_until_future_complete(node, result_future) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    auto result = result_future.get();
    if (result->wallfound == true) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The robot has reached a wall");
      wall = true;
    } else {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Something went wrong");
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Failed to call service /FindWall");
  }
  return wall;
}

int main(int argc, char *argv[]) {
  // main function
  rclcpp::init(argc, argv);

  // add nodes to executor
  auto odomrecord_client = std::make_shared<OdomRecordActionClient>();
  auto wall_follower = std::make_shared<WallFollower>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(odomrecord_client);
  executor.add_node(wall_follower);

  // start findwall service client
  RCLCPP_INFO(wall_follower->get_logger(), "Finding wall...");
  bool findwall_result = find_wall();
  RCLCPP_INFO(wall_follower->get_logger(), "wall ready? %s",
              findwall_result ? "true" : "false");

  // spin executor after reaching wall
  if (findwall_result) {
    executor.spin();
  }
  rclcpp::shutdown();
  return 0;
}