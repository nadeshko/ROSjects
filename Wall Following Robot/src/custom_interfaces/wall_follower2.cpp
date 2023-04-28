#include "custom_interfaces/action/detail/odom_record__struct.hpp"
#include "custom_interfaces/action/odom_record.hpp"
#include "custom_interfaces/srv/find_wall.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/node_interfaces/get_node_base_interface.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/client_goal_handle.hpp"
#include "rclcpp_action/create_client.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <chrono>
#include <cstdio>
#include <memory>

using FindWall = custom_interfaces::srv::FindWall;
using namespace std::chrono_literals;
using std::placeholders::_1;

class WallFollower : public rclcpp ::Node {
  // wall follower construct
public:
  WallFollower() : Node("wall_follower") {
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
  // class aliases
  using OdomRecord_Action = custom_interfaces::action::OdomRecord;
  using GoalHandle_Odom = rclcpp_action::ClientGoalHandle<OdomRecord_Action>;

  explicit OdomRecordActionClient(
      const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
      : Node("odom_record_action_client", node_options), goal_done_(false) {
    // action client
    this->odomrecord_action_client_ =
        rclcpp_action::create_client<OdomRecord_Action>(
            this->get_node_base_interface(), this->get_node_graph_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(), "record_odom");

    this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&OdomRecordActionClient::send_goal, this));
  }

  bool is_goal_done() const { return this->goal_done_; }

  void send_goal() {
    using namespace std::placeholders;

    // cancel timer so that it only gets executed 1 time
    // because we dont want to keep sending goal to action server
    this->timer_->cancel();
    this->goal_done_ = false;

    if (!this->odomrecord_action_client_) { // check if action server is running
      RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }

    if (!this->odomrecord_action_client_->wait_for_action_server(
            std::chrono::seconds(10))) {
      // wait for action server to start for 10 seconds, if not ready after 10
      // sec we pass
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      this->goal_done_ = true;
      return;
    }

    auto goal_msg = OdomRecord_Action::Goal(); // goal object = *blank*
    RCLCPP_INFO(this->get_logger(), "Sending goal to call action server");

    // goal options and sending goal
    auto send_goal_options =
        rclcpp_action::Client<OdomRecord_Action>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&OdomRecordActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&OdomRecordActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&OdomRecordActionClient::result_callback, this, _1);
    auto goal_handle_future = this->odomrecord_action_client_->async_send_goal(
        goal_msg, send_goal_options);
  }

private:
  void goal_response_callback(const GoalHandle_Odom::SharedPtr future) {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
      GoalHandle_Odom::SharedPtr,
      const std::shared_ptr<const OdomRecord_Action::Feedback> feedback) {
    // distance travelled feedback every second
    RCLCPP_INFO(this->get_logger(), "Distance Travelled: %fm",
                feedback->current_total);
  }

  void result_callback(const GoalHandle_Odom::WrappedResult &result) {
    this->goal_done_ = true;

    // check result.code variable to see what happened to our goal and print
    // results
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
    std::vector<geometry_msgs::msg::Point32> list_of_pos;

    for (auto number : result.result->list_of_odoms) {
      RCLCPP_INFO(this->get_logger(), "Odom List; x = %f, y %f, theta = %f",
                  result.result->list_of_odoms[0],
                  result.result->list_of_odoms[1],
                  result.result->list_of_odoms[2]);
    }
  }

  rclcpp_action::Client<OdomRecord_Action>::SharedPtr odomrecord_action_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_done_;
};

bool find_wall() {
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

  // create action client, executor and spin it
  auto odomrecord_client = std::make_shared<OdomRecordActionClient>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(odomrecord_client);
  executor.spin();

  // find wall if robot is not near wall, if ready, spin node
  bool findwall_result = find_wall();
  std::printf("wall ready: %s", findwall_result ? "true" : "false");
  if (findwall_result) {
    rclcpp::spin(std::make_shared<WallFollower>());
  }
  rclcpp::shutdown();
  return 0;
}

// First, you should call the action server to start recording odometry data.
// As mentioned in the course, you can run other nodes while running your
// action. so, after you run the action server and action client, you can run
// your service server and call it. as a result, your action and service will
// run simultaneously.

// you can also first start the action server and service server, then call
// them by action client and service client in order.