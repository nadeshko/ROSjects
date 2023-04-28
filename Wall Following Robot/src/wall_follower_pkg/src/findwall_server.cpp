#include "custom_interfaces/srv/find_wall.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <chrono>
#include <memory>
#include <thread>

using FindWall = custom_interfaces::srv::FindWall;
using std::placeholders::_1;
using std::placeholders::_2;

class FindWall_Server : public rclcpp::Node {
public:
  FindWall_Server() : Node("findwall_server") {
    // Initializations
    this->laser_front = 0.0;
    this->laser_right = 0.0;
    this->laser_min = 999.0;

    // call-back groups
    grp_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions opt;
    opt.callback_group = grp_;

    // Service with a callback function and group for finding a wall
    srv_ = create_service<FindWall>(
        "FindWall",
        std::bind(&FindWall_Server::findwall_callback, this, _1, _2),
        ::rmw_qos_profile_default, grp_);

    // Publisher to send velocity command to robot
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Subscriber for receiving laser scan data from robot
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS(),
        std::bind(&FindWall_Server::laser_callback, this, _1), opt);
  }

private:
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr laser) {
    // get laser data (front, right, and minimum (ideally left too))
    this->laser_right = laser->ranges[180];
    this->laser_front = laser->ranges[360];
    for (int i = 0; i <= 719; i++) {
      if (laser->ranges[i] <= this->laser_min) {
        this->laser_min = laser->ranges[i];
      }
    }
  }

  void findwall_callback(const std::shared_ptr<FindWall::Request> request,
                         const std::shared_ptr<FindWall::Response> response) {
    // check if we're facing or near a wall, or at a corner
    RCLCPP_INFO(this->get_logger(), "[FRONT]= [%f]", this->laser_front);
    RCLCPP_INFO(this->get_logger(), "[RIGHT]= [%f]", this->laser_right);
    if (this->laser_right < 0.5) {
      if (this->laser_front < 0.5) {
        at_corner = true;
      }
      wall_ready = true;
      wall_found = true;
      wall_reached = true;
    } else {
      wall_ready = false;
      if (this->laser_front < 0.3) {
        wall_found = true;
        wall_reached = true;
      } else {
        wall_found = false;
        wall_reached = false;
      }
    }

    // print status and wait so viewer can read
    RCLCPP_INFO(this->get_logger(), "wall_found= %s",
                wall_found ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "wall_reached= %s",
                wall_reached ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "wall_ready= %s",
                wall_ready ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "at_corner= %s",
                at_corner ? "true" : "false");
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // find closest wall by turning robot
    while (!wall_found) {
      RCLCPP_INFO(this->get_logger(), "[F, MIN]= [%f], [%f]", this->laser_front,
                  this->laser_min);
      if (this->laser_front < this->laser_min + 0.15) {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        wall_found = true;
        RCLCPP_INFO(this->get_logger(), "wall found!");
      } else {
        cmd.angular.z = 0.1;
        wall_found = false;
      }
      publisher_->publish(this->cmd);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // once wall is found, head towards it until front laser is < 0.3m
    while (!wall_reached) {
      RCLCPP_INFO(this->get_logger(), "[FRONT]= [%f]", this->laser_front);
      if (this->laser_front < 0.3) {
        cmd.linear.x = 0.0;
        wall_reached = true;
        RCLCPP_INFO(this->get_logger(), "wall reached!");
      } else {
        cmd.linear.x = 0.08;
        wall_reached = false;
      }
      publisher_->publish(this->cmd);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // turn until right laser is 0.35m (give way to robot side)
    while (!wall_ready) {
      RCLCPP_INFO(this->get_logger(), "[RIGHT]= [%f]", this->laser_right);
      if (this->laser_right < 0.35) {
        cmd.angular.z = 0.0;
        wall_ready = true;
      } else {
        cmd.angular.z = 0.1;
        wall_ready = false;
      }
      publisher_->publish(this->cmd);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // check again if robot is at corner
    if (this->laser_right < 0.5 && this->laser_front < 0.5) {
      at_corner = true;
      RCLCPP_INFO(this->get_logger(), "robot at corner!");
    } else {
      at_corner = false;
    }

    // if at corner, turn robot until front laser has enough space
    while (at_corner) {
      RCLCPP_INFO(this->get_logger(), "[FRONT]= [%f]", this->laser_front);
      if (this->laser_front < 0.6) {
        cmd.angular.z = 0.1;
        at_corner = true;
      } else {
        cmd.angular.z = 0.0;
        RCLCPP_INFO(this->get_logger(), "got out of corner!");
        at_corner = false;
      }
      publisher_->publish(this->cmd);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // service is complete once wall_ready and not at corner
    if (wall_ready && !at_corner) {
      // send response back to client
      RCLCPP_INFO(this->get_logger(), "ready to follow wall!");
      response->wallfound = true;
    } else {
      response->wallfound = false;
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Service<FindWall>::SharedPtr srv_;
  rclcpp::CallbackGroup::SharedPtr grp_;
  geometry_msgs::msg::Twist cmd;

  // values
  float laser_front;
  float laser_right;
  float laser_min;
  bool wall_found;
  bool wall_reached;
  bool wall_ready;
  bool at_corner;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // create node
  std::shared_ptr<FindWall_Server> findwall_server_node =
      std::make_shared<FindWall_Server>();
  // Initialize one MultiThreadedExecutor object
  rclcpp::executors::MultiThreadedExecutor executor;
  // Add node to executor
  executor.add_node(findwall_server_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}