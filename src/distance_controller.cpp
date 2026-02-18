#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include <cmath>
#include <cstddef>
#include <tuple>
#include <vector>

class DistanceController : public rclcpp::Node {
  using Twist = geometry_msgs::msg::Twist;
  using Odometry = nav_msgs::msg::Odometry;

  struct Waypoint {
    double x;
    double y;
    double yaw;
  };

  enum class State {
    BOOTSTRAP, // wait for odom
    TRACKING,  // drive to target position
    WAITING,   // wait between track and advance
    ADVANCING, // load next waypoint
    FINISHED,  // stop after finishing all waypoint
    FAULT,     // safety stop
  };

public:
  DistanceController()
      : Node("distance_controller"), state_(State::BOOTSTRAP),
        wait_duration_(2.0), wp_world_(get_wp()), wp_idx_(0) {
    RCLCPP_INFO(get_logger(), "Initializing node...");

    pub_ = create_publisher<Twist>("/cmd_vel", 10);
    sub_ = create_subscription<Odometry>(
        "/odometry/filtered", 10,
        [this](Odometry::SharedPtr msg) { odom_callback(msg); });
    timer_ = create_wall_timer(std::chrono::milliseconds(25),
                               [this]() { motion_callback(); });

    update_position(0.0, 0.0, 0.0);
    auto &wp = wp_world_[wp_idx_];
    update_target(wp.x, wp.y, wp.yaw);

    RCLCPP_INFO(get_logger(), "Initialized node");
  }

  ~DistanceController() { RCLCPP_INFO(get_logger(), "Terminated node"); }

private:
  void odom_callback(const Odometry::SharedPtr msg) {
    auto &position = msg->pose.pose.position;
    auto &orientation = msg->pose.pose.orientation;
    tf2::Quaternion q(orientation.x, orientation.y, orientation.z,
                      orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    update_position(position.x, position.y, yaw);

    if (state_ == State::BOOTSTRAP) {
      state_ = State::TRACKING;
      RCLCPP_INFO(get_logger(), "Moving to the waypoint %zu, [%f, %f, %f]",
                  wp_idx_, target_x_, target_y_, target_yaw_);
    }
  }

  void motion_callback() {
    switch (state_) {
    case State::BOOTSTRAP:
      stop_robot();
      break;
    case State::TRACKING:
      move_robot();
      break;
    case State::WAITING:
      wait_robot();
      break;
    case State::ADVANCING:
      advance_next_wp();
      break;
    case State::FINISHED:
      finish_task();
      break;
    case State::FAULT:
      stop_robot();
      break;
    }
  }

  void stop_robot() const { publish_vel(0.0, 0.0, 0.0); }

  void move_robot() {
    if (is_reached()) {
      stop_robot();
      wait_start_ = now();
      state_ = State::WAITING;
      return;
    }
    auto [vx, vy, vz] = get_vel_robot();
    publish_vel(vx, vy, vz);
  }

  void wait_robot() {
    stop_robot();
    if ((now() - wait_start_).seconds() >= wait_duration_) {
      state_ = State::ADVANCING;
    }
  }

  void advance_next_wp() {
    wp_idx_++;
    if (wp_idx_ >= wp_world_.size()) {
      stop_robot();
      state_ = State::FINISHED;
      return;
    }
    auto &wp = wp_world_[wp_idx_];
    update_target(wp.x, wp.y, wp.yaw);
    state_ = State::TRACKING;
    RCLCPP_INFO(get_logger(), "Moving to the waypoint %zu, [%f, %f, %f]",
                wp_idx_, target_x_, target_y_, target_yaw_);
  }

  void finish_task() {
    stop_robot();
    timer_->cancel();
    RCLCPP_INFO(get_logger(), "Finished moving the all waypoints");
  }

  bool is_reached() const {
    auto [err_x, err_y, err_yaw] = get_err();
    return std::hypot(err_x, err_y) < 0.03 && std::abs(err_yaw) < 0.06;
  }

  std::tuple<double, double, double> get_err() const {
    double err_x = target_x_ - x_;
    double err_y = target_y_ - y_;
    double err_yaw = normalize_angle(target_yaw_ - yaw_);
    return {err_x, err_y, err_yaw};
  }

  double normalize_angle(double angle) const {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  void update_position(double x, double y, double yaw) {
    x_ = x;
    y_ = y;
    yaw_ = yaw;
  }

  void update_target(double x, double y, double yaw) {
    target_x_ = x;
    target_y_ = y;
    target_yaw_ = yaw;
  }

  std::tuple<double, double, double> get_vel_robot() const {
    auto [dx, dy, dphi] = get_err();
    double vx = std::cos(yaw_) * dx + std::sin(yaw_) * dy;
    double vy = -std::sin(yaw_) * dx + std::cos(yaw_) * dy;
    return {vx, vy, dphi};
  }

  void publish_vel(double vx, double vy, double vz) const {
    Twist cmd;
    cmd.linear.x = vx;
    cmd.linear.y = vy;
    cmd.angular.z = vz;
    pub_->publish(cmd);
  }

  std::vector<Waypoint> get_wp() const {
    return {{0.0, 1.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, -1.0, 0.0}, {0.0, 0.0, 0.0},
            {1.0, 1.0, 0.0}, {0.0, 0.0, 0.0}, {1.0, -1.0, 0.0}, {0.0, 0.0, 0.0},
            {1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
  }

  rclcpp::Publisher<Twist>::SharedPtr pub_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  State state_;
  rclcpp::Time wait_start_;
  double wait_duration_; // in seconds

  std::vector<Waypoint> wp_world_;
  size_t wp_idx_;

  double x_;
  double y_;
  double yaw_;
  double target_x_;
  double target_y_;
  double target_yaw_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DistanceController>());
  rclcpp::shutdown();
  return 0;
}