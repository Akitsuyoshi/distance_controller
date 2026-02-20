#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <tuple>
#include <vector>

/**
 * A waypoint-based position controller using
 * independent PID loops for x, y, and yaw.
 *
 * The controller:
 *  - Subscribes to filtered odometry
 *  - Tracks a sequence of predefined waypoints
 *  - Uses a finite state machine for motion control
 *  - Publishes velocity commands to /cmd_vel
 *
 * The robot should support holonomic motion (vx, vy, wz).
 */
class DistanceController : public rclcpp::Node {
  using Twist = geometry_msgs::msg::Twist;
  using Odometry = nav_msgs::msg::Odometry;

  /**
   * Implements a discrete-time PID controller with:
   *  - Configurable gains (kp, ki, kd)
   *  - Integral windup protection (clamped integral term)
   *  - Derivative computed using backward difference
   */
  struct PID {
    // Tune param
    double kp{0.0};
    double ki{0.0};
    double kd{0.0};
    // Error
    double integral{0.0};
    double prev_err{0.0};

    void set_param(double p, double i, double d) {
      kp = p;
      ki = i;
      kd = d;
    }

    void clear_err() {
      integral = 0.0;
      prev_err = 0.0;
    }

    double compute_pid(double err, double dt) {
      integral += err * dt;
      integral = std::clamp(integral, -5.0, 5.0);
      double deriv = (dt > 0.0) ? (err - prev_err) / dt : 0.0;
      prev_err = err;
      return kp * err + ki * integral + kd * deriv;
    }
  };

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
    FAULT,     // safety stop, for future use
  };

public:
  DistanceController() : Node("distance_controller"), wp_world_(get_wp()) {
    RCLCPP_INFO(get_logger(), "Initializing node...");

    pub_ = create_publisher<Twist>("/cmd_vel", 10);
    sub_ = create_subscription<Odometry>(
        "/odometry/filtered", 10,
        [this](Odometry::SharedPtr msg) { odom_callback(msg); });
    timer_ = create_wall_timer(std::chrono::milliseconds(25),
                               [this]() { motion_callback(); });

    // Set parameters for PID
    set_param();
    last_pid_time_ = now();

    // Set first target position
    auto &wp = wp_world_[wp_idx_];
    update_target(wp.x, wp.y, wp.yaw);

    RCLCPP_INFO(get_logger(), "Initialized node");
  }

  ~DistanceController() { RCLCPP_INFO(get_logger(), "Terminated node"); }

private:
  void odom_callback(const Odometry::SharedPtr msg) {
    last_odom_time_ = now();

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
      recover_robot();
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
    // Check odom validity
    if ((now() - last_odom_time_).seconds() > odom_timeout_) {
      RCLCPP_ERROR(get_logger(), "Odometry timeout");
      state_ = State::FAULT;
      return;
    }
    // Compute velocity using PID controllers
    auto [vx, vy, wz] = get_vel_robot();
    publish_vel(vx, vy, wz);
  }

  void wait_robot() {
    stop_robot();
    if ((now() - wait_start_).seconds() >= wait_duration_) {
      state_ = State::ADVANCING;
    }
  }

  void advance_next_wp() {
    wp_idx_++;
    pid_x_.clear_err();
    pid_y_.clear_err();
    pid_yaw_.clear_err();

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
    rclcpp::shutdown();
  }

  void recover_robot() {
    stop_robot();

    // Check if odometry has been restored
    if ((now() - last_odom_time_).seconds() <= odom_timeout_) {
      RCLCPP_WARN(get_logger(), "Odometry restored. Recovering...");

      pid_x_.clear_err();
      pid_y_.clear_err();
      pid_yaw_.clear_err();
      last_pid_time_ = now();
      state_ = State::TRACKING;
    }
  }

  bool is_reached() const {
    auto [err_x, err_y, err_yaw] = get_err();
    return std::hypot(err_x, err_y) < 0.05 && std::abs(err_yaw) < 0.1;
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

  /**
   * Compute velocity command {vx, vy, wz} in robot frame:
   * - Compute position and yaw error in world frame
   * - Transform position error into robot frame
   * - Compute PID outputs for x, y, and yaw
   * - Clamp outputs to maximum velocity limits
   */
  std::tuple<double, double, double> get_vel_robot() {
    auto [dx, dy, dphi] = get_err();
    // Transform world to robot frame, using current robot yaw
    double err_x = std::cos(yaw_) * dx + std::sin(yaw_) * dy;
    double err_y = -std::sin(yaw_) * dx + std::cos(yaw_) * dy;

    rclcpp::Time now_t = now();
    double dt = (now_t - last_pid_time_).seconds();
    // Clamp dt to prevent instability from large timing gaps
    dt = std::clamp(dt, 0.001, 0.1);
    last_pid_time_ = now_t;

    // Compute pid with feed-forward for each x, y, and yaw
    double vx = pid_x_.compute_pid(err_x, dt) + 0.2 * err_x;
    double vy = pid_y_.compute_pid(err_y, dt) + 0.2 * err_y;
    double wz = pid_yaw_.compute_pid(dphi, dt) + 0.1 * dphi;

    vx = std::clamp(vx, -max_v_, max_v_);
    vy = std::clamp(vy, -max_v_, max_v_);
    wz = std::clamp(wz, -max_w_, max_w_);

    return {vx, vy, wz};
  }

  void publish_vel(double vx, double vy, double wz) const {
    Twist cmd;
    cmd.linear.x = vx;
    cmd.linear.y = vy;
    cmd.angular.z = wz;
    pub_->publish(cmd);
  }

  std::vector<Waypoint> get_wp() const {
    // Waypoints are defined in world frame (x, y, yaw)
    return {{0.0, 1.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, -1.0, 0.0}, {0.0, 0.0, 0.0},
            {1.0, 1.0, 0.0}, {0.0, 0.0, 0.0}, {1.0, -1.0, 0.0}, {0.0, 0.0, 0.0},
            {1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
  }

  void set_param() {
    max_v_ = declare_parameter<double>("max_v_", 0.6);
    max_w_ = declare_parameter<double>("max_w_", 1.2);
    pid_x_.set_param(declare_parameter<double>("pid_x.kp", 1.2),
                     declare_parameter<double>("pid_x.ki", 0.01),
                     declare_parameter<double>("pid_x.kd", 0.05));

    pid_y_.set_param(declare_parameter<double>("pid_y.kp", 1.2),
                     declare_parameter<double>("pid_y.ki", 0.01),
                     declare_parameter<double>("pid_y.kd", 0.05));

    pid_yaw_.set_param(declare_parameter<double>("pid_yaw.kp", 1.5),
                       declare_parameter<double>("pid_yaw.ki", 0.0),
                       declare_parameter<double>("pid_yaw.kd", 0.1));
  }

  rclcpp::Publisher<Twist>::SharedPtr pub_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time last_odom_time_;
  double odom_timeout_{0.5}; // in second

  State state_{State::BOOTSTRAP};
  rclcpp::Time wait_start_;
  double wait_duration_{2.0}; // in second

  PID pid_x_{};
  PID pid_y_{};
  PID pid_yaw_{};
  rclcpp::Time last_pid_time_;
  double max_v_{0.0};
  double max_w_{0.0};

  std::vector<Waypoint> wp_world_{};
  size_t wp_idx_{0};

  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};

  double target_x_{0.0};
  double target_y_{0.0};
  double target_yaw_{0.0};
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DistanceController>());
  rclcpp::shutdown();
  return 0;
}