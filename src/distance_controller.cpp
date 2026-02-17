#include "rclcpp/rclcpp.hpp"

class DistanceController : public rclcpp::Node {
public:
  DistanceController() : Node("distance_controller") {
    RCLCPP_INFO(get_logger(), "Initialized node");
  }

  ~DistanceController() { RCLCPP_INFO(get_logger(), "Terminated node"); }

private:
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DistanceController>());
  rclcpp::shutdown();
  return 0;
}