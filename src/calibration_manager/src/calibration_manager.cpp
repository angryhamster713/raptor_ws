#include "rclcpp/rclcpp.hpp"
#include "calibration_manager/CalibrateAxis.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto n = rclcpp::Node::make_shared("calibration_manager");

  CalibrateAxis mCalibrateAxis(n);

  RCLCPP_INFO(n->get_logger(), "Calibration Manager node started.");

  rclcpp::spin(n);
  rclcpp::shutdown();
  return 0;
}
