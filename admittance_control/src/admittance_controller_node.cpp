#include <rclcpp/rclcpp.hpp>
#include "AdmittanceController.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  const double frequency = 100.0;

  // Constructing the controller
  AdmittanceController admittance_controller(frequency);
  admittance_controller.setup_moveit_servo();

  // Running the controller
  admittance_controller.run();

  return 0;
}
