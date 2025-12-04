#include <rclcpp/rclcpp.hpp>
#include "AdmittanceController.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  const double frequency = 250.0;

  // Constructing the controller
  std::shared_ptr<AdmittanceController> admittance_controller =
      std::make_shared<AdmittanceController>(frequency);
  admittance_controller->setup_moveit_servo();
  admittance_controller->wait_for_transformations();

  // Running the controller
  admittance_controller->run();

  return 0;
}
