#include "lidardrive/vm.hpp"
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<VM>();
  rclcpp::spin(node);
  rclcpp::shutdown();


  return 0;
}
