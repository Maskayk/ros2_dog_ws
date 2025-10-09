#include "dog_controllers/dog_leg_controller.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace dog_controllers
{

controller_interface::CallbackReturn DogLegController::on_init()
{
  RCLCPP_INFO(get_node()->get_logger(), "DogLegController initialized");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration DogLegController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = {
    "front_left_hip_joint/position",
    "front_left_knee_joint/position",
    "front_left_shoulder_joint/position"
  };
  return config;
}

controller_interface::InterfaceConfiguration DogLegController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = {
    "front_left_hip_joint/position",
    "front_left_knee_joint/position",
    "front_left_shoulder_joint/position"
  };
  return config;
}

controller_interface::return_type DogLegController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  RCLCPP_DEBUG(get_node()->get_logger(), "DogLegController updating...");

  // Здесь будет логика управления моторами лапы
  // Например: вычисление углов из IK и запись команд в joint_command_interfaces_

  return controller_interface::return_type::OK;
}

}  // namespace dog_controllers

PLUGINLIB_EXPORT_CLASS(
  dog_controllers::DogLegController,
  controller_interface::ControllerInterface)
