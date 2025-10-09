#pragma once

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <string>
#include <vector>

namespace dog_controllers
{

class DogLegController : public controller_interface::ControllerInterface
{
public:
  // Основной инициализационный метод (теперь называется on_init)
  controller_interface::CallbackReturn on_init() override;

  // Какие интерфейсы команд нужны контроллеру
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  // Какие интерфейсы состояний (датчиков) он будет читать
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  // Основной цикл обновления
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<hardware_interface::LoanedCommandInterface> joint_command_interfaces_;
  std::vector<hardware_interface::LoanedStateInterface> joint_state_interfaces_;
};

}  // namespace dog_controllers
