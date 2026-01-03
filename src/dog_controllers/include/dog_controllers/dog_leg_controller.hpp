#pragma once

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <string>
#include <vector>
#include "dog_controllers/leg_kinematics.hpp"
#include "dog_controllers/foot_trajectory.hpp"
#include "dog_controllers/gait_generator.hpp"


namespace dog_controllers
{

class DogLegController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  std::vector<hardware_interface::LoanedCommandInterface> command_interfaces_;
  std::vector<hardware_interface::LoanedStateInterface> state_interfaces_;
  // твои структуры
  LegKinematics3DOF kinematics_{0.1525, 0.144};
  FootTrajectory trajectory_;
  GaitGenerator gait_;
};
}