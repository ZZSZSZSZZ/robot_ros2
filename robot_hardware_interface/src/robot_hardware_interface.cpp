#include <limits>
#include <vector>

#include "robot_hardware_interface/robot_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot_hardware_interface{
hardware_interface::CallbackReturn RobotHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  robot_ = std::make_unique<robot::Robot>(8080);

  const size_t total_joints = info_.joints.size();
  pos_commands_.resize(total_joints, 0.0);
  vel_commands_.resize(total_joints, 0.0);
  pos_states_.resize(total_joints, 0.0);
  vel_states_.resize(total_joints, 0.0);

  if (robot_->init_robot()){
    return CallbackReturn::SUCCESS;
  }

  return CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn RobotHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_pos_states_.resize(info_.joints.size(), 0.0);
  joint_vel_states_.resize(info_.joints.size(), 0.0);

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobotHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,&pos_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,&vel_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn RobotHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  pos_commands_ = pos_states_;
  joint_pos_states_ = pos_states_;

  vel_commands_ = vel_states_;
  joint_vel_states_ = vel_states_;

  // if (robot_->enable_robot()){
  //   return CallbackReturn::SUCCESS;
  // }
  // return CallbackReturn::ERROR

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // if (robot_->disable_robot()){
  //   return CallbackReturn::SUCCESS;
  // }
  // return CallbackReturn::ERROR

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type RobotHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // pos_states_ = joint_pos_states_;
  vel_states_ = joint_vel_states_;

  if (robot_->read(&pos_states_)){
    printf("read");
    for (auto i = 0ul; i < pos_states_.size(); i++){
    printf("%f\n", pos_states_[i]);
    // printf("%f\n", vel_states_[i]);
    }
    
    return hardware_interface::return_type::OK;
  }

  return hardware_interface::return_type::ERROR;
}

hardware_interface::return_type RobotHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // joint_pos_states_ = pos_commands_;
  joint_vel_states_ = vel_commands_;

  // printf("write");
  // for (auto i = 0ul; i < joint_pos_states_.size(); i++){
  //   printf("%f\n", pos_commands_[i]);
  //   printf("%f\n", vel_commands_[i]);
  // }

  if (robot_->write(pos_commands_)){
    
    return hardware_interface::return_type::OK;
  }

  return hardware_interface::return_type::ERROR;
}

}  // namespace robot_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  robot_hardware_interface::RobotHardwareInterface, hardware_interface::SystemInterface)
