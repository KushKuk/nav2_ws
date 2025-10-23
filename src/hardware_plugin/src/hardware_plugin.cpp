#include "hardware_plugin/hardware_plugin.hpp"
#include "pluginlib/class_list_macros.hpp"

FortressRoverHardware::FortressRoverHardware() {}

CallbackReturn FortressRoverHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  wheel_pos_ = 0.0;
  wheel_vel_ = 0.0;
  wheel_cmd_ = 0.0;

  RCLCPP_INFO(rclcpp::get_logger("FortressRoverHardware"), "Hardware initialized successfully.");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FortressRoverHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back("wheel", "position", &wheel_pos_);
  state_interfaces.emplace_back("wheel", "velocity", &wheel_vel_);
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FortressRoverHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back("wheel", "velocity", &wheel_cmd_);
  return command_interfaces;
}

hardware_interface::return_type FortressRoverHardware::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  (void)period;
  wheel_vel_ = wheel_cmd_;
  wheel_pos_ += wheel_vel_ * period.seconds();  // basic integration
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FortressRoverHardware::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  (void)period;
  // send wheel_cmd_ to hardware (if applicable)
  return hardware_interface::return_type::OK;
}


// ✅ Register the class with pluginlib
PLUGINLIB_EXPORT_CLASS(FortressRoverHardware, hardware_interface::SystemInterface)
