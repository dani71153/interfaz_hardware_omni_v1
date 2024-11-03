#ifndef OMNI_ROBOT_HARDWARE_INTERFACE_HPP_
#define OMNI_ROBOT_HARDWARE_INTERFACE_HPP_

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <vector>

namespace omni_robot_hardware
{
class OmniRobotHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(OmniRobotHardwareInterface);

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
  hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  // Serial connections
  serial::Serial serial_arduino_;
  serial::Serial serial_esp_;

  // Velocity commands and encoder feedback
  std::vector<double> velocity_commands_;
  std::vector<double> encoder_readings_;

  // Hardware parameters
  std::string arduino_port_;
  std::string esp_port_;
  unsigned long baud_rate_;
  int serial_timeout_;
  double publish_frequency_;

  // Topic and node names
  std::string node_name_;
  std::string velocity_topic_;
  std::string state_topic_;

  // Speed limits
  double min_rps_;
  double max_rps_;

  // Encoder adjustment factors for each wheel
  std::vector<double> encoder_adjustments_;

  // Helper function to limit velocity
  double clamp_velocity(double velocity);
};

}  // namespace omni_robot_hardware

#endif  // OMNI_ROBOT_HARDWARE_INTERFACE_HPP_
